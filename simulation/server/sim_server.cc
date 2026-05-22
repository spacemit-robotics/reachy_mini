#include <grpcpp/grpcpp.h>
#include <algorithm>
#include <chrono>
#include <condition_variable>
#include <iostream>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>


#include "mujoco_sim.h"
#include "reachy_sim.grpc.pb.h"

using grpc::Server;
using grpc::ServerBuilder;
using grpc::ServerContext;
using grpc::ServerReaderWriter;
using grpc::Status;
using reachy::sim::ControlRequest;
using reachy::sim::ReachySimService;
using reachy::sim::SimState;

// -----------------------------------------------------------------------------
// Service Implementation
// -----------------------------------------------------------------------------
class ReachySimServiceImpl final : public ReachySimService::Service {
public:
    ReachySimServiceImpl() : running_(true) {
        target_joints_.assign(9, 0.0f);
        publisher_thread_ = std::thread(&ReachySimServiceImpl::publisherLoop, this);
    }

    ~ReachySimServiceImpl() override {
        {
            std::lock_guard<std::mutex> lock(state_mtx_);
            running_ = false;
        }
        cv_.notify_all();
        if (publisher_thread_.joinable()) {
            publisher_thread_.join();
        }
    }

    Status
    ControlStream(ServerContext *context,
                    ServerReaderWriter<SimState, ControlRequest> *stream) override {
        {
            std::lock_guard<std::mutex> lock(stream_mtx_);
            streams_.push_back(stream);
        }

        ControlRequest req;
        while (stream->Read(&req)) {
            {
                std::lock_guard<std::mutex> lock(mtx_);
                target_joints_[0] = req.body_yaw();
                target_joints_[1] = req.stewart_1();
                target_joints_[2] = req.stewart_2();
                target_joints_[3] = req.stewart_3();
                target_joints_[4] = req.stewart_4();
                target_joints_[5] = req.stewart_5();
                target_joints_[6] = req.stewart_6();
                target_joints_[7] = req.right_antenna();
                target_joints_[8] = req.left_antenna();
            }
        }

        {
            std::lock_guard<std::mutex> lock(stream_mtx_);
            auto it = std::find(streams_.begin(), streams_.end(), stream);
            if (it != streams_.end()) {
                streams_.erase(it);
            }
        }
        return Status::OK;
    }

    std::vector<double> getTargetJoints() {
        std::lock_guard<std::mutex> lock(mtx_);
        return target_joints_;
    }

    void pushState(const SimState& state) {
        {
            std::lock_guard<std::mutex> lock(state_mtx_);
            latest_state_ = state;
            has_new_state_ = true;
        }
        cv_.notify_one();
    }

private:
    void publisherLoop() {
        while (true) {
            SimState state;
            {
                std::unique_lock<std::mutex> lock(state_mtx_);
                cv_.wait(lock, [this]() { return has_new_state_ || !running_; });
                if (!running_) {
                    break;
                }
                state = latest_state_;
                has_new_state_ = false;
            }

            std::lock_guard<std::mutex> lock(stream_mtx_);
            for (auto* stream : streams_) {
                stream->Write(state);
            }
        }
    }

    std::mutex mtx_;
    std::vector<double> target_joints_;

    std::mutex stream_mtx_;
    std::vector<ServerReaderWriter<SimState, ControlRequest>*> streams_;

    std::mutex state_mtx_;
    std::condition_variable cv_;
    SimState latest_state_;
    bool has_new_state_ = false;
    bool running_ = true;
    std::thread publisher_thread_;
};

// -----------------------------------------------------------------------------
// Main
// -----------------------------------------------------------------------------
int main(int argc, char **argv) {
    std::string server_address("0.0.0.0:50051");
    std::string yaml_path = "reachy.yaml";
    std::string xml_path = "robot.xml";

    if (argc > 1) server_address = argv[1];
    if (argc > 2) yaml_path = argv[2];
    if (argc > 3) xml_path = argv[3];

    if (server_address.find(":") == std::string::npos) {
        server_address += ":50051";
    }

    ReachySimServiceImpl service;

    ServerBuilder builder;
    builder.AddListeningPort(server_address, grpc::InsecureServerCredentials());
    builder.RegisterService(&service);
    std::unique_ptr<Server> server(builder.BuildAndStart());
    if (!server) {
        std::cerr << "[!] Failed to start gRPC server" << std::endl;
        return 1;
    }
    std::cout << "[*] Server listening on " << server_address << std::endl;

    try {
        mujoco_sim::Simulator simulator(yaml_path, "reachy_mini", 9, xml_path, true);

        mujoco_sim::StepFn step_cb = [&](const mujoco_sim::SimState& state) -> std::optional<mujoco_sim::SimControl> {
            // 每一仿真步主动向客户端推送状态，包括当前的仿真时间
            SimState proto_state;
            proto_state.set_sim_time(state.time);
            proto_state.set_status("Running");
            service.pushState(proto_state);

            mujoco_sim::SimControl ctrl;
            ctrl.enable = true;
            ctrl.target_pos = service.getTargetJoints();
            return ctrl;
        };

        simulator.Run(step_cb, nullptr, -1);
    } catch (const std::exception& e) {
        std::cerr << "[!] Simulator error: " << e.what() << std::endl;
    }

    server->Shutdown();
    return 0;
}
