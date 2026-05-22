#include <chrono>
#include <iostream>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "mujoco_sim.h"
#include "reachy_sim.grpc.pb.h"
#include <grpcpp/grpcpp.h>

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
  ReachySimServiceImpl() {
    target_joints_.assign(9, 0.0f);
  }

  Status
  ControlStream(ServerContext *context,
                ServerReaderWriter<SimState, ControlRequest> *stream) override {
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

      SimState res;
      res.set_status("OK");
      stream->Write(res);
    }
    return Status::OK;
  }

  std::vector<double> getTargetJoints() {
    std::lock_guard<std::mutex> lock(mtx_);
    return target_joints_;
  }

private:
  std::mutex mtx_;
  std::vector<double> target_joints_;
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
