cmake_minimum_required(VERSION 3.10)
project(reachy_mini_mujoco_sim C CXX)

set(CMAKE_C_STANDARD 99)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

# ── Find Packages (针对开发板极简优化) ──────────────────────
find_package(PkgConfig REQUIRED)

# 使用 PkgConfig 查找所有依赖，这在 Linux 板卡上通常是最快路径
pkg_check_modules(PROTOBUF REQUIRED protobuf)
pkg_check_modules(GRPC REQUIRED grpc++ grpc)

# 直接查找插件，避免显式 PATHS 导致的大规模磁盘扫描
find_program(GRPC_CPP_PLUGIN_EXE NAMES grpc_cpp_plugin)
if(NOT GRPC_CPP_PLUGIN_EXE)
    message(FATAL_ERROR "找不到 grpc_cpp_plugin 插件")
endif()

# 设置变量以兼容后续逻辑
set(GRPC_LIBRARIES ${GRPC_LIBRARIES})
set(PROTOBUF_LIBRARIES ${PROTOBUF_LIBRARIES})

# ── Protobuf & gRPC 代码生成 ─────────────────────────────
set(PROTO_FILE    "${CMAKE_CURRENT_SOURCE_DIR}/reachy_sim.proto")
set(PROTO_GEN_DIR "${CMAKE_CURRENT_BINARY_DIR}/gen")
file(MAKE_DIRECTORY ${PROTO_GEN_DIR})

set(PROTO_HDRS "${PROTO_GEN_DIR}/reachy_sim.pb.h" "${PROTO_GEN_DIR}/reachy_sim.grpc.pb.h")
set(PROTO_SRCS "${PROTO_GEN_DIR}/reachy_sim.pb.cc" "${PROTO_GEN_DIR}/reachy_sim.grpc.pb.cc")

add_custom_command(
    OUTPUT ${PROTO_HDRS} ${PROTO_SRCS}
    COMMAND protoc # 直接使用系统路径中的 protoc
    ARGS --cpp_out=${PROTO_GEN_DIR}
         --grpc_out=${PROTO_GEN_DIR}
         --plugin=protoc-gen-grpc=${GRPC_CPP_PLUGIN_EXE}
         --proto_path=${CMAKE_CURRENT_SOURCE_DIR}
         ${PROTO_FILE}
    DEPENDS ${PROTO_FILE}
    COMMENT "Generating gRPC and Protobuf code from ${PROTO_FILE}"
)

# ── Targets ──────────────────────────────────────────────

# 设置全局 PIC (解决 RISC-V 上的重定位错误)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# 通用包含路径
include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/simulation_server/kinematics
    ${PROTO_GEN_DIR}
    ${PROTOBUF_INCLUDE_DIRS}
    ${GRPC_INCLUDE_DIRS}
)

# Kinematics 源文件 (IK 在客户端完成)
set(KINEMATICS_SRC "${CMAKE_CURRENT_SOURCE_DIR}/simulation_server/kinematics/reachy_kinematics.c")

# 3. control_client (解耦控制客户端，含 IK 解算)
add_executable(control_client control_client.cc ${KINEMATICS_SRC} ${PROTO_SRCS})

# 链路聚合搜索路径
link_directories(${PROTOBUF_LIBRARY_DIRS} ${GRPC_LIBRARY_DIRS})

# 链接库 (注意变量大小写必须与 pkg_check_modules 的前缀一致)
target_link_libraries(control_client 
    ${GRPC_LIBRARIES} 
    ${PROTOBUF_LIBRARIES} 
    m pthread
)

# ── Installation ─────────────────────────────────────────
install(TARGETS control_client DESTINATION bin)
