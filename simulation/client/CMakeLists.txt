# ── Find Packages (针对开发板极简优化) ──────────────────────
find_package(PkgConfig REQUIRED)
pkg_check_modules(PROTOBUF REQUIRED protobuf)
pkg_check_modules(GRPC REQUIRED grpc++ grpc)

find_program(GRPC_CPP_PLUGIN_EXE NAMES grpc_cpp_plugin)
if(NOT GRPC_CPP_PLUGIN_EXE)
    message(FATAL_ERROR "找不到 grpc_cpp_plugin 插件")
endif()

# ── Protobuf & gRPC 代码生成 ─────────────────────────────
set(PROTO_FILE    "${CMAKE_CURRENT_SOURCE_DIR}/../reachy_sim.proto")
set(PROTO_GEN_DIR "${CMAKE_CURRENT_BINARY_DIR}/gen_sim")
file(MAKE_DIRECTORY ${PROTO_GEN_DIR})

set(PROTO_HDRS "${PROTO_GEN_DIR}/reachy_sim.pb.h" "${PROTO_GEN_DIR}/reachy_sim.grpc.pb.h")
set(PROTO_SRCS "${PROTO_GEN_DIR}/reachy_sim.pb.cc" "${PROTO_GEN_DIR}/reachy_sim.grpc.pb.cc")

add_custom_command(
    OUTPUT ${PROTO_HDRS} ${PROTO_SRCS}
    COMMAND protoc
    ARGS --cpp_out=${PROTO_GEN_DIR}
         --grpc_out=${PROTO_GEN_DIR}
         --plugin=protoc-gen-grpc=${GRPC_CPP_PLUGIN_EXE}
         --proto_path=${CMAKE_CURRENT_SOURCE_DIR}/..
         ${PROTO_FILE}
    DEPENDS ${PROTO_FILE}
    COMMENT "Generating gRPC and Protobuf code from ${PROTO_FILE}"
)

# 收敛 include 路径到本目录，避免污染主 CMakeLists
include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}/..
    ${PROTO_GEN_DIR}
    ${PROTOBUF_INCLUDE_DIRS}
    ${GRPC_INCLUDE_DIRS}
    ${CMAKE_SOURCE_DIR}/src/kinematics/include
)

link_directories(${PROTOBUF_LIBRARY_DIRS} ${GRPC_LIBRARY_DIRS})

# 3. control_client (解耦控制客户端，含 IK 解算)
# 使用主工程的 reachy_kinematics.c
add_executable(control_client control_client.cc ${CMAKE_SOURCE_DIR}/src/kinematics/src/reachy_kinematics.c ${PROTO_SRCS})

target_link_libraries(control_client 
    ${GRPC_LIBRARIES} 
    ${PROTOBUF_LIBRARIES} 
    m pthread
)

# 安装规则
install(TARGETS control_client RUNTIME DESTINATION bin)
