cmake_minimum_required(VERSION 3.16)
project(reachy_mini_sim_server C CXX)

set(CMAKE_C_STANDARD 99)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

# ── Find Packages ──────────────────────
find_package(PkgConfig REQUIRED)
pkg_check_modules(PROTOBUF REQUIRED protobuf)
pkg_check_modules(GRPC REQUIRED grpc++ grpc)

find_program(GRPC_CPP_PLUGIN_EXE NAMES grpc_cpp_plugin)
if(NOT GRPC_CPP_PLUGIN_EXE)
    message(FATAL_ERROR "找不到 grpc_cpp_plugin 插件")
endif()

# ── Protobuf & gRPC 代码生成 ─────────────────────────────
set(PROTO_FILE    "${CMAKE_CURRENT_SOURCE_DIR}/../reachy_sim.proto")
set(PROTO_GEN_DIR "${CMAKE_CURRENT_BINARY_DIR}/gen")
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

# ── Targets ──────────────────────────────────────────────
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}/..
    ${PROTO_GEN_DIR}
    ${PROTOBUF_INCLUDE_DIRS}
    ${GRPC_INCLUDE_DIRS}
)
link_directories(${PROTOBUF_LIBRARY_DIRS} ${GRPC_LIBRARY_DIRS})

# 引入 mujoco_sim 库
set(COMPONENTS_SRC_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../../../../../components")
set(STAGING_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../../../../output/staging")

# 辅助函数：确保组件已构建并导入
function(ensure_component_built _name _rel_path)
    set(_found_lib "")
    set(_lib_names ${ARGN})
    
    foreach(_ln ${_lib_names})
        if(EXISTS "${STAGING_DIR}/lib/${_ln}")
            set(_found_lib "${STAGING_DIR}/lib/${_ln}")
            break()
        endif()
    endforeach()

    if(NOT TARGET components::${_name})
        file(MAKE_DIRECTORY "${STAGING_DIR}/include")
        add_library(components::${_name} UNKNOWN IMPORTED)
        set_target_properties(components::${_name} PROPERTIES
            INTERFACE_INCLUDE_DIRECTORIES "${STAGING_DIR}/include"
        )

        if(_found_lib)
            message(STATUS "[Internal] Found ${_name} at ${_found_lib}")
            set_target_properties(components::${_name} PROPERTIES IMPORTED_LOCATION "${_found_lib}")
        else()
            set(_src_dir "${COMPONENTS_SRC_ROOT}/${_rel_path}")
            set(_build_dir "${CMAKE_BINARY_DIR}/external/${_name}")
            set(_primary_lib "${STAGING_DIR}/lib/${ARGV2}")
            
            set(_extra_args "")
            if(DEFINED MUJOCO_DIR)
                set(_extra_args "-DMUJOCO_DIR=${MUJOCO_DIR}")
            endif()
            
            message(WARNING "[Internal] ${_name} missing, creating auto-build target for ${_src_dir}")
            
            add_custom_command(
                OUTPUT "${_primary_lib}"
                COMMAND ${CMAKE_COMMAND} -S "${_src_dir}" -B "${_build_dir}" -DCMAKE_INSTALL_PREFIX="${STAGING_DIR}" "-DCMAKE_C_FLAGS=-w" "-DCMAKE_CXX_FLAGS=-w -Wno-missing-template-keyword" ${_extra_args}
                COMMAND ${CMAKE_COMMAND} --build "${_build_dir}" --target install -j4
                COMMENT "Auto-building missing component: ${_name}"
            )
            add_custom_target(${_name}_auto_build DEPENDS "${_primary_lib}")
            
            set_target_properties(components::${_name} PROPERTIES IMPORTED_LOCATION "${_primary_lib}")
            add_dependencies(components::${_name} ${_name}_auto_build)
        endif()
    endif()
endfunction()

ensure_component_built(mujoco_sim simulation/mujoco libmujoco_sim.so)

# sim_server
add_executable(sim_server sim_server.cc ${PROTO_SRCS})

target_link_libraries(sim_server 
    components::mujoco_sim
    ${GRPC_LIBRARIES} 
    ${PROTOBUF_LIBRARIES} 
    m pthread
)

# ── Installation ─────────────────────────────────────────
install(TARGETS sim_server DESTINATION bin)
