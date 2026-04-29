cmake_minimum_required(VERSION 3.10)
project(reachy_mini LANGUAGES C CXX)

# 设置 C/C++ 标准
set(CMAKE_C_STANDARD 99)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 启用全局 PIC 解决开发板重定位逻辑 (必须在定义 target 前设置)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# ============================================================================
# 路径定义
# ============================================================================
set(STAGING_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../../output/staging")

# ============================================================================
# 依赖组件导入 — 通过 CMake config 导入（由构建系统 package.xml 保证先编译）
# ============================================================================
find_package(motor REQUIRED HINTS ${STAGING_DIR}/lib/cmake/motor)
find_package(Components QUIET HINTS ${STAGING_DIR}/lib/cmake/Components)

# 自动识别组件源码路径
set(COMPONENTS_SRC_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../../../components")

# 辅助函数：确保组件已构建并导入
# usage: ensure_component_built(name src_rel_path lib_names...)
function(ensure_component_built _name _rel_path)
    set(_found_lib "")
    set(_lib_names ${ARGN})
    
    # 搜索现有库文件
    foreach(_ln ${_lib_names})
        if(EXISTS "${STAGING_DIR}/lib/${_ln}")
            set(_found_lib "${STAGING_DIR}/lib/${_ln}")
            break()
        endif()
    endforeach()

    if(NOT TARGET components::${_name})
        add_library(components::${_name} UNKNOWN IMPORTED)
        set_target_properties(components::${_name} PROPERTIES
            INTERFACE_INCLUDE_DIRECTORIES "${STAGING_DIR}/include"
        )

        if(_found_lib)
            message(STATUS "[Internal] Found ${_name} at ${_found_lib}")
            set_target_properties(components::${_name} PROPERTIES IMPORTED_LOCATION "${_found_lib}")
        else()
            # 库缺失，创建自动构建目标
            set(_src_dir "${COMPONENTS_SRC_ROOT}/${_rel_path}")
            set(_build_dir "${CMAKE_BINARY_DIR}/external/${_name}")
            set(_primary_lib "${STAGING_DIR}/lib/${ARGV2}") # 默认取列表第一个作为主库名
            
            message(WARNING "[Internal] ${_name} missing, creating auto-build target for ${_src_dir}")
            
            add_custom_command(
                OUTPUT "${_primary_lib}"
                COMMAND ${CMAKE_COMMAND} -S "${_src_dir}" -B "${_build_dir}" -DCMAKE_INSTALL_PREFIX="${STAGING_DIR}"
                COMMAND ${CMAKE_COMMAND} --build "${_build_dir}" --target install -j4
                COMMENT "Auto-building missing component: ${_name}"
            )
            add_custom_target(${_name}_auto_build DEPENDS "${_primary_lib}")
            
            set_target_properties(components::${_name} PROPERTIES IMPORTED_LOCATION "${_primary_lib}")
            add_dependencies(components::${_name} ${_name}_auto_build)
        endif()
    endif()
endfunction()

# 始终确保组件目标存在（无论 ComponentsConfig.cmake 是否被找到）
# vision
ensure_component_built(vision model_zoo/vision libvision.so)
# audio (支持 libspacemit_audio.a 或 libspace_audio.a)
ensure_component_built(space_audio multimedia/audio libspacemit_audio.a libspace_audio.a)
ensure_component_built(audio_resampler multimedia/audio libaudio_resampler.a)
# asr
ensure_component_built(stt model_zoo/asr libstt.a)
ensure_component_built(sensevoice model_zoo/asr libsensevoice.a)
ensure_component_built(zipformer model_zoo/asr libzipformer.a)
ensure_component_built(kaldi-native-fbank-core model_zoo/asr libkaldi-native-fbank-core.so libkaldi-native-fbank-core.a)
# tts
ensure_component_built(tts model_zoo/tts libtts.a)
ensure_component_built(kaldifst_core model_zoo/tts libkaldifst_core.a)
ensure_component_built(kaldifst_fst model_zoo/tts libkaldifst_fst.a)
# vad
ensure_component_built(vad model_zoo/vad libvad.a)
ensure_component_built(silero_vad model_zoo/vad libsilero_vad.a)
# llm
ensure_component_built(llm_service_cpp model_zoo/llm libllm_service_cpp.so)
# mcp
ensure_component_built(mcp agent_tools/mcp libmcp.so)

# ============================================================================
# 依赖查找 — OpenCV
# ============================================================================
set(OpenCV_DIR "/opt/opencv-spacemit/lib/cmake/opencv4" CACHE PATH "OpenCV cmake dir")
find_package(OpenCV REQUIRED COMPONENTS core imgproc imgcodecs highgui videoio)

# ============================================================================
# 公共包含目录 & 库目录
# ============================================================================
include_directories(
    ${STAGING_DIR}/include
    ${OpenCV_INCLUDE_DIRS}
    ${CMAKE_CURRENT_SOURCE_DIR}/src/kinematics/include
    ${CMAKE_CURRENT_SOURCE_DIR}/src/motor_ctl
    ${CMAKE_CURRENT_SOURCE_DIR}/src/tracker_generic
    ${CMAKE_CURRENT_SOURCE_DIR}/src/vision
    ${CMAKE_CURRENT_SOURCE_DIR}/src/talent_show
)

link_directories(
    ${STAGING_DIR}/lib
    ${STAGING_DIR}/lib/x86_64-linux-gnu
    ${STAGING_DIR}/lib/aarch64-linux-gnu
    ${STAGING_DIR}/lib/riscv64-linux-gnu
    ${STAGING_DIR}/lib/riscv32-linux-gnu
)

# RPATH 设置：安装后可执行文件能找到 ../lib 下的共享库
set(CMAKE_BUILD_RPATH_USE_ORIGIN ON)
set(CMAKE_INSTALL_RPATH "\$ORIGIN/../lib")

# ============================================================================
# 公共 motor 链接库列表（通过 find_package 导入，传递依赖自动继承）
# ============================================================================
set(MOTOR_LIBS
    -Wl,--whole-archive
    motor::motor
    -Wl,--no-whole-archive
)

# Rust 伴生库（如果 motor 构建时启用了 drv_uart_rm）
if(MOTOR_RUST_LIB AND NOT "${MOTOR_RUST_LIB}" STREQUAL "")
    list(APPEND MOTOR_LIBS ${MOTOR_RUST_LIB})
endif()

# ============================================================================
# 公共源文件集合
# ============================================================================
# 运动学核心
set(KINEMATICS_SOURCES
    src/kinematics/src/motion_api.c
    src/kinematics/src/reachy_kinematics.c
)

# 电机控制层（C++ 异步控制器 + C 坐标-运动转换）
set(MOTOR_CTL_SOURCES
    src/motor_ctl/motor_controller.cpp
    src/motor_ctl/coordinate_to_motion.c
)

# 通用跟踪辅助
set(TRACKER_GENERIC_SOURCES
    src/tracker_generic/tracker_app_helper.c
    src/tracker_generic/tracker_utils.c
)

# 视觉 C 封装层
set(VISION_WRAPPER_SOURCES
    src/vision/vision_wrapper.cpp
)

# ============================================================================
# 可执行文件 1: test_api — 运动学 API 测试（仅依赖 motor）
# ============================================================================
add_executable(test_api
    src/kinematics/test/test_api.c
    ${KINEMATICS_SOURCES}
)
set_target_properties(test_api PROPERTIES LINKER_LANGUAGE C)
target_link_libraries(test_api ${MOTOR_LIBS})

# ============================================================================
# 可执行文件 2: test_motion — 运动学 + 电机运动测试（仅依赖 motor）
# ============================================================================
add_executable(test_motion
    src/kinematics/test/test_motion.c
    src/kinematics/src/reachy_kinematics.c
)
set_target_properties(test_motion PROPERTIES LINKER_LANGUAGE C)
target_link_libraries(test_motion ${MOTOR_LIBS})

# ============================================================================
# 可执行文件 3: face_tracker — 人脸跟随（依赖 motor + vision）
# ============================================================================
add_executable(face_tracker
    src/face_tracker.c
    ${KINEMATICS_SOURCES}
    ${MOTOR_CTL_SOURCES}
    ${TRACKER_GENERIC_SOURCES}
    ${VISION_WRAPPER_SOURCES}
)
target_compile_definitions(face_tracker PRIVATE CONFIG_DIR="${CMAKE_CURRENT_SOURCE_DIR}/config")
target_link_libraries(face_tracker
    ${MOTOR_LIBS}
    components::vision
    ${OpenCV_LIBS}
)

# ============================================================================
# 可执行文件 4: gesture_tracker — 手势跟随（依赖 motor + vision）
# ============================================================================
add_executable(gesture_tracker
    src/gesture_tracker.c
    ${KINEMATICS_SOURCES}
    ${MOTOR_CTL_SOURCES}
    ${TRACKER_GENERIC_SOURCES}
    ${VISION_WRAPPER_SOURCES}
)
target_compile_definitions(gesture_tracker PRIVATE CONFIG_DIR="${CMAKE_CURRENT_SOURCE_DIR}/config")
target_link_libraries(gesture_tracker
    ${MOTOR_LIBS}
    components::vision
    ${OpenCV_LIBS}
)

# ============================================================================
# 可执行文件 4.5: test_dance — 舞蹈动作与音频测试
# ============================================================================
option(BUILD_TEST_DANCE "Build the test_dance target" ON)

if(BUILD_TEST_DANCE)
    find_package(PkgConfig REQUIRED)
    pkg_check_modules(SNDFILE sndfile)
    pkg_check_modules(SAMPLERATE samplerate)
    pkg_check_modules(ALSA alsa)

    if(SNDFILE_FOUND AND SAMPLERATE_FOUND AND ALSA_FOUND)
        add_executable(test_dance
            src/talent_show/test_dance.c
            src/talent_show/dance_interface.c
            src/talent_show/dance_player.c
            src/talent_show/audio/audio_player.c
            ${MOTOR_CTL_SOURCES}
            ${KINEMATICS_SOURCES}
            src/tracker_generic/tracker_utils.c
        )
        target_compile_definitions(test_dance PRIVATE DANCE_AUDIO_DIR="${CMAKE_CURRENT_SOURCE_DIR}/src/talent_show/audio")
        target_link_libraries(test_dance
            ${MOTOR_LIBS}
            ${SNDFILE_LIBRARIES}
            ${SAMPLERATE_LIBRARIES}
            ${ALSA_LIBRARIES}
        )
        install(TARGETS test_dance RUNTIME DESTINATION bin)
    else()
        message(WARNING "跳过 test_dance：缺少 sndfile, samplerate 或 alsa 库")
        set(BUILD_TEST_DANCE OFF CACHE BOOL "Build the test_dance target" FORCE)
    endif()
endif()

# ============================================================================
# 安装规则 — 所有基础可执行文件安装到 output/staging/bin
# ============================================================================
install(TARGETS test_api test_motion face_tracker gesture_tracker
    RUNTIME DESTINATION bin
)

# ============================================================================
# 可执行文件 5: reachy_voice_bot — 独立语音对话控制机器人 (结合电机与 AI 引擎)
# ============================================================================
option(BUILD_VOICE_BOT "Build the reachy_voice_bot target" ON)
option(USE_MCP "Enable MCP tools" ON)

if(BUILD_VOICE_BOT)
    if(USE_MCP)
        add_definitions(-DUSE_MCP)
    endif()
    message(STATUS "[VoiceBot] Checking system audio libraries...")
    find_package(PkgConfig REQUIRED)
    pkg_check_modules(SNDFILE REQUIRED sndfile)
    pkg_check_modules(SAMPLERATE REQUIRED samplerate)
    pkg_check_modules(FFTW3 REQUIRED fftw3f)
    pkg_check_modules(ALSA REQUIRED alsa)

    message(STATUS "[VoiceBot] SNDFILE: ${SNDFILE_FOUND}, SAMPLERATE: ${SAMPLERATE_FOUND}, FFTW3: ${FFTW3_FOUND}, ALSA: ${ALSA_FOUND}")

    set(OMNI_AGENT_SRC_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../omni_agent")

    # onnxruntime and curl are system libs (not auto-built)
    find_library(ONNXRUNTIME_LIB NAMES onnxruntime)
    find_library(SPACEMIT_EP_LIB NAMES spacemit_ep)
    find_package(CURL REQUIRED)

    # portaudio — runtime dependency of libspace_audio
    pkg_check_modules(PORTAUDIO REQUIRED portaudio-2.0)

    message(STATUS "[VoiceBot] onnx: ${ONNXRUNTIME_LIB}, curl: ${CURL_FOUND}")

    add_executable(reachy_voice_bot
        src/voice_bot.cpp
        src/voice/voice_ctl.c
        src/voice/voice_common.cpp
        src/voice/mcp_helper.cpp
        src/voice/engine_init.cpp
        src/voice/voice_pipeline.cpp
        src/voice/text_buffer.cpp
        src/voice/src/mcp_action_provider.cpp
        ${MOTOR_CTL_SOURCES}
        ${KINEMATICS_SOURCES}
        src/tracker_generic/tracker_utils.c
        src/talent_show/dance_interface.c
        src/talent_show/dance_player.c
        src/talent_show/audio/audio_player.c
    )

    target_compile_definitions(reachy_voice_bot PRIVATE 
        DANCE_AUDIO_DIR="${CMAKE_CURRENT_SOURCE_DIR}/src/talent_show/audio"
        DEFAULT_MCP_CONFIG_PATH="${CMAKE_CURRENT_SOURCE_DIR}/src/voice/configs/mcp_robot.json"
    )

    target_include_directories(reachy_voice_bot PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/src/voice
        ${CMAKE_CURRENT_SOURCE_DIR}/src/voice/include
        ${CMAKE_CURRENT_SOURCE_DIR}/src/talent_show/audio
        ${SAMPLERATE_INCLUDE_DIRS}
        ${STAGING_DIR}/include/tts
    )

    # 通过 find_package(Components) 导入的组件库 + 系统库
    target_link_libraries(reachy_voice_bot
        ${MOTOR_LIBS}
        # audio 组件
        components::space_audio
        components::audio_resampler
        ${SAMPLERATE_LIBRARIES}
        # asr 组件
        components::stt
        components::sensevoice
        # tts 组件
        components::tts
        components::kaldifst_core
        components::kaldifst_fst
        # vad 组件
        components::vad
        components::silero_vad
        # llm 组件
        components::llm_service_cpp
        # mcp 组件
        components::mcp
        # asr 传递依赖
        components::zipformer
        components::kaldi-native-fbank-core
        -lkissfft-float
        # vision 组件
        components::vision
        ${OpenCV_LIBS}
        # 系统库
        ${ONNXRUNTIME_LIB}
        ${SPACEMIT_EP_LIB}
        ${CURL_LIBRARIES}
        ${SNDFILE_LIBRARIES}
        ${FFTW3_LIBRARIES}
        ${ALSA_LIBRARIES}
        ${PORTAUDIO_LIBRARIES}
    )

    install(TARGETS reachy_voice_bot RUNTIME DESTINATION bin)
    install(FILES src/voice/configs/mcp_robot.json DESTINATION etc/reachy)
endif()


# ============================================================================
# 安装 OpenCV 运行时共享库到 output/staging/lib
# ============================================================================
# vision_wrapper.cpp 和 libvision.so 运行时依赖 OpenCV 共享库，
# 需要将其安装到 staging/lib 以便部署到目标板。
foreach(_opencv_lib ${OpenCV_LIBS})
    # 获取导入目标的实际 .so 路径
    if(TARGET ${_opencv_lib})
        get_target_property(_lib_path ${_opencv_lib} IMPORTED_LOCATION)
        if(NOT _lib_path)
            get_target_property(_lib_path ${_opencv_lib} IMPORTED_LOCATION_RELEASE)
        endif()
        if(NOT _lib_path)
            get_target_property(_lib_path ${_opencv_lib} IMPORTED_LOCATION_NOCONFIG)
        endif()
        if(_lib_path)
            # 安装 .so 本体及其所有符号链接（如 libopencv_core.so.4.14 → libopencv_core.so.414）
            get_filename_component(_lib_dir "${_lib_path}" DIRECTORY)
            get_filename_component(_lib_name "${_lib_path}" NAME)
            # 提取不带版本号的库名前缀，用于 glob 匹配同名符号链接
            string(REGEX MATCH "^lib[a-z_]+" _lib_prefix "${_lib_name}")
            file(GLOB _lib_files "${_lib_dir}/${_lib_prefix}*")
            install(FILES ${_lib_files} DESTINATION lib)
        endif()
    endif()
endforeach()

# ============================================================================
# 仿真控制客户端 (Simulation Client) — 针对开发板优化
# ============================================================================

find_package(PkgConfig REQUIRED)

# 极简查找逻辑：优先使用 PkgConfig (在 Linux 开发板上速度最快)
pkg_check_modules(PROTOBUF REQUIRED protobuf)
pkg_check_modules(GRPC REQUIRED grpc++ grpc)

# 查找插件
find_program(GRPC_CPP_PLUGIN_EXE NAMES grpc_cpp_plugin)
if(NOT GRPC_CPP_PLUGIN_EXE)
    message(FATAL_ERROR "找不到 grpc_cpp_plugin 插件")
endif()

# ── Protobuf & gRPC 代码生成 ─────────────────────────────
set(PROTO_SRC_DIR "${CMAKE_CURRENT_SOURCE_DIR}/simulation")
set(PROTO_FILE    "${PROTO_SRC_DIR}/reachy_sim.proto")
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
         --proto_path=${PROTO_SRC_DIR}
         ${PROTO_FILE}
    DEPENDS ${PROTO_FILE}
    COMMENT "Generating gRPC and Protobuf code from ${PROTO_FILE}"
)

# 仿真目标包含路径
include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}/simulation
    ${PROTO_GEN_DIR}
    ${PROTOBUF_INCLUDE_DIRS}
    ${GRPC_INCLUDE_DIRS}
)

# 聚合搜索路径
link_directories(${PROTOBUF_LIBRARY_DIRS} ${GRPC_LIBRARY_DIRS})

# 3. control_client (解耦控制客户端，含 IK 解算)
add_executable(control_client simulation/control_client.cc src/kinematics/src/reachy_kinematics.c ${PROTO_SRCS})
target_link_libraries(control_client 
    ${GRPC_LIBRARIES} 
    ${PROTOBUF_LIBRARIES} 
    m pthread
)

# 安装规则
install(TARGETS control_client RUNTIME DESTINATION bin)