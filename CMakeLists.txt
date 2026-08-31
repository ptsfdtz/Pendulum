cmake_minimum_required(VERSION 3.25)

project(PendulumLab VERSION 0.1.0 LANGUAGES CXX)

option(PENDULUMLAB_BUILD_TESTS "Build PendulumLab tests" ON)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

include(FetchContent)
FetchContent_Declare(
    nlohmann_json
    GIT_REPOSITORY https://github.com/nlohmann/json.git
    GIT_TAG v3.11.3
    GIT_SHALLOW TRUE
)
FetchContent_MakeAvailable(nlohmann_json)

set(NIDAQMX_ROOT
    "C:/Program Files (x86)/National Instruments/Shared/ExternalCompilerSupport/C"
    CACHE PATH "NI-DAQmx ExternalCompilerSupport C directory"
)
set(ADVANTECH_DAQNAVI_ROOT
    "C:/Advantech/DAQNavi"
    CACHE PATH "Advantech DAQNavi installation directory"
)

find_path(NIDAQMX_INCLUDE_DIR NIDAQmx.h PATHS "${NIDAQMX_ROOT}/include" NO_DEFAULT_PATH REQUIRED)
find_library(NIDAQMX_LIBRARY NAMES NIDAQmx PATHS "${NIDAQMX_ROOT}/lib64/msvc" NO_DEFAULT_PATH REQUIRED)
find_path(BDAQ_INCLUDE_DIR bdaqctrl.h PATHS "${ADVANTECH_DAQNAVI_ROOT}/Inc" NO_DEFAULT_PATH REQUIRED)

add_library(pendulum_core
    src/calibration/HomeCenterController.cpp
    src/calibration/MotorEncoderCalibration.cpp
    src/calibration/MotorZeroCalibrator.cpp
    src/config/Config.cpp
    src/control/AnglePdController.cpp
    src/control/LqrController.cpp
    src/control/PendulumStateEstimator.cpp
    src/hardware/NI6602.cpp
    src/hardware/PCI1723.cpp
    src/logging/AsyncLogger.cpp
    src/safety/ProcessSafety.cpp
    src/safety/SafetyManager.cpp
)
target_include_directories(pendulum_core
    PUBLIC include
    SYSTEM PRIVATE "${NIDAQMX_INCLUDE_DIR}" "${BDAQ_INCLUDE_DIR}"
)
target_link_libraries(pendulum_core PUBLIC nlohmann_json::nlohmann_json PRIVATE "${NIDAQMX_LIBRARY}")
target_compile_definitions(pendulum_core PRIVATE NOMINMAX WIN32_LEAN_AND_MEAN)
target_compile_options(pendulum_core PRIVATE /W4 /permissive- /EHsc)
set_source_files_properties(src/hardware/PCI1723.cpp PROPERTIES COMPILE_OPTIONS "/wd4201")

add_executable(pendulum_self_test src/main.cpp)
target_link_libraries(pendulum_self_test PRIVATE pendulum_core)
target_compile_options(pendulum_self_test PRIVATE /W4 /permissive- /EHsc)

add_executable(pendulum_encoder_calibrate src/tools/EncoderCalibrationMain.cpp)
target_link_libraries(pendulum_encoder_calibrate PRIVATE pendulum_core)
target_compile_options(pendulum_encoder_calibrate PRIVATE /W4 /permissive- /EHsc)

add_executable(pendulum_manual_console src/tools/ManualConsoleMain.cpp)
target_link_libraries(pendulum_manual_console PRIVATE pendulum_core)
target_compile_definitions(pendulum_manual_console PRIVATE NOMINMAX WIN32_LEAN_AND_MEAN)
target_compile_options(pendulum_manual_console PRIVATE /W4 /permissive- /EHsc)

if(PENDULUMLAB_BUILD_TESTS)
    enable_testing()
    add_executable(pendulum_tests tests/Phase1Tests.cpp)
    target_link_libraries(pendulum_tests PRIVATE pendulum_core)
    target_compile_options(pendulum_tests PRIVATE /W4 /permissive- /EHsc)
    add_test(NAME phase1_tests COMMAND pendulum_tests "${CMAKE_SOURCE_DIR}/config/config.json")
endif()
