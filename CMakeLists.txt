cmake_minimum_required(VERSION 2.8)
project(MOV_SLAM)

IF(NOT CMAKE_BUILD_TYPE)
  SET(CMAKE_BUILD_TYPE Release)
ENDIF()

MESSAGE("Build type: " ${CMAKE_BUILD_TYPE})

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -Wall   -O3")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall   -O3 -fopenmp -DG2O_USE_VENDORED_CERES")
set(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} -march=native")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -march=native ")

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_WARN_DEPRECATED OFF CACHE BOOL "" FORCE)

add_library(sse4_2 INTERFACE IMPORTED)
if(MSVC)
    target_compile_options(sse4_2 INTERFACE /arch:SSE4.2)
else()
    target_compile_options(sse4_2 INTERFACE -msse4.2)
endif()

add_definitions(-DCOMPILEDWITHC17)

LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)

find_package(OpenCV 4.4)
   if(NOT OpenCV_FOUND)
      message(FATAL_ERROR "OpenCV > 4.4 not found.")
   endif()

MESSAGE("OPENCV VERSION:")
MESSAGE(${OpenCV_VERSION})

find_package(Eigen3 3.1.0 REQUIRED)
find_package(Pangolin REQUIRED)

# ffmpeg
find_path(AVCODEC_INCLUDE_DIR libavcodec/avcodec.h)
find_library(AVCODEC_LIBRARY avcodec)

find_path(AVFORMAT_INCLUDE_DIR libavformat/avformat.h)
find_library(AVFORMAT_LIBRARY avformat)

find_path(AVUTIL_INCLUDE_DIR libavutil/avutil.h)
find_library(AVUTIL_LIBRARY avutil)

find_path(AVDEVICE_INCLUDE_DIR libavdevice/avdevice.h)
find_library(AVDEVICE_LIBRARY avdevice)

find_path(SUITEPARSE_INCLUDE_DIR suiteparse/cholmod.h)

include_directories(
${PROJECT_SOURCE_DIR}
${PROJECT_SOURCE_DIR}/include
${PROJECT_SOURCE_DIR}/include/CameraModels
${EIGEN3_INCLUDE_DIR}
${Pangolin_INCLUDE_DIRS}
/usr/include/suitesparse
)

set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)

add_library(${PROJECT_NAME} SHARED
src/System.cc
src/Tracking.cc
src/LocalMapping.cc
src/FrameDrawer.cc
src/Converter.cc
src/MapPoint.cc
src/KeyFrame.cc
src/Atlas.cc
src/Map.cc
src/MapDrawer.cc
src/Optimizer.cc
src/Frame.cc
src/Viewer.cc
src/ImuTypes.cc
src/G2oTypes.cc
src/CameraModels/Pinhole.cpp
src/OptimizableTypes.cpp
src/GeometricTools.cc
src/TwoViewReconstruction.cc
src/Config.cc
src/Settings.cc
src/VideoDecoder.cc
src/MOVExtractor.cc
include/System.h
include/Tracking.h
include/LocalMapping.h
include/MOVMatcher.h
include/FrameDrawer.h
include/Converter.h
include/MapPoint.h
include/KeyFrame.h
include/Atlas.h
include/Map.h
include/MapDrawer.h
include/Optimizer.h
include/Frame.h
include/Viewer.h
include/ImuTypes.h
include/G2oTypes.h
include/CameraModels/GeometricCamera.h
include/CameraModels/Pinhole.h
include/OptimizableTypes.h
include/GeometricTools.h
include/TwoViewReconstruction.h
include/SerializationUtils.h
include/Config.h
include/Settings.h
include/VideoDecoder.h
include/EXPRESS.h
include/MOVExtractor.h
include/VideoBase.h)

target_link_libraries(${PROJECT_NAME}
libcholmod.so
${OpenCV_LIBS}
${EIGEN3_LIBS}
${Pangolin_LIBRARIES}
-lboost_serialization
-lcrypto
${AVCODEC_LIBRARY} ${AVFORMAT_LIBRARY} ${AVUTIL_LIBRARY} ${AVDEVICE_LIBRARY}
libg2o_cli.so
libg2o_hierarchical.so
libg2o_opengl_helper.so
libg2o_solver_csparse.so
libg2o_solver_slam2d_linear.so
libg2o_types_icp.so
libg2o_types_slam2d.so
libg2o_core.so
libg2o_incremental.so
libg2o_parser.so
libg2o_solver_dense.so
libg2o_solver_structure_only.so
libg2o_types_sba.so
libg2o_types_slam2d_addons.so
libg2o_csparse_extension.so
libg2o_interactive.so
libg2o_simulator.so
libg2o_solver_eigen.so
libg2o_stuff.so
libg2o_types_sclam2d.so
libg2o_types_slam3d.so
libg2o_ext_freeglut_minimal.so
libg2o_interface.so
libg2o_solver_cholmod.so
libg2o_solver_pcg.so
libg2o_types_data.so
libg2o_types_sim3.so
libg2o_types_slam3d_addons.so
)

add_definitions("-D SOPHUS_USE_BASIC_LOGGING")

# Build examples

#Monocular examples

add_executable(mono_video_tartan
        Examples/Monocular/mono_video_tartan.cc)
target_link_libraries(mono_video_tartan ${PROJECT_NAME})

# Stereo Examples

add_executable(stereo_video_tartan
        Examples/Stereo/stereo_video_tartan.cc)
target_link_libraries(stereo_video_tartan ${PROJECT_NAME})
