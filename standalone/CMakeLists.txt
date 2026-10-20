cmake_minimum_required(VERSION 3.22)
project(hydroscan_kalibr VERSION 0.2.2 LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)
get_filename_component(KALIBR_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/.." ABSOLUTE)
find_package(Eigen3 REQUIRED)
# Ubuntu 22.04 ships Python 3.10 and Ubuntu 24.04 ships Python 3.12.
find_package(Python3 3.10...<3.13 REQUIRED COMPONENTS Interpreter Development NumPy)
find_package(Boost REQUIRED COMPONENTS filesystem serialization system thread regex
  python${Python3_VERSION_MAJOR}${Python3_VERSION_MINOR})
find_package(OpenCV REQUIRED COMPONENTS core imgproc imgcodecs calib3d features2d)
if(OpenCV_VERSION VERSION_LESS 4.6)
  # OpenCV 4.5 (Ubuntu 22.04) ArUco finds far fewer 2-bit-border AprilGrid tags than 4.6, which
  # degrades stereo calibration; build the unmodified 4.6.0 module privately instead.
  file(GLOB KALIBR_ARUCO_SOURCES "${CMAKE_CURRENT_SOURCE_DIR}/opencv_aruco_4.6.0/src/*.cpp")
  add_library(kalibr_aruco STATIC ${KALIBR_ARUCO_SOURCES})
  target_include_directories(kalibr_aruco PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/opencv_aruco_4.6.0/include")
  target_link_libraries(kalibr_aruco PUBLIC ${OpenCV_LIBS})
  install(FILES "${CMAKE_CURRENT_SOURCE_DIR}/opencv_aruco_4.6.0/LICENSE"
    DESTINATION /usr/share/doc/hydroscan-kalibr/licenses RENAME opencv_contrib-4.6.0-aruco-LICENSE)
  set(KALIBR_ARUCO kalibr_aruco)
  # The private headers must win over any system opencv2/aruco.hpp.
  include_directories(BEFORE "${CMAKE_CURRENT_SOURCE_DIR}/opencv_aruco_4.6.0/include")
else()
  find_package(OpenCV REQUIRED COMPONENTS core imgproc imgcodecs calib3d features2d aruco)
  set(KALIBR_ARUCO "")
endif()
find_package(yaml-cpp REQUIRED)
find_package(CLI11 QUIET)
if(NOT TARGET CLI11::CLI11)
  # Ubuntu 22.04 packages CLI11 as headers without a CMake package configuration.
  find_path(CLI11_INCLUDE_DIR CLI/CLI.hpp REQUIRED)
  add_library(CLI11::CLI11 INTERFACE IMPORTED)
  target_include_directories(CLI11::CLI11 INTERFACE "${CLI11_INCLUDE_DIR}")
endif()
include(FetchContent)
FetchContent_Declare(common_robotics_utilities
  GIT_REPOSITORY https://github.com/ToyotaResearchInstitute/common_robotics_utilities
  GIT_TAG 1d8fc9db7c15884c503532d853997cbb05fff25b)
FetchContent_Populate(common_robotics_utilities)
include_directories("${common_robotics_utilities_SOURCE_DIR}/include")
install(FILES "${common_robotics_utilities_SOURCE_DIR}/LICENSE"
  DESTINATION /usr/share/doc/hydroscan-kalibr/licenses RENAME common_robotics_utilities-LICENSE)
find_path(SUITESPARSE_INCLUDE cholmod.h PATH_SUFFIXES suitesparse REQUIRED)
foreach(name cholmod spqr amd colamd camd ccolamd suitesparseconfig)
  find_library(SUITESPARSE_${name} NAMES ${name} REQUIRED)
  list(APPEND SUITESPARSE_LIBRARIES "${SUITESPARSE_${name}}")
endforeach()
file(GLOB KALIBR_INCLUDE_DIRS "${KALIBR_ROOT}/*/*/include")
list(FILTER KALIBR_INCLUDE_DIRS EXCLUDE REGEX "kalibr2_ros")
# The legacy CV error-term tree contains a different header with the same name.
list(PREPEND KALIBR_INCLUDE_DIRS "${KALIBR_ROOT}/aslam_nonparametric_estimation/aslam_splines/include")
include_directories(${KALIBR_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS}
  ${Python3_INCLUDE_DIRS} ${Python3_NumPy_INCLUDE_DIRS} ${SUITESPARSE_INCLUDE}
  "${KALIBR_ROOT}/Schweizer-Messer/numpy_eigen/include/numpy_eigen")
add_compile_definitions(KALIBR_HEADLESS __STRICT_ANSI__)

# Install extensions privately; their only native dependency is the shared math core.
function(kalibr_python target package upstream)
  add_library(${target} SHARED ${ARGN})
  target_link_libraries(${target} PRIVATE kalibr_math Python3::Python ${Boost_LIBRARIES})
  set_target_properties(${target} PROPERTIES PREFIX "lib" SUFFIX ".so"
    INSTALL_RPATH "$ORIGIN/../../lib")
  install(TARGETS ${target} LIBRARY DESTINATION "python/${package}")
  install(DIRECTORY "${KALIBR_ROOT}/${upstream}/python/${package}/"
    DESTINATION "python/${package}" FILES_MATCHING PATTERN "*.py")
endfunction()

include(sources.cmake)
kalibr_python(incremental_calibration_python incremental_calibration
  aslam_incremental_calibration/incremental_calibration_python
  "${KALIBR_ROOT}/aslam_incremental_calibration/incremental_calibration_python/src/module.cpp"
  "${KALIBR_ROOT}/aslam_incremental_calibration/incremental_calibration_python/src/OptimizationProblem.cpp"
  "${KALIBR_ROOT}/aslam_incremental_calibration/incremental_calibration_python/src/IncrementalEstimator.cpp"
  "${KALIBR_ROOT}/aslam_incremental_calibration/incremental_calibration_python/src/LinearSolver.cpp")
add_library(kalibr_math SHARED ${KALIBR_MATH_SOURCES}
  "${KALIBR_ROOT}/Schweizer-Messer/sm_boost/src/portable_binary_iarchive.cpp"
  "${KALIBR_ROOT}/Schweizer-Messer/sm_boost/src/portable_binary_oarchive.cpp")
target_link_libraries(kalibr_math PUBLIC Eigen3::Eigen ${Boost_LIBRARIES}
  ${KALIBR_ARUCO} ${OpenCV_LIBS} ${SUITESPARSE_LIBRARIES} pthread)
set_target_properties(kalibr_math PROPERTIES INSTALL_RPATH "$ORIGIN")
install(TARGETS kalibr_math LIBRARY DESTINATION lib)

file(GLOB NUMPY_SOURCES "${KALIBR_ROOT}/Schweizer-Messer/numpy_eigen/src/autogen_module/*.cpp")
add_library(numpy_eigen SHARED ${NUMPY_SOURCES})
target_compile_options(numpy_eigen PRIVATE -fpermissive)
target_link_libraries(numpy_eigen PRIVATE Python3::Python ${Boost_LIBRARIES} Eigen3::Eigen)
install(TARGETS numpy_eigen LIBRARY DESTINATION python/numpy_eigen)
install(FILES "${KALIBR_ROOT}/Schweizer-Messer/numpy_eigen/src/numpy_eigen/__init__.py"
  DESTINATION python/numpy_eigen)

add_library(_kalibr_image MODULE ImageBindings.cpp)
target_link_libraries(_kalibr_image PRIVATE kalibr_math Python3::Python ${OpenCV_LIBS})
set_target_properties(_kalibr_image PROPERTIES PREFIX "" INSTALL_RPATH "$ORIGIN/../lib")
install(TARGETS _kalibr_image LIBRARY DESTINATION python)

include_directories("${CMAKE_CURRENT_SOURCE_DIR}")
add_executable(kalibr-calibrate-cameras "${KALIBR_ROOT}/aslam_offline_calibration/kalibr2_ros/src/CalibrateCameras.cpp" Dataset.cpp)
target_link_libraries(kalibr-calibrate-cameras PRIVATE kalibr_math yaml-cpp CLI11::CLI11)
set_target_properties(kalibr-calibrate-cameras PROPERTIES INSTALL_RPATH "$ORIGIN/../lib")
install(TARGETS kalibr-calibrate-cameras RUNTIME DESTINATION bin)
install(PROGRAMS calibrate-imu DESTINATION bin RENAME kalibr-calibrate-imu)
install(DIRECTORY "${KALIBR_ROOT}/aslam_offline_calibration/kalibr_imu_ros2/kalibr_common"
  "${KALIBR_ROOT}/aslam_offline_calibration/kalibr_imu_ros2/kalibr_imu_ros2"
  DESTINATION python FILES_MATCHING PATTERN "*.py")

# Check the native YAML exporter against the SM matrix, including nonzero
# rotation: identity-only fixtures cannot detect quaternion convention errors.
enable_testing()
add_executable(kalibr-transform-export-test TransformExportTest.cpp Dataset.cpp)
target_link_libraries(kalibr-transform-export-test PRIVATE kalibr_math yaml-cpp)
add_test(NAME transform-export COMMAND kalibr-transform-export-test)
add_executable(kalibr-time-step-test TimeStepTest.cpp)
target_link_libraries(kalibr-time-step-test PRIVATE kalibr_math)
add_test(NAME time-step-recovery COMMAND kalibr-time-step-test)
