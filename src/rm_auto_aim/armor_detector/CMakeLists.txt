cmake_minimum_required(VERSION 3.10)
project(armor_detector)

## Use C++14
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

## By adding -Wall and -Werror, the compiler does not ignore warnings anymore,
## enforcing cleaner code.
add_definitions(-Wall -Werror)

## Export compile commands for clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

#######################
## Find dependencies ##
#######################

find_package(ament_cmake_auto REQUIRED)
find_package(OpenCV REQUIRED)
ament_auto_find_build_dependencies()

###########
## Build ##
###########

ament_auto_add_library(${PROJECT_NAME} SHARED
  DIRECTORY src
)

target_include_directories(${PROJECT_NAME} PUBLIC ${OpenCV_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS})

# TensorRT + CUDA（YOLO 检测器）
find_library(NVINFER_LIB nvinfer)
find_library(NVINFER_PLUGIN_LIB nvinfer_plugin)
target_link_libraries(${PROJECT_NAME} ${NVINFER_LIB} ${NVINFER_PLUGIN_LIB})
find_package(CUDA QUIET)
if(CUDA_FOUND)
  target_include_directories(${PROJECT_NAME} PUBLIC ${CUDA_INCLUDE_DIRS})
  target_link_libraries(${PROJECT_NAME} ${CUDA_LIBRARIES})
else()
  # 回退：Jetson 默认 CUDA 路径
  target_include_directories(${PROJECT_NAME} PUBLIC /usr/local/cuda/include)
  find_library(CUDART_LIB cudart PATHS /usr/local/cuda/lib64)
  target_link_libraries(${PROJECT_NAME} ${CUDART_LIB})
endif()
# TensorRT 8.5 的 enqueueV2/destroy 等 API 标记为 deprecated，但仍可用；关闭该告警（配合全局 -Werror）
target_compile_options(${PROJECT_NAME} PRIVATE -Wno-deprecated-declarations)

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN rm_auto_aim::ArmorDetectorNode
  EXECUTABLE armor_detector_node
)

#############
## Testing ##
#############

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  list(APPEND AMENT_LINT_AUTO_EXCLUDE
    ament_cmake_copyright
    ament_cmake_uncrustify
    ament_cmake_cpplint
  )
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest)
  ament_add_gtest(test_node_startup test/test_node_startup.cpp)
  target_link_libraries(test_node_startup ${PROJECT_NAME})

  ament_add_gtest(test_number_cls test/test_number_cls.cpp)
  target_link_libraries(test_number_cls ${PROJECT_NAME})

endif()

#############
## Install ##
#############

ament_auto_package(
  INSTALL_TO_SHARE
  model
)
