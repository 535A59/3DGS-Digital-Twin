
cmake_minimum_required(VERSION 3.16)
project(mtc_dynamic)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(ament_cmake REQUIRED)

find_package(generate_parameter_library REQUIRED)
find_package(moveit_common REQUIRED)
moveit_package()
find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_task_constructor_core REQUIRED)
find_package(moveit_task_constructor_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(octomap_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometric_shapes REQUIRED)

set(THIS_PACKAGE_INCLUDE_DEPENDS
    moveit_core
    moveit_ros_planning
    moveit_ros_planning_interface
    moveit_task_constructor_core
    moveit_task_constructor_msgs
    rclcpp
    tf2_eigen
    octomap_msgs
    geometry_msgs
    tf2_ros
    moveit_msgs
    rclcpp_action
    geometric_shapes
)

add_library(${PROJECT_NAME}_pick_place_task SHARED
    src/pick_place_task.cpp
    src/geometry_utils.cpp
)
ament_target_dependencies(${PROJECT_NAME}_pick_place_task ${THIS_PACKAGE_INCLUDE_DEPENDS})
install(TARGETS ${PROJECT_NAME}_pick_place_task
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION lib/${PROJECT_NAME})

include_directories(include)

# declare a demo consisting of a single cpp file
function(demo name)
    add_executable(${PROJECT_NAME}_${name} src/${name}.cpp)
    set(parameter_filename ${name}_parameters)
    # 这个函数会在 src/src/ 目录下寻找参数文件，您的文件结构是正确的
    if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/src/${parameter_filename}.yaml)
        generate_parameter_library(${parameter_filename} src/${parameter_filename}.yaml)
        target_link_libraries(${PROJECT_NAME}_${name} ${parameter_filename})
    endif()
    ament_target_dependencies(${PROJECT_NAME}_${name} ${THIS_PACKAGE_INCLUDE_DEPENDS})
    set_target_properties(${PROJECT_NAME}_${name} PROPERTIES OUTPUT_NAME ${name} PREFIX "")
    install(TARGETS ${PROJECT_NAME}_${name}
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION lib/${PROJECT_NAME}
    )
endfunction()


demo(pick_place_demo)
demo(collectEnvDemo)

# 这两行是您原来就有的，保持不变
target_link_libraries(${PROJECT_NAME}_pick_place_task pick_place_demo_parameters)
target_link_libraries(${PROJECT_NAME}_pick_place_demo ${PROJECT_NAME}_pick_place_task)

# ▼▼▼ 新增内容 2：为 collectEnvDemo 链接到核心库 ▼▼▼
# 因为 collectEnvDemo.cpp 也用到了 pick_place_task 的功能
target_link_libraries(${PROJECT_NAME}_collectEnvDemo ${PROJECT_NAME}_pick_place_task)


# 安装脚本和配置文件，保持不变
install(PROGRAMS
      
        DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch config
        DESTINATION share/${PROJECT_NAME}
)

ament_package()