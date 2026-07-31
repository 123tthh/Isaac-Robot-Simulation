cmake_minimum_required(VERSION 3.8)
project(arms_controller_common)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif ()

set(CMAKE_BUILD_TYPE Release)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(dependencies
        hardware_interface
        rclcpp
        pinocchio
        urdf
        ament_index_cpp
        tf2_geometry_msgs
        tf2
        geometry_msgs
)

# Find Eigen3 (required by Pinocchio)
find_package(Eigen3 REQUIRED)

find_package(ament_cmake REQUIRED)
foreach (Dependency IN ITEMS ${dependencies})
    find_package(${Dependency} REQUIRED)
endforeach ()

get_filename_component(PINOCCHIO_PREFIX "${pinocchio_DIR}/../../../.." ABSOLUTE)
set(PINOCCHIO_DEPRECATED_INCLUDE_DIR "${PINOCCHIO_PREFIX}/include/pinocchio/deprecated")
if (EXISTS "${PINOCCHIO_DEPRECATED_INCLUDE_DIR}/pinocchio/multibody/model.hpp")
    list(APPEND PINOCCHIO_COMPAT_INCLUDE_DIRS "${PINOCCHIO_DEPRECATED_INCLUDE_DIR}")
    set(PINOCCHIO_COMPAT_FORCE_INCLUDE
        "-include${PINOCCHIO_DEPRECATED_INCLUDE_DIR}/pinocchio/container/boost-container-limits.hpp")
endif ()

add_library(${PROJECT_NAME} SHARED
        src/FSM/StateHome.cpp
        src/FSM/StateHold.cpp
        src/FSM/StateMoveJ.cpp
        src/utils/GravityCompensation.cpp
        src/utils/JointLimitsManager.cpp
)

target_include_directories(${PROJECT_NAME}
        PUBLIC
        "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
        "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
        ${PINOCCHIO_COMPAT_INCLUDE_DIRS}
        PRIVATE
        src)

target_compile_definitions(${PROJECT_NAME}
        PUBLIC
        BOOST_MPL_CFG_NO_PREPROCESSED_HEADERS
        BOOST_MPL_LIMIT_LIST_SIZE=30
        BOOST_FUSION_INVOKE_MAX_ARITY=10
        BOOST_FUSION_INVOKE_FUNCTION_OBJECT_MAX_ARITY=10
        BOOST_FUSION_INVOKE_PROCEDURE_MAX_ARITY=10)
target_compile_options(${PROJECT_NAME} PUBLIC ${PINOCCHIO_COMPAT_FORCE_INCLUDE})

ament_target_dependencies(
        ${PROJECT_NAME} PUBLIC
        ${dependencies}
)
target_link_libraries(${PROJECT_NAME} PUBLIC pinocchio::pinocchio)

# Link Eigen3
target_include_directories(${PROJECT_NAME} PUBLIC ${EIGEN3_INCLUDE_DIR})

install(
        DIRECTORY include/
        DESTINATION include/${PROJECT_NAME}
)

install(
        TARGETS ${PROJECT_NAME}
        EXPORT export_${PROJECT_NAME}
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
)

ament_export_dependencies(${dependencies})
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)

ament_package()
