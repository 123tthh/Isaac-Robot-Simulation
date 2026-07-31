cmake_minimum_required(VERSION 3.14)
project(ocs2_pinocchio_interface)

set(CMAKE_BUILD_TYPE Release)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(dependencies
        ocs2_robotic_tools
        pinocchio
        urdfdom
        hpp-fcl
)

find_package(ament_cmake REQUIRED)
find_package(ocs2_robotic_tools REQUIRED)
find_package(urdfdom REQUIRED)
find_package(pinocchio REQUIRED)
find_package(hpp-fcl REQUIRED)

# Add pinocchio configurations
include(cmake/pinocchio_config.cmake)

# ROS Jazzy ships Pinocchio 3 with legacy OCS2 headers under the deprecated
# include tree; expose that tree and raise Boost.MPL's variant limit for the
# full Pinocchio joint collection.
get_filename_component(PINOCCHIO_PREFIX "${pinocchio_DIR}/../../../.." ABSOLUTE)
set(PINOCCHIO_DEPRECATED_INCLUDE_DIR "${PINOCCHIO_PREFIX}/include/pinocchio/deprecated")
if (EXISTS "${PINOCCHIO_DEPRECATED_INCLUDE_DIR}/pinocchio/multibody/model.hpp")
    list(APPEND PINOCCHIO_COMPAT_INCLUDE_DIRS "${PINOCCHIO_DEPRECATED_INCLUDE_DIR}")
    set(PINOCCHIO_COMPAT_FORCE_INCLUDE
        "-include${PINOCCHIO_DEPRECATED_INCLUDE_DIR}/pinocchio/container/boost-container-limits.hpp")
endif ()

###########
## Build ##
###########

# ocs2 pinocchio interface library
add_library(${PROJECT_NAME} SHARED
        src/PinocchioInterface.cpp
        src/PinocchioInterfaceCppAd.cpp
        src/PinocchioEndEffectorKinematics.cpp
        src/PinocchioEndEffectorKinematicsCppAd.cpp
        src/urdf.cpp
)
target_include_directories(${PROJECT_NAME}
        PUBLIC
        ${pinocchio_INCLUDE_DIRS}
        ${PINOCCHIO_COMPAT_INCLUDE_DIRS}
        ${hpp-fcl_INCLUDE_DIRS}
        "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
        "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")

ament_target_dependencies(${PROJECT_NAME} ${dependencies})
target_link_libraries(${PROJECT_NAME} pinocchio::pinocchio)
target_compile_options(${PROJECT_NAME} PUBLIC ${OCS2_PINOCCHIO_FLAGS})
target_compile_options(${PROJECT_NAME} PUBLIC ${PINOCCHIO_COMPAT_FORCE_INCLUDE})
target_compile_definitions(${PROJECT_NAME}
        PUBLIC
        BOOST_MPL_CFG_NO_PREPROCESSED_HEADERS
        BOOST_MPL_LIMIT_LIST_SIZE=30
        BOOST_FUSION_INVOKE_MAX_ARITY=10
        BOOST_FUSION_INVOKE_FUNCTION_OBJECT_MAX_ARITY=10
        BOOST_FUSION_INVOKE_PROCEDURE_MAX_ARITY=10)

#############
## Install ##
#############
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


############
# Testing ##
############
find_package(ament_lint_auto REQUIRED)
ament_lint_auto_find_test_dependencies()

if (BUILD_TESTING)
    find_package(ament_cmake_gtest REQUIRED)

    ament_add_gtest(testPinocchioInterface
            test/testPinocchioInterface.cpp
            test/testPinocchioEndEffectorKinematics.cpp
    )
    target_link_libraries(testPinocchioInterface ${PROJECT_NAME})
    ament_target_dependencies(testPinocchioInterface ${dependencies})
    target_compile_options(testPinocchioInterface PUBLIC ${OCS2_PINOCCHIO_FLAGS})
    target_compile_options(testPinocchioInterface PUBLIC ${PINOCCHIO_COMPAT_FORCE_INCLUDE})
    target_compile_definitions(testPinocchioInterface
            PUBLIC
            BOOST_MPL_CFG_NO_PREPROCESSED_HEADERS
            BOOST_MPL_LIMIT_LIST_SIZE=30
            BOOST_FUSION_INVOKE_MAX_ARITY=10
            BOOST_FUSION_INVOKE_FUNCTION_OBJECT_MAX_ARITY=10
            BOOST_FUSION_INVOKE_PROCEDURE_MAX_ARITY=10)

endif ()

ament_package(CONFIG_EXTRAS "cmake/pinocchio_config.cmake")
