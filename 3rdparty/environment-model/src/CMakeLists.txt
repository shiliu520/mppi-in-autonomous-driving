set(ENV_MODEL_SRC_FILES
        commonroad_cpp/auxiliaryDefs/regulatory_elements.cpp
        commonroad_cpp/roadNetwork/road_network.cpp
        commonroad_cpp/roadNetwork/lanelet/lanelet.cpp
        commonroad_cpp/roadNetwork/lanelet/bound.cpp
        commonroad_cpp/roadNetwork/lanelet/lane.cpp
        commonroad_cpp/roadNetwork/lanelet/lane_operations.cpp
        commonroad_cpp/roadNetwork/lanelet/lanelet_operations.cpp
        commonroad_cpp/roadNetwork/regulatoryElements/stop_line.cpp
        commonroad_cpp/roadNetwork/regulatoryElements/traffic_light.cpp
        commonroad_cpp/roadNetwork/regulatoryElements/traffic_sign.cpp
        commonroad_cpp/roadNetwork/regulatoryElements/traffic_sign_element.cpp
        commonroad_cpp/roadNetwork/regulatoryElements/regulatory_elements_utils.cpp
        commonroad_cpp/roadNetwork/intersection/incoming_group.cpp
        commonroad_cpp/roadNetwork/intersection/intersection.cpp
        commonroad_cpp/roadNetwork/intersection/intersection_operations.cpp
        commonroad_cpp/roadNetwork/intersection/outgoing_group.cpp
        commonroad_cpp/roadNetwork/intersection/crossing_group.cpp
        commonroad_cpp/roadNetwork/road_network_config.cpp
        commonroad_cpp/obstacle/initial_state.cpp
        commonroad_cpp/obstacle/obstacle.cpp
        commonroad_cpp/obstacle/obstacle_operations.cpp
        commonroad_cpp/obstacle/obstacle_reference.cpp
        commonroad_cpp/obstacle/actuator_parameters.cpp
        commonroad_cpp/obstacle/sensor_parameters.cpp
        commonroad_cpp/obstacle/time_parameters.cpp
        commonroad_cpp/obstacle/signal_state.cpp
        commonroad_cpp/obstacle/state.cpp
        commonroad_cpp/obstacle/occupancy.cpp
        commonroad_cpp/obstacle/state_meta_info.cpp
        commonroad_cpp/geometry/circle.cpp
        commonroad_cpp/geometry/geometric_operations.cpp
        commonroad_cpp/geometry/rectangle.cpp
        commonroad_cpp/geometry/shape_group.cpp
        commonroad_cpp/geometry/polygon.cpp
        commonroad_cpp/interfaces/commonroad/commonroad_factory_2018b.cpp
        commonroad_cpp/interfaces/commonroad/commonroad_factory_2020a.cpp
        commonroad_cpp/interfaces/commonroad/xml_reader.cpp
        commonroad_cpp/interfaces/commonroad/input_utils.cpp
        commonroad_cpp/goal_state.cpp
        commonroad_cpp/world.cpp
        commonroad_cpp/planning_problem.cpp
        commonroad_cpp/auxiliaryDefs/timer.cpp
        commonroad_cpp/roadNetwork/lanelet/lanelet_graph.cpp
        commonroad_cpp/roadNetwork/environment/environment.cpp
        commonroad_cpp/roadNetwork/environment/area.cpp
        commonroad_cpp/roadNetwork/environment/area_border.cpp
        commonroad_cpp/interfaces/commonroad/protobuf_reader.cpp)

set(ENV_MODEL_HDR_FILES
        commonroad_cpp/auxiliaryDefs/interval.h
        commonroad_cpp/auxiliaryDefs/regulatory_elements.h
        commonroad_cpp/auxiliaryDefs/structs.h
        commonroad_cpp/auxiliaryDefs/timer.h
        commonroad_cpp/auxiliaryDefs/types_and_definitions.h
        commonroad_cpp/geometry/circle.h
        commonroad_cpp/geometry/geometric_operations.h
        commonroad_cpp/geometry/rectangle.h
        commonroad_cpp/geometry/shape.h
        commonroad_cpp/geometry/shape_group.h
        commonroad_cpp/geometry/polygon.h
        commonroad_cpp/geometry/types.h
        commonroad_cpp/interfaces/commonroad/input_utils.h
        commonroad_cpp/interfaces/commonroad/xml_reader.h
        commonroad_cpp/interfaces/commonroad/protobuf_reader.h
        commonroad_cpp/obstacle/actuator_parameters.h
        commonroad_cpp/obstacle/initial_state.h
        commonroad_cpp/obstacle/obstacle.h
        commonroad_cpp/obstacle/obstacle_reference.h
        commonroad_cpp/obstacle/obstacle_operations.h
        commonroad_cpp/obstacle/sensor_parameters.h
        commonroad_cpp/obstacle/time_parameters.h
        commonroad_cpp/obstacle/state.h
        commonroad_cpp/obstacle/obstacle_cache.h
        commonroad_cpp/obstacle/occupancy.h
        commonroad_cpp/obstacle/signal_state.h
        commonroad_cpp/obstacle/state_meta_info.h
        commonroad_cpp/roadNetwork/intersection/incoming_group.h
        commonroad_cpp/roadNetwork/intersection/intersection.h
        commonroad_cpp/roadNetwork/intersection/intersection_operations.h
        commonroad_cpp/roadNetwork/intersection/outgoing_group.h
        commonroad_cpp/roadNetwork/intersection/crossing_group.h
        commonroad_cpp/roadNetwork/lanelet/dijkstra.h
        commonroad_cpp/roadNetwork/lanelet/graph.h
        commonroad_cpp/roadNetwork/lanelet/lane.h
        commonroad_cpp/roadNetwork/lanelet/lanelet.h
        commonroad_cpp/roadNetwork/lanelet/bound.h
        commonroad_cpp/roadNetwork/lanelet/lanelet_graph.h
        commonroad_cpp/roadNetwork/lanelet/lanelet_operations.h
        commonroad_cpp/roadNetwork/lanelet/lane_operations.h
        commonroad_cpp/roadNetwork/regulatoryElements/regulatory_elements_utils.h
        commonroad_cpp/roadNetwork/regulatoryElements/stop_line.h
        commonroad_cpp/roadNetwork/regulatoryElements/traffic_light.h
        commonroad_cpp/roadNetwork/regulatoryElements/traffic_sign.h
        commonroad_cpp/roadNetwork/regulatoryElements/traffic_sign_element.h
        commonroad_cpp/roadNetwork/environment/environment.h
        commonroad_cpp/roadNetwork/environment/area.h
        commonroad_cpp/roadNetwork/environment/area_border.h
        commonroad_cpp/roadNetwork/road_network.h
        commonroad_cpp/roadNetwork/types.h
        commonroad_cpp/roadNetwork/road_network_config.h
        commonroad_cpp/goal_state.h
        commonroad_cpp/planning_problem.h
        commonroad_cpp/scenario.h
        commonroad_cpp/world.h
)

set(ENV_MODEL_PREDICATES_SRC_FILES
        commonroad_cpp/predicates/commonroad_predicate.cpp
        commonroad_cpp/predicates/predicate_parameter.cpp
        commonroad_cpp/predicates/predicate_parameter_collection.cpp

        commonroad_cpp/lanePredicates/commonroad_lane.cpp
        commonroad_cpp/lanePredicates/is_same_lane_pred.cpp

        commonroad_cpp/predicates/braking/keeps_safe_distance_prec_predicate.cpp
        commonroad_cpp/predicates/braking/brakes_stronger_predicate.cpp
        commonroad_cpp/predicates/braking/causes_braking_intersection_predicate.cpp
        commonroad_cpp/predicates/braking/braking_at_intersection_possible_predicate.cpp
        commonroad_cpp/predicates/braking/decelerates_predicate.cpp

        commonroad_cpp/predicates/misc/low_computation_time_test_predicate.cpp
        commonroad_cpp/predicates/misc/high_computation_time_test_predicate.cpp
        commonroad_cpp/predicates/misc/in_projection_domain_predicate.cpp

        commonroad_cpp/predicates/general/orientation_towards_predicate.cpp
        commonroad_cpp/predicates/general/signal_set_predicate.cpp
        commonroad_cpp/predicates/general/makes_u_turn_predicate.cpp
        commonroad_cpp/predicates/general/is_of_type_predicate.cpp

        commonroad_cpp/predicates/intersection/at_intersection_type_predicate.cpp
        commonroad_cpp/predicates/intersection/approach_intersection_predicate.cpp
        commonroad_cpp/predicates/intersection/at_same_intersection_predicate.cpp
        commonroad_cpp/predicates/intersection/in_intersection_conflict_area_predicate.cpp
        commonroad_cpp/predicates/intersection/close_to_intersection_predicate.cpp
        commonroad_cpp/predicates/intersection/unobstructed_intersection_view_predicate.cpp
        commonroad_cpp/predicates/intersection/on_incoming_left_of_predicate.cpp
        commonroad_cpp/predicates/intersection/on_oncoming_of_predicate.cpp

        commonroad_cpp/predicates/lane/overtaking_allowed_predicate.cpp
        commonroad_cpp/predicates/lane/interstate_broad_enough_predicate.cpp
        commonroad_cpp/predicates/lane/narrow_road_predicate.cpp
        commonroad_cpp/predicates/lane/lane_based_orientation_similar_predicate.cpp
        commonroad_cpp/predicates/lane/two_or_more_lanes_for_one_dir_predicate.cpp
        commonroad_cpp/predicates/lane/adjacent_lanelet_of_type_predicate.cpp
        commonroad_cpp/predicates/lane/on_lanelet_with_type_predicate.cpp
        commonroad_cpp/predicates/lane/on_lanelet_with_successor_type_predicate.cpp
        commonroad_cpp/predicates/lane/on_road_predicate.cpp
        commonroad_cpp/predicates/lane/on_similar_oriented_lanelet_with_type_predicate.cpp
        commonroad_cpp/predicates/lane/on_similar_oriented_lanelet_without_type_predicate.cpp
        commonroad_cpp/predicates/lane/in_outermost_lane_predicate.cpp
        commonroad_cpp/predicates/lane/close_to_lane_border_predicate.cpp
        commonroad_cpp/predicates/lane/completely_on_lanelet_type_predicate.cpp
        commonroad_cpp/predicates/lane/in_same_lane_predicate.cpp
        commonroad_cpp/predicates/lane/in_single_lane_predicate.cpp
        commonroad_cpp/predicates/lane/parallel_to_lane_marking_of_type_on_side_predicate.cpp
        commonroad_cpp/predicates/lane/in_neighboring_lane_predicate.cpp
        commonroad_cpp/predicates/lane/neighboring_lane_opp_driving_dir_predicate.cpp
        commonroad_cpp/predicates/lane/on_same_road_predicate.cpp
        commonroad_cpp/predicates/lane/lane_based_orientation_side_predicate.cpp

        commonroad_cpp/predicates/position/in_front_of_predicate.cpp
        commonroad_cpp/predicates/position/left_of_predicate.cpp
        commonroad_cpp/predicates/position/right_of_predicate.cpp
        commonroad_cpp/predicates/position/lateral_distance_to_obstacle_above_threshold_predicate.cpp

        commonroad_cpp/predicates/regulatory/at_traffic_light_predicate.cpp
        commonroad_cpp/predicates/regulatory/at_traffic_sign_predicate.cpp
        commonroad_cpp/predicates/regulatory/stop_line_in_front_predicate.cpp
        commonroad_cpp/predicates/regulatory/close_to_stop_line_predicate.cpp
        commonroad_cpp/predicates/regulatory/relevant_traffic_light_predicate.cpp
        commonroad_cpp/predicates/regulatory/has_priority_predicate.cpp

        commonroad_cpp/predicates/velocity/preserves_traffic_flow_predicate.cpp
        commonroad_cpp/predicates/velocity/slow_other_vehicle_predicate.cpp
        commonroad_cpp/predicates/velocity/keeps_lane_speed_limit_predicate.cpp
        commonroad_cpp/predicates/velocity/keeps_min_speed_limit_predicate.cpp
        commonroad_cpp/predicates/velocity/in_standstill_predicate.cpp
        commonroad_cpp/predicates/velocity/drives_faster_predicate.cpp
        commonroad_cpp/predicates/velocity/reverses_predicate.cpp
        commonroad_cpp/predicates/velocity/keeps_type_speed_limit_predicate.cpp
        commonroad_cpp/predicates/velocity/keeps_fov_speed_limit_predicate.cpp
        commonroad_cpp/predicates/velocity/keeps_braking_speed_limit_predicate.cpp
        commonroad_cpp/predicates/velocity/velocity_below_threshold_predicate.cpp
        commonroad_cpp/predicates/velocity/drives_with_slightly_higher_speed_predicate.cpp)

set(ENV_MODEL_PREDICATES_HDR_FILES
        commonroad_cpp/predicates/predicate_parameter.h
        commonroad_cpp/predicates/predicate_parameter_collection.h
        commonroad_cpp/predicates/commonroad_predicate.h

        commonroad_cpp/lanePredicates/commonroad_lane.h
        commonroad_cpp/lanePredicates/is_same_lane_pred.h

        commonroad_cpp/predicates/braking/keeps_safe_distance_prec_predicate.h
        commonroad_cpp/predicates/braking/brakes_stronger_predicate.h
        commonroad_cpp/predicates/braking/causes_braking_intersection_predicate.h
        commonroad_cpp/predicates/braking/braking_at_intersection_possible_predicate.h
        commonroad_cpp/predicates/braking/decelerates_predicate.h

        commonroad_cpp/predicates/misc/low_computation_time_test_predicate.h
        commonroad_cpp/predicates/misc/high_computation_time_test_predicate.h
        commonroad_cpp/predicates/misc/in_projection_domain_predicate.h

        commonroad_cpp/predicates/general/orientation_towards_predicate.h
        commonroad_cpp/predicates/general/signal_set_predicate.h
        commonroad_cpp/predicates/general/makes_u_turn_predicate.h
        commonroad_cpp/predicates/general/is_of_type_predicate.h

        commonroad_cpp/predicates/intersection/at_intersection_type_predicate.h
        commonroad_cpp/predicates/intersection/approach_intersection_predicate.h
        commonroad_cpp/predicates/intersection/at_same_intersection_predicate.h
        commonroad_cpp/predicates/intersection/in_intersection_conflict_area_predicate.h
        commonroad_cpp/predicates/intersection/close_to_intersection_predicate.h
        commonroad_cpp/predicates/intersection/unobstructed_intersection_view_predicate.h
        commonroad_cpp/predicates/intersection/on_incoming_left_of_predicate.h
        commonroad_cpp/predicates/intersection/on_oncoming_of_predicate.h

        commonroad_cpp/predicates/lane/overtaking_allowed_predicate.h
        commonroad_cpp/predicates/lane/interstate_broad_enough_predicate.h
        commonroad_cpp/predicates/lane/narrow_road_predicate.h
        commonroad_cpp/predicates/lane/lane_based_orientation_similar_predicate.h
        commonroad_cpp/predicates/lane/two_or_more_lanes_for_one_dir_predicate.h
        commonroad_cpp/predicates/lane/adjacent_lanelet_of_type_predicate.h
        commonroad_cpp/predicates/lane/on_lanelet_with_type_predicate.h
        commonroad_cpp/predicates/lane/on_lanelet_with_successor_type_predicate.h
        commonroad_cpp/predicates/lane/on_road_predicate.h
        commonroad_cpp/predicates/lane/on_similar_oriented_lanelet_with_type_predicate.h
        commonroad_cpp/predicates/lane/on_similar_oriented_lanelet_without_type_predicate.h
        commonroad_cpp/predicates/lane/in_outermost_lane_predicate.h
        commonroad_cpp/predicates/lane/close_to_lane_border_predicate.h
        commonroad_cpp/predicates/lane/completely_on_lanelet_type_predicate.h
        commonroad_cpp/predicates/lane/in_same_lane_predicate.h
        commonroad_cpp/predicates/lane/in_single_lane_predicate.h
        commonroad_cpp/predicates/lane/parallel_to_lane_marking_of_type_on_side_predicate.h
        commonroad_cpp/predicates/lane/in_neighboring_lane_predicate.h
        commonroad_cpp/predicates/lane/neighboring_lane_opp_driving_dir_predicate.h
        commonroad_cpp/predicates/lane/on_same_road_predicate.h
        commonroad_cpp/predicates/lane/lane_based_orientation_side_predicate.h

        commonroad_cpp/predicates/position/in_front_of_predicate.h
        commonroad_cpp/predicates/position/left_of_predicate.h
        commonroad_cpp/predicates/position/right_of_predicate.h
        commonroad_cpp/predicates/position/lateral_distance_to_obstacle_above_threshold_predicate.h

        commonroad_cpp/predicates/regulatory/at_traffic_light_predicate.h
        commonroad_cpp/predicates/regulatory/at_traffic_sign_predicate.h
        commonroad_cpp/predicates/regulatory/stop_line_in_front_predicate.h
        commonroad_cpp/predicates/regulatory/close_to_stop_line_predicate.h
        commonroad_cpp/predicates/regulatory/relevant_traffic_light_predicate.h
        commonroad_cpp/predicates/regulatory/has_priority_predicate.h

        commonroad_cpp/predicates/velocity/preserves_traffic_flow_predicate.h
        commonroad_cpp/predicates/velocity/slow_other_vehicle_predicate.h
        commonroad_cpp/predicates/velocity/keeps_lane_speed_limit_predicate.h
        commonroad_cpp/predicates/velocity/keeps_min_speed_limit_predicate.h
        commonroad_cpp/predicates/velocity/in_standstill_predicate.h
        commonroad_cpp/predicates/velocity/drives_faster_predicate.h
        commonroad_cpp/predicates/velocity/reverses_predicate.h
        commonroad_cpp/predicates/velocity/keeps_type_speed_limit_predicate.h
        commonroad_cpp/predicates/velocity/keeps_fov_speed_limit_predicate.h
        commonroad_cpp/predicates/velocity/keeps_braking_speed_limit_predicate.h
        commonroad_cpp/predicates/velocity/velocity_below_threshold_predicate.h
        commonroad_cpp/predicates/velocity/drives_with_slightly_higher_speed_predicate.h
)

add_library(env_model_core ${ENV_MODEL_SRC_FILES})

# TODO: Currently specifying the headers is a little weird as they're in a parent directory.
# It might be easier if src/include are in a common subdirectory,
# e.g. commonroad_cpp/src and commonroad_cpp/include.

set(ENV_MODEL_HDR_FILES_ABS ${ENV_MODEL_HDR_FILES})
set(ENV_MODEL_PREDICATES_HDR_FILES_ABS ${ENV_MODEL_PREDICATES_HDR_FILES})

list(TRANSFORM ENV_MODEL_HDR_FILES_ABS PREPEND ../include/)
list(TRANSFORM ENV_MODEL_PREDICATES_HDR_FILES_ABS PREPEND ../include/)

target_sources(env_model_core PRIVATE ${ENV_MODEL_HDR_FILES_ABS})

if(CMAKE_VERSION VERSION_GREATER_EQUAL 3.23.0)
    target_sources(env_model_core
            INTERFACE
            FILE_SET env_model_core_headers
            TYPE HEADERS
            BASE_DIRS ../include
            FILES ${ENV_MODEL_HDR_FILES_ABS})
endif()

target_compile_features(env_model_core
        PUBLIC cxx_lambdas cxx_auto_type
        PRIVATE cxx_lambdas cxx_auto_type)

set_property(TARGET env_model_core PROPERTY POSITION_INDEPENDENT_CODE ON)

target_include_directories(env_model_core
        PUBLIC
        $<INSTALL_INTERFACE:include>
        $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
        )

target_link_libraries(env_model_core
        PRIVATE
        pugixml::pugixml
        spdlog::spdlog
)

target_link_libraries(env_model_core
        PUBLIC
        Eigen3::Eigen
        Boost::geometry
        range-v3::range-v3
        commonroad_protobuf
        tsl::robin_map
        CommonRoadCLCS::crccosy
        )

target_precompile_headers(env_model_core
        PUBLIC
        $<BUILD_INTERFACE:<Eigen/Dense$<ANGLE-R>>
        $<BUILD_INTERFACE:<boost/geometry/geometry.hpp$<ANGLE-R>>
        $<BUILD_INTERFACE:<geometry/curvilinear_coordinate_system.h$<ANGLE-R>>
        $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include/commonroad_cpp/roadNetwork/lanelet/lane.h>
        $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include/commonroad_cpp/roadNetwork/lanelet/lanelet_operations.h>
        $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include/commonroad_cpp/roadNetwork/lanelet/lane_operations.h>
        $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include/commonroad_cpp/roadNetwork/road_network.h>
        $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include/commonroad_cpp/obstacle/obstacle.h>
        $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include/commonroad_cpp/obstacle/obstacle_operations.h>
    #    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include/commonroad_cpp/obstacle/obstacle_reference.h>
        $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include/commonroad_cpp/world.h>
        )

add_library(env_model_predicates ${ENV_MODEL_PREDICATES_SRC_FILES})


target_sources(env_model_predicates PRIVATE ${ENV_MODEL_PREDICATES_HDR_FILES_ABS})

if(CMAKE_VERSION VERSION_GREATER_EQUAL 3.23.0)
    target_sources(env_model_predicates
            INTERFACE
            FILE_SET env_model_predicates_headers
            TYPE HEADERS
            BASE_DIRS ../include
            FILES ${ENV_MODEL_PREDICATES_HDR_FILES_ABS})
endif()

target_include_directories(env_model_predicates
        PUBLIC
        $<INSTALL_INTERFACE:include>
        $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
        )

target_link_libraries(env_model_predicates
        PRIVATE
        spdlog::spdlog
        )

target_link_libraries(env_model_predicates
        PUBLIC
        Eigen3::Eigen
        range-v3
        env_model_core
        )

set_property(TARGET env_model_predicates PROPERTY POSITION_INDEPENDENT_CODE ON)

target_precompile_headers(env_model_predicates
        PUBLIC
        $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include/commonroad_cpp/predicates/commonroad_predicate.h>
        $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include/commonroad_cpp/predicates/predicate_parameter.h>
        $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include/commonroad_cpp/predicates/predicate_parameter_collection.h>
        )

# Create environment model library
add_library(env_model INTERFACE)

target_link_libraries(env_model
        INTERFACE
        env_model_core
        env_model_predicates
        )

if(APPLE)
    # Required to find library -lomp on mac
    # MAC_LIBOMP_PATH defined in root CMakeLists.txt
    target_link_directories(env_model INTERFACE "${MAC_LIBOMP_PATH}/lib")
endif()

add_library(EnvironmentModel::env_model ALIAS env_model)


if (ENV_MODEL_BUILD_PYTHON_BINDINGS)
    if (SKBUILD)
        set(python_module_name _crcpp)
        nanobind_add_module(${python_module_name} MODULE
                bindings/python_interface.cpp
                bindings/python_interface_core.cpp
                bindings/python_interface_core.h
                bindings/translate_python_types.cpp
                bindings/translate_python_types.h
                bindings/python_interface_predicates.cpp
                bindings/python_interface_predicates.h
        )

        target_precompile_headers(nanobind-static
                PUBLIC
                <nanobind/nanobind.h>
        )

        nanobind_add_stub(
                _crcpp_stub
                MODULE _crcpp
                OUTPUT _crcpp.pyi
                PYTHON_PATH $<TARGET_FILE_DIR:${python_module_name}>
                DEPENDS ${python_module_name}
                MARKER_FILE py.typed
                VERBOSE
        )

        install(FILES
                ${CMAKE_CURRENT_BINARY_DIR}/_crcpp.pyi
                ${CMAKE_CURRENT_BINARY_DIR}/py.typed
                DESTINATION crcpp
                COMPONENT PythonModules
        )

        target_link_libraries(${python_module_name} PRIVATE env_model env_model_predicates spdlog::spdlog)

        install(TARGETS ${python_module_name}
                LIBRARY DESTINATION crcpp
                COMPONENT PythonModules
        )

        add_custom_target(install-python-modules-${python_module_name}
                ${CMAKE_COMMAND}
                -DCMAKE_INSTALL_COMPONENT=PythonModules
                -P "${PROJECT_BINARY_DIR}/cmake_install.cmake"
                DEPENDS ${python_module_name}
        )
    endif ()
endif ()

include(predicates.cmake)

foreach(target IN ITEMS env_model_core env_model_predicates)
    # _USE_MATH_DEFINES is required for M_PI
    if(CMAKE_CXX_COMPILER_ID MATCHES "MSVC")
        target_compile_definitions(${target} PRIVATE _USE_MATH_DEFINES)
    endif()

    if (${PROJECT_NAME}_IS_TOP_LEVEL)
        # this adds many warnings to the build. They usually help to find some bugs
        # TODO: Check whether each warning flag is actually supported by the compiler before adding it
        if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang|AppleClang")
            target_compile_options(${target} PRIVATE -Wall -Wextra -Wconversion
                -pedantic -Wfatal-errors -Wno-unused-parameter)
        endif()
        if(CMAKE_CXX_COMPILER_ID MATCHES "MSVC")
            target_compile_options(${target} PRIVATE /W3)
        endif()
    endif()
endforeach()
