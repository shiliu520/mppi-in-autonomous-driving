set(COMMONROAD_PROTOS
        common/util.proto
        common/state.proto
        common/traffic_light_state.proto
        common/traffic_sign_element.proto
        common/scenario_meta_information.proto
        scenario/commonroad_scenario.proto
        scenario/scenario_tags.proto
        scenario/planning_problem.proto
        map/commonroad_map.proto
        map/area.proto
        map/intersection.proto
        map/lanelet.proto
        map/location.proto
        map/traffic_light.proto
        map/traffic_sign.proto
        map/environment_obstacle.proto
        dynamic/commonroad_dynamic.proto
        dynamic/dynamic_obstacle.proto
        dynamic/environment.proto
        dynamic/obstacle.proto
        dynamic/phantom_obstacle.proto
        dynamic/static_obstacle.proto
        dynamic/traffic_light_cycle.proto
        dynamic/traffic_sign_value.proto)

# CMP0116 (3.20) - Ninja generators transform DEPFILEs from add_custom_command().
if(POLICY CMP0116)
        cmake_policy(SET CMP0116 NEW)
endif()

add_library(commonroad_protobuf)

if(CMAKE_VERSION VERSION_GREATER_EQUAL 3.23.0)
    # Set up header file set
    target_sources(commonroad_protobuf
            INTERFACE
            FILE_SET commonroad_protobuf_headers
            TYPE HEADERS
            BASE_DIRS ${PROJECT_BINARY_DIR}/include
    )
endif()

set(gendir "${CMAKE_CURRENT_BINARY_DIR}/generated")
set(depdir "${CMAKE_CURRENT_BINARY_DIR}/dep")

file(MAKE_DIRECTORY ${gendir})
file(MAKE_DIRECTORY ${depdir})

foreach(proto ${COMMONROAD_PROTOS})
        set(proto_orig "${proto}")
        set(proto "${CMAKE_CURRENT_SOURCE_DIR}/${proto}")
        cmake_path(GET proto PARENT_PATH proto_dir)
        cmake_path(GET proto STEM proto_stem)

        set(proto_cpp_file "${proto_stem}.pb.cc")
        set(proto_hdr_file "${proto_stem}.pb.h")
        set(proto_dep_file "${proto_stem}.d")

        set(proto_cpp_file_path "${gendir}/${proto_cpp_file}")
        set(proto_hdr_file_path "${gendir}/${proto_hdr_file}")
        set(proto_dep_file_path "${depdir}/${proto_dep_file}")

        add_custom_command(
                OUTPUT ${proto_cpp_file_path} ${proto_hdr_file_path}
                COMMAND protobuf::protoc
                ARGS
                        --cpp_out=${gendir}
                        --proto_path=${proto_dir}
                        --proto_path=${proto_dir}/../common
                        --dependency_out=${proto_dep_file_path}
                        ${proto}
                DEPENDS
                        ${proto}
                DEPFILE
                        ${proto_dep_file_path}
                VERBATIM
                WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
                COMMENT "Running CXX protocol buffer compiler on ${proto_orig}"
                )

        target_sources(commonroad_protobuf PRIVATE ${proto_cpp_file_path} ${proto_hdr_file_path})

        set_property(SOURCE ${proto_cpp_file} PROPERTY SKIP_UNITY_BUILD_INCLUSION ON)

        if(CMAKE_VERSION VERSION_GREATER_EQUAL 3.23.0)
                target_sources(commonroad_protobuf
                INTERFACE
                FILE_SET commonroad_protobuf_headers
                TYPE HEADERS
                FILES ${proto_hdr_file_path}
                )
        endif()
        target_precompile_headers(commonroad_protobuf
                PRIVATE
                ${proto_hdr_file_path}
                )
endforeach()


target_include_directories(commonroad_protobuf
        SYSTEM INTERFACE
        $<BUILD_INTERFACE:${PROJECT_BINARY_DIR}/include>
        )

target_link_libraries(commonroad_protobuf
        PUBLIC
        $<BUILD_INTERFACE:protobuf::libprotobuf>
        )

set_property(TARGET commonroad_protobuf PROPERTY POSITION_INDEPENDENT_CODE ON)
