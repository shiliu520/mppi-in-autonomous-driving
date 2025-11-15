# This file contains procedures to automatically generate a list of predicates
# which, in turn, is used to automatically generate certain definitions that
# are required for every predicate.

# Split snake_case_name into parts (e.g. list with [snake, case, name])
function(get_parts str out_parts)
    set(parts "")
    while(${str} MATCHES "^([a-zA-Z]+)\_")
        # CMAKE_MATCH_1 now contains capturing group 1 (name part without underscore)

        list(APPEND parts ${CMAKE_MATCH_1})

        # We need to skip over the match itself and the underscore
        string(LENGTH ${CMAKE_MATCH_1} match1_length)
        math(EXPR skip_length "${match1_length} + 1" OUTPUT_FORMAT DECIMAL)

        # Set str to remaining string
        string(SUBSTRING ${str} ${skip_length} -1 str)
    endwhile()

    # Add final part
    list(APPEND parts ${str})

    set(${out_parts} ${parts} PARENT_SCOPE)
endfunction()

# Check whether last part in argument parts matches "predicate"
function(check_ends_with_predicate parts out_result)
    list(POP_BACK parts last_part)
    set(result FALSE)
    if(${last_part} MATCHES "predicate")
        set(result TRUE)
    endif()

    set(${out_result} ${result} PARENT_SCOPE)
endfunction()

# Convert list of lower-case name parts to upper camel case name
function(convert_parts_to_upper_camelcase parts out_str)
    set(final "")
    foreach(part ${parts})
        # Split part into head:tail
        string(SUBSTRING ${part} 0 1 head)
        string(SUBSTRING ${part} 1 -1 tail)
        string(TOUPPER ${head} head)
        string(APPEND final ${head} ${tail})
    endforeach()

    set(${out_str} ${final} PARENT_SCOPE)
endfunction()

function(generate_instantiations predicates all_predicates_gen_file out_gen_file)
    set(instantiate_macro_name "INSTANTIATE_PREDICATE")

    set(predicate_instantiations ${predicates})
    list(TRANSFORM predicate_instantiations PREPEND "\t${instantiate_macro_name}(")
    list(TRANSFORM predicate_instantiations APPEND ")")
    list(JOIN predicate_instantiations "\n" predicate_instantiations_str)
    set(predicate_instantiation_src_file ${CMAKE_CURRENT_SOURCE_DIR}/predicate_instantiation.cpp.inc.in)
    set(predicate_instantiation_gen_file ${CMAKE_CURRENT_BINARY_DIR}/predicate_instantiation.cpp)
    configure_file(
        ${predicate_instantiation_src_file}
        ${predicate_instantiation_gen_file}
        @ONLY)

    set(${out_gen_file} ${predicate_instantiation_gen_file} PARENT_SCOPE)
endfunction()

function(generate_all_predicates_header hdr_files out_gen_file)
    set(predicate_map_includes ${hdr_files})

    list(REMOVE_ITEM predicate_map_includes "commonroad_cpp/predicates/predicate_config.h")
    list(REMOVE_ITEM predicate_map_includes "commonroad_cpp/predicates/commonroad_predicate.h")

    list(TRANSFORM predicate_map_includes PREPEND "#include <")
    list(TRANSFORM predicate_map_includes APPEND ">")
    list(JOIN predicate_map_includes "\n" predicate_map_includes_str)

    set(all_predicates_src_file ${CMAKE_CURRENT_SOURCE_DIR}/all_predicates.h.in)
    set(all_predicates_gen_file ${CMAKE_CURRENT_BINARY_DIR}/all_predicates.h)
    configure_file(
        ${all_predicates_src_file}
        ${all_predicates_gen_file}
        @ONLY)

    set(${out_gen_file} ${all_predicates_gen_file} PARENT_SCOPE)
endfunction()

function(generate_map_entries predicate_map_entries all_predicates_gen_file out_gen_file)
    set(map_entry_macro_name "PREDICATE_MAP_ENTRY")

    list(TRANSFORM predicate_map_entries PREPEND "\t${map_entry_macro_name}")
    list(JOIN predicate_map_entries "\n" predicate_map_entries_str)
    set(predicate_map_src_file ${CMAKE_CURRENT_SOURCE_DIR}/predicate_map.cpp.in)
    set(predicate_map_gen_file ${CMAKE_CURRENT_BINARY_DIR}/predicate_map.cpp)
    configure_file(
        ${predicate_map_src_file}
        ${predicate_map_gen_file}
        @ONLY)

    set(${out_gen_file} ${predicate_map_gen_file} PARENT_SCOPE)
endfunction()

set(predicates "")
set(predicate_map_entries "")
foreach(predicate_file ${ENV_MODEL_PREDICATES_SRC_FILES})
    # message(STATUS "fn file=${predicate_file}")
    cmake_path(GET predicate_file STEM predicate_snake_case)

    if(${predicate_snake_case} MATCHES "commonroad_predicate")
        continue()
    endif()

    get_parts(${predicate_snake_case} fn_parts)
    check_ends_with_predicate("${fn_parts}" ends_with_predicate)
    message(DEBUG "predicate file: parts=${fn_parts} ends_with_predicate=${ends_with_predicate}")
    if(NOT ${ends_with_predicate})
        continue()
    endif()

    convert_parts_to_upper_camelcase("${fn_parts}" predicate_camelcase)
    list(APPEND predicates ${predicate_camelcase})

    list(APPEND predicate_map_entries "(${predicate_snake_case}, ${predicate_camelcase})")
endforeach()

generate_all_predicates_header("${ENV_MODEL_PREDICATES_HDR_FILES}" all_predicates_gen_file)

if (ENV_MODEL_BUILD_PYTHON_BINDINGS)
    if(SKBUILD)
        generate_instantiations("${predicates}" ${all_predicates_gen_file} predicate_instantiation_gen_file)
        target_sources(_crcpp PRIVATE ${predicate_instantiation_gen_file})
    endif()
endif()

generate_map_entries("${predicate_map_entries}" ${all_predicates_gen_file} predicate_map_gen_file)
target_sources(env_model_predicates PRIVATE ${predicate_map_gen_file})

foreach(predicate_cpp_file ${ENV_MODEL_PREDICATES_SRC_FILES})
    string(REGEX REPLACE "\.cpp$" ".h" predicate_hdr_file ${predicate_cpp_file})
    if(NOT EXISTS ${PROJECT_SOURCE_DIR}/include/${predicate_hdr_file})
        message(WARNING
            "Expected to find header file\n\t${predicate_hdr_file}\n"
            "but it does not exist!")
    endif()
    if(NOT ${predicate_hdr_file} IN_LIST ENV_MODEL_PREDICATES_HDR_FILES)
        message(WARNING
            "Expected to find header file\n\t${predicate_hdr_file}\n"
            "in ENV_MODEL_PREDICATES_HDR_FILES")
    endif()
endforeach()
