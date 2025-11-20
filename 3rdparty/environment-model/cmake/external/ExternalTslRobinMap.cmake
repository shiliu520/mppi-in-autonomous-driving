# Fetch and provide tsl::robin_map header-only hash map if not already available

if(NOT TARGET tsl::robin_map)
    include(FetchContent)
    message(STATUS "Fetching robin-map (tsl::robin_map) dependency")
    FetchContent_Declare(tsl_robin_map
        GIT_REPOSITORY https://github.com/Tessil/robin-map.git
        GIT_TAG v1.0.1
    )
    FetchContent_GetProperties(tsl_robin_map)
    if(NOT tsl_robin_map_POPULATED)
        FetchContent_Populate(tsl_robin_map)
        # Header-only interface target
        add_library(tsl_robin_map INTERFACE)
        target_include_directories(tsl_robin_map INTERFACE ${tsl_robin_map_SOURCE_DIR}/include)
        add_library(tsl::robin_map ALIAS tsl_robin_map)
    endif()
endif()
