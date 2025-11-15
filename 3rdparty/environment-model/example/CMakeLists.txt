# Create environment model test node
add_executable(env_model_example_node
    src/command_line_input.cpp
    src/predicate_manager.cpp
    src/main.cpp
    )

target_link_libraries(env_model_example_node
    PRIVATE
        OpenMP::OpenMP_CXX
        env_model
        yaml-cpp::yaml-cpp
    )
