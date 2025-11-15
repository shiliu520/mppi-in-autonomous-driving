## Installing the Environment Model

### Integrating the Environment Model into Another C++ Project
Recent CMake provide the `FetchContent` module which vastly simplifies the integration
for common use scenarios.
Simply insert the following snippet somewhere in your `CMakeLists.txt`:
```cmake
include(FetchContent)

FetchContent_Declare(EnvironmentModel
        GIT_REPOSITORY git@gitlab.lrz.de:cps/commonroad/environment-model.git
        # You can specify any reference here, but prefer specifying a concrete commit if possible
        # as that will speed up the build since Git won't need to check whether branch moved in the meantime
        GIT_TAG <reference to commit, branch, tag...>
)
FetchContent_MakeAvailable(EnvironmentModel)
```

Then add the Environment Model as a dependency to the targets which require it:
```cmake
target_link_libraries(<MyLibraryOrExecutable> PUBLIC EnvironmentModel::env_model)
```

## Using the Environment Model

The main purpose of this code is to serve as library containing the main CommonRoad elements and predicates, e.g., for traffic rules.
The code can be used as C++-standalone version for extracting the predicate cost and satisfaction probability of predicates.
The standalone execution can be configured via the config file and be executed via the **env_model_example_node** which can be executed via
```bash
pathToExecutable/env_model_example_node --input-file pathToRepository/src/commonroad_cpp/default_config.yaml --t 6
```
where *--input-file* specifies the path to a configuration file based on the default configuration file and *--t* specifies the number of threads which should be used.
**Attention**: The environment-model library is not developed for parallelization.
Therefore, we recommend to use copies inside threads.
