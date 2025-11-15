# CommonRoad Environment Model

The CommonRoad C++ Environment Model provides classes and methods to represent the CommonRoad format in C++17.
It contains an interface to Python and predicates for evaluating traffic rules.

## Overview

### Dependencies to other repositories

Inclusion of other internal repositoryis is automatically done via Git.
An SSH key in your Gitlab account is required.
See [here](https://docs.gitlab.com/ee/ssh/) for instructions to add an SSH key.

For development, access to the following repositories is necessary:
- [CommonRoad Drivability Checker (Curvilinear Coordinate System)](https://gitlab.lrz.de/cps/commonroad-drivability-checker)

### Detailed Instructions
We extracted the relevant information in separate readmes depending on the way you want to use the environment-model:

- You want to *install and use* the Python side of the Environment Model: [Python Usage](python.md)
- You want to *develop* the Python side of the Environment Model: [Python Development](python_dev.md)
- You want to integrate the Environment Model into another CMake project: [C++ Usage](cpp.md)
- You want to set up a development environment for the Environment Model itself: [C++ Development](cpp_dev.md)
- You want to generate the docu of the environment-model: [Documentation Generation](doc.md)
- You want to set up git hooks for the repository: [GitHooks](git.md)
