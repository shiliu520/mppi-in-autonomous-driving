## Working on the C++ Code itself

### Using nix
For using nix, please visit [Nix Tutorial](./nix.md)

### Compiler and Build System
A recent C/C++ compiler is required in any case (we tested gcc11).
CMake is required in order to build the C++ library.
For the Python module, you don't need to have CMake installed as pip will
automatically provide CMake to the build system.

### Accelerating the Build with System Packages
The build system can automatically download and configure all dependencies without
further intervention [^1].
However, if you install the dependencies on the system with your package manager,
you can speed up the build process since it won't be necessary to build our own
version of them, as long as the version is sufficient for our requirements.
Certain dependencies take a long time to build, so this is generally a good idea.
Try to install system packags at least for the following dependencies if possible:
- Boost
- Protobuf (the library `libprotobuf` and the compiler `protoc`)

These other dependencies do not take a long time to download and build,
but you might still want to install them to save some time:
- Eigen3
- spdlog
- pugixml
- yaml-cpp
- GoogleTest

The following optional tools are required only for certain tasks:
- For building the documentation
    * Doxygen
    * Graphviz
- For code coverage
    * gcovr

[Specific installation instructions](#installing-dependencies-on-common-distributions) are provided for common distributions.

[^1]: For C++, there is one exception - CMake will need to be installed
on the system.

### Build Instructions for the C++ Library

#### Modern Build using the Ninja Multi-Config Generator (recommended)
The Ninja Multi-Config generator is a recent addition to CMake with some improvements over
the classical Makefile generator:
- The Ninja build system itself is generally faster than Make
- Ninja automatically selects the number of parallel jobs based on the number of cores -
  fiddling with `-j` options is no longer necessary
- You can build any configuration from a single build directory.

In order to use the Ninja Multi-Config generator, you need to have Ninja installed
on your system.

```bash
# Configuration step
cmake -G "Ninja Multi-Config" -S . -B build

# Build default targets, default configuration (debug)
cmake --build build
# Build default targets, release configuration
cmake --build build --config Release
# Build and run tests
cmake --build build --target test

# Alternatively, you can invoke Ninja directly:
cd build

# Build default targets, default configuration (debug)
ninja
# Build default targets, release configuration
ninja -f build-Release.ninja
# Build and run tests
ninja test
```

#### Classical Build using the Makefile Generator
You can also perform a build using the Makefile generator.
```bash
cmake -S . -B build
# Or specify the build type:
cmake -S . -B build -DCMAKE_BUILD_TYPE=Debug

# Build default targets
cmake --build build
# Replace 4 with the number of parallel jobs you want (generally, the CPU core count)
cmake --build build --parallel 4
# Build and run tests
cmake --build build --target test

# Alternatively, you can invoke Make directly:
cd build

# Build default targets
make
# Replace 4 with the number of parallel jobs you want (generally, the CPU core count)
make -j4
# Build and run tests
make test
```


## Installing Dependencies on Common Distributions

### Debian/Ubuntu

We recommend Ubuntu 20.04 or newer.

#### Ubuntu 20.04
You need to install the following packages:
`build-essential git pkg-config wget libomp-dev libeigen3-dev libboost-all-dev uuid-dev libspdlog-dev default-jdk`,
where  default-jdk is only needed for the CommonRoad Monitor or Runtime-Verification repository.
All other required packages should be part of the standard Ubuntu installation.


#### Arch Linux

```bash
sudo pacman -S base-devel boost boost-libs pugixml spdlog yaml-cpp protobuf eigen gtest
```

Optional dependencies:
```bash
sudo pacman -S doxygen graphviz gcovr
```
