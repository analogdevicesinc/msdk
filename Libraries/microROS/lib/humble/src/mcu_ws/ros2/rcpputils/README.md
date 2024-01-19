# rcpputils: ROS 2 C++ Utilities
![License](https://img.shields.io/github/license/ros2/rcpputils)
![Test rcpputils](https://github.com/ros2/rcpputils/workflows/Test%20rcpputils/badge.svg)


`rcpputils` is a C++ API consisting of macros, functions, and data structures intended for use throughout the ROS 2 codebase

## Quality Declaration

This package claims to be in the **Quality Level 1** category, see the [Quality Declaration](QUALITY_DECLARATION.md) for more details.

## API
This package currently contains:
* Assertion functions
* Clang thread safety annotation macros
* Library discovery
* String helpers
* File system helpers
* Type traits helpers
* Class that dynamically loads, unloads and get symbols from shared libraries at run-time.

Features are described in more detail at [docs/FEATURES.md](docs/FEATURES.md)
