# External Dependency Quality declaration libyaml

This document is a declaration of software quality for the `libyaml` external dependency, based on the guidelines in [REP-2004](https://github.com/ros-infrastructure/rep/blob/rep-2004/rep-2004.rst).

The [libyaml](https://github.com/yaml/libyaml) external dependency is a C library for parsing and emitting YAML.
It is maintained in the YAML Project Github organization together with 115 other repositories.
First, a summary discussing how this library is qualified is presented, and then it will be listed how this library matches the standards defined for ROS packages.

## Summary

The `libyaml` meets the basic requirements for a software platform in terms of testing its basic functionality, providing a [valid license](https://github.com/yaml/libyaml/blob/master/License) for the code used and a public Github repository with the changes made to the code over time.

Even though the library does not provide an API/ABI policy targeting the desired use of the library, the imported version of `libyaml` through the `libyaml_vendor` package is fixed to the 0.18 version. This will make the API/ABI stable.

There is no explicit support for any OS platform, however their [Github repository](https://github.com/yaml/libyaml) installation appears to be targeting Linux.
The first version of this library was developed in 2006, and it is used widely.
There is no explicit metric of how much the library is used, but the equivalent library for Python, developed by the same organization is required for at least 150k repositories (According to [Github metrics](https://github.com/yaml/pyyaml/network/dependents?package_id=UGFja2FnZS01MjUyMjEzNQ%3D%3D)).
`libyaml` library is used for some optional fast functionality.
The [safe_yaml](https://rubygems.org/gems/safe_yaml) ruby gem has over 80 million downloads and one of its implementations uses `libyaml` through psych.
It is also used in the [Go-yaml project](https://github.com/go-yaml/yaml), the project supporting YAML in the Go language.

ROS 2 uses this library to load parameters from a file when the arguments are properly used. There are no official tests or code coverage information about this library, but in this case we are only interested in the portion of public API that reads YAML files and we will cover these
tests and coverage in the vendored package. Including specific tests for the `libyaml` version imported by `libyaml_vendor` for the tier 1 platforms listed in [REP-2000](https://www.ros.org/reps/rep-2000.html#support-tiers).

Considering the previously mentioned reasons, we consider this library to be robust and reliable and at Quality Level 1.

# Comparison with ROS packages quality standards

## Version policy [1]

### Version Scheme [1.i]

It is not stated if the library supports any kind of version policy. But the version increases with a new release. The imported version of
`libyaml` through the `libyaml_vendor` package is fixed to the 0.18 version.

### Version Stability [1.ii]

Current version of `libyaml` in [its repository](https://github.com/yaml/libyaml) is 0.24, however, for the purposes of ROS2 Quality Level analysis, the imported version of `libyaml` through the `libyaml_vendor` package is fixed to the 0.18 version.

### Public API Declaration [1.iii]

As a C library, elements available in [yaml.h](https://github.com/yaml/libyaml/blob/master/include/yaml.h) are considered to be the library's public API.

### API Stability Policy [1.iv]

There is no policy for API stability. This is not a problem because the `libyaml_vendor` package importing the `libyaml` dependency is using a fixed version, in this case, the [0.18](https://github.com/yaml/libyaml/tree/release-0.1.8).

### ABI Stability Policy [1.v]

There is no policy for ABI stability. This is not a problem because the `libyaml_vendor` package importing the `libyaml` dependency is using a fixed version, in this case, the [0.18](https://github.com/yaml/libyaml/tree/release-0.1.8).

### ABI and ABI Stability Within a Released ROS Distribution [1.vi]

There is not a direct correlation between the `libyaml` releases and the ROS distributions, however this is not a problem because the `libyaml_vendor` package importing the `libyaml` dependency is using a fixed version, in this case, the [0.18](https://github.com/yaml/libyaml/tree/release-0.1.8)

## Change Control Process [2]

### Change Requests [2.i]

Checking through the commits history, it can be seen is not the case.

### Contributor Origin [2.ii]

Does not have it (or it does not seem like it’s the case).

### Peer Review Policy [2.iii]

Seems to be followed for pull requests on the Github repository, but as not all code changes occur through change requests, this can not be confirmed for these changes.

### Continuous Integration [2.iv]

`libyaml` is compiled in the ROS 2 buildfarm for all [tier 1 platforms](https://www.ros.org/reps/rep-2000.html#support-tiers).

Currently nightly results can be seen here:
* [linux-aarch64_release](https://ci.ros2.org/view/nightly/job/nightly_linux-aarch64_release/lastBuild/testReport/libyaml_vendor/)
* [linux_release](https://ci.ros2.org/view/nightly/job/nightly_linux_release/lastBuild/testReport/libyaml_vendor/)
* [mac_osx_release](https://ci.ros2.org/view/nightly/job/nightly_osx_release/lastBuild/testReport/libyaml_vendor/)
* [windows_release](https://ci.ros2.org/view/nightly/job/nightly_win_rel/lastBuild/testReport/libyaml_vendor/)

### Documentation Policy [2.v]

Not available.

## Documentation [3]

### Feature Documentation [3.i]

Provided [doxygen documentation](https://github.com/yaml/libyaml/tree/master/doc) for the whole project. It is not provided as a document, it has to be built separately when downloading.

### Public API Documentation [3.ii]

Yes, doxygen documentation is available for library [here](https://libyaml.docsforge.com/master/).

### License [3.iii]

MIT license declared for the repository, it can be found [here](https://github.com/yaml/libyaml/blob/master/LICENSE).

### Copyright Statements [3.iv]

Is not available.

### Quality Declaration [3.v]

This document represents the Quality Declaration document for the `libyaml` ROS dependency.

## Testing [4]

### Feature Testing [4.i]

Tests provided to cover the expected usage of the library, for the version of the library used can be found [here](https://github.com/yaml/libyaml/tree/release-0.1.8/tests).

### Public API Testing [4.ii]

Not clear without coverage results to check if all the API is covered.

### Coverage [4.iii]

Code coverage and internal policies are not public, if any.

### Performance [4.iv]

Performance tests are defined in the vendored package.

### Linters and Static Analysis [4.v]

Not available publicly, if any.

## Dependencies [5]

### Direct Runtime ROS Dependencies [5.i]

The `libyaml` library does not add additional dependencies, it only requires C++ standard libraries to be built and used.

### Optional Direct Runtime ROS Dependencies [5.ii]

Does not apply for external dependencies.

### Direct Runtime non-ROS Dependency [5.iii]

This library does not have external dependencies.

## Platform Support [6]

This library does not state support for any specific platform, but it is built in the ROS 2 buildfarm for all tier 1 platforms:

Currently nightly results can be seen here:
* [linux-aarch64_release](https://ci.ros2.org/view/nightly/job/nightly_linux-aarch64_release/lastBuild/testReport/libyaml_vendor/)
* [linux_release](https://ci.ros2.org/view/nightly/job/nightly_linux_release/lastBuild/testReport/libyaml_vendor/)
* [mac_osx_release](https://ci.ros2.org/view/nightly/job/nightly_osx_release/lastBuild/testReport/libyaml_vendor/)
* [windows_release](https://ci.ros2.org/view/nightly/job/nightly_win_rel/lastBuild/testReport/libyaml_vendor/)

## Security [7]

### Vulnerability Disclosure Policy [7.i]

The `libyaml` library does not have a Vulnerability Disclosure Policy. But for ROS 2's purposes, see the policy defined in the Quality Declaration of the `libyaml_vendor` package.
