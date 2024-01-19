^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package rosidl_generator_dds_idl
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.8.1 (2022-03-30)
------------------
* Add changelog (`#56 <https://github.com/ros2/rosidl_dds/issues/56>`_)
* Contributors: Ivan Santiago Paunovic

0.8.0 (2021-03-19)
------------------
* Expose .idl to DDS .idl conversion via rosidl translate CLI. (`#55 <https://github.com/ros2/rosidl_dds/issues/55>`_)
* Update maintainers. (`#54 <https://github.com/ros2/rosidl_dds/issues/54>`_)
* Contributors: Michel Hidalgo, Shane Loretz

0.7.1 (2019-05-08)
------------------
* Fix customization hooks. (`#51 <https://github.com/ros2/rosidl_dds/issues/51>`_)
* Simplify code using updated definition API. (`#50 <https://github.com/ros2/rosidl_dds/issues/50>`_)
* Update code to match refactoring of rosidl definitions. (`#49 <https://github.com/ros2/rosidl_dds/issues/49>`_)
* Contributors: Dirk Thomas, Michael Carroll

0.7.0 (2019-04-12)
------------------
* Change generator to IDL-based pipeline. (`#47 <https://github.com/ros2/rosidl_dds/issues/47>`_)
* Contributors: Jacob Perron

0.6.0 (2018-11-15)
------------------
* Allow generated IDL files. (`#45 <https://github.com/ros2/rosidl_dds/issues/45>`_)
* Support service generation on action folders. (`#44 <https://github.com/ros2/rosidl_dds/issues/44>`_)
* Contributors: Alexis Pojomovsky, Michel Hidalgo, Shane Loretz

0.5.0 (2018-06-23)
------------------
* Use CMAKE_CURRENT_BINARY_DIR for arguments json. (`#42 <https://github.com/ros2/rosidl_dds/issues/42>`_)
* Contributors: Dirk Thomas

0.4.0 (2017-12-08)
------------------
* Update style to satisfy new flake8 plugins. (`#41 <https://github.com/ros2/rosidl_dds/issues/41>`_)
* Fix spelling in docblock.
* Comply with flake8 + flake8-import-order linters. (`#40 <https://github.com/ros2/rosidl_dds/issues/40>`_)
* Remove unnecessary include. (`#39 <https://github.com/ros2/rosidl_dds/issues/39>`_)
* Support sequences of upper bounded strings. (`#38 <https://github.com/ros2/rosidl_dds/issues/38>`_)
* Update schema url in manfiest file.
* Add schema to manifest files.
* Require CMake 3.5. (`#35 <https://github.com/ros2/rosidl_dds/issues/35>`_)
* Change .template into .em (`#34 <https://github.com/ros2/rosidl_dds/issues/34>`_)
* Use CTest BUILD_TESTING. (`#33 <https://github.com/ros2/rosidl_dds/issues/33>`_)
* Update indentation of template logic. (`#32 <https://github.com/ros2/rosidl_dds/issues/32>`_)
* Change constant member to allow reuse of other functions using the type. (`#30 <https://github.com/ros2/rosidl_dds/issues/30>`_)
* Change ifder to match filename.
* Add explicit build type.
* Fix SKIP_INSTALL to affect generated files. (`#26 <https://github.com/ros2/rosidl_dds/issues/26>`_)
* Refactor generators, fix timestamp of generated files. (`#24 <https://github.com/ros2/rosidl_dds/issues/24>`_)
* Fix style.
* Fix assert.
* Fix generated idl for static arrays of non-primitive types. (`#22 <https://github.com/ros2/rosidl_dds/issues/22>`_)
* Refactor message generation. (`#20 <https://github.com/ros2/rosidl_dds/issues/20>`_)
* Replace hard coded pragma with extension point. (`#18 <https://github.com/ros2/rosidl_dds/issues/18>`_)
* Only rewrite templated files when the content has changed. (`#14 <https://github.com/ros2/rosidl_dds/issues/14>`_)
* Disable debug output.
* Add missing copyright / license information.
* Code style only.
* Use two uint64 as writer guid. (`#10 <https://github.com/ros2/rosidl_dds/issues/10>`_)
* Added support for Request and Response IDL files. (`#7 <https://github.com/ros2/rosidl_dds/issues/7>`_)
* Fix warning from rti code generator. (`#6 <https://github.com/ros2/rosidl_dds/issues/6>`_)
* White space only.
* Use ament_lint_auto. (`#2 <https://github.com/ros2/rosidl_dds/issues/2>`_)
* Add support for services. (`#1 <https://github.com/ros2/rosidl_dds/issues/1>`_)
* Use project(.. NONE).
* Consolidate dependencies.
* Expand PYTHON_INSTALL_DIR at configure time.
* Use normalize_path().
* Allow customizing idl generator.
* Add constant support.
* Support upper bounds on arrays and string, support default values for primitive types.
* Pass upper bound to idl as well as introspection type support.
* Refactor namespaces / includes for cross implementation communication.
* Update mapping of primitive types.
* Support multiple type supports.
* Add packages which have been moved from other repos.
* Contributors: Dirk Thomas, Esteve Fernandez, dhood
