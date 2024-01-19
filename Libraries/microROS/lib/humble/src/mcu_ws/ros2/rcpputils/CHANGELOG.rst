^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package rcpputils
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

2.4.1 (2023-04-25)
------------------
* Fix possible race condition in create_directories() (`#162 <https://github.com/ros2/rcpputils/issues/162>`_) (`#176 <https://github.com/ros2/rcpputils/issues/176>`_)
* Contributors: mergify[bot]

2.4.0 (2022-03-01)
------------------
* Install includes to include/${PROJECT_NAME} (`#160 <https://github.com/ros2/rcpputils/issues/160>`_)
* Contributors: Shane Loretz

2.3.2 (2022-01-14)
------------------
* Fix include order for cpplint (`#158 <https://github.com/ros2/rcpputils/issues/158>`_)
* [path] Declare the default assignment operator (`#156 <https://github.com/ros2/rcpputils/issues/156>`_)
* Contributors: Abrar Rahman Protyasha, Jacob Perron

2.3.1 (2021-12-17)
------------------
* Fixes for uncrustify 0.72 (`#154 <https://github.com/ros2/rcpputils/issues/154>`_)
* Fix the BSD license headers to use the standard one. (`#153 <https://github.com/ros2/rcpputils/issues/153>`_)
* Update maintainers to Chris Lalancette (`#152 <https://github.com/ros2/rcpputils/issues/152>`_)
* Contributors: Audrow Nash, Chris Lalancette

2.3.0 (2021-11-18)
------------------
* Add checked convert_to_nanoseconds() function (`#145 <https://github.com/ros2/rcpputils/issues/145>`_)
* Add missing sections in docs/FEATURES.md TOC (`#151 <https://github.com/ros2/rcpputils/issues/151>`_)
* [env] Add `set_env_var` function (`#150 <https://github.com/ros2/rcpputils/issues/150>`_)
* Add missing cstddef include (`#147 <https://github.com/ros2/rcpputils/issues/147>`_)
* Add accumulator test to CMakeLists.txt (`#144 <https://github.com/ros2/rcpputils/issues/144>`_)
* `rcpputils::fs`: Fix doxygen parameter identifier (`#142 <https://github.com/ros2/rcpputils/issues/142>`_)
* Make thread safety macro C++ standards compliant (`#141 <https://github.com/ros2/rcpputils/issues/141>`_)
* Fix API documentation for clean `rosdoc2` build (`#139 <https://github.com/ros2/rcpputils/issues/139>`_)
* Improve `rcppmath` Doxygen documentation (`#138 <https://github.com/ros2/rcpputils/issues/138>`_)
* Improve documentation of utilities in docs/FEATURES.md (`#137 <https://github.com/ros2/rcpputils/issues/137>`_)
* Include `rcppmath` utilities in docs/FEATURES.md (`#136 <https://github.com/ros2/rcpputils/issues/136>`_)
* Fix `IllegalStateException` reference in FEATURES (`#135 <https://github.com/ros2/rcpputils/issues/135>`_)
* migrate rolling mean from ros2_controllers to rcppmath (`#133 <https://github.com/ros2/rcpputils/issues/133>`_)
* Update includes after rcutils/get_env.h deprecation (`#132 <https://github.com/ros2/rcpputils/issues/132>`_)
* Contributors: Abrar Rahman Protyasha, Barry Xu, Christophe Bedard, Karsten Knese, Octogonapus

2.2.0 (2021-04-02)
------------------
* Update quality declaration links (`#130 <https://github.com/ros2/rcpputils/issues/130>`_)
* Add functions for getting library path and filename (`#128 <https://github.com/ros2/rcpputils/issues/128>`_)
* Contributors: Nikolai Morin, Simon Honigmann

2.1.0 (2021-03-01)
------------------
* Add path equality operators (`#127 <https://github.com/ros2/rcpputils/issues/127>`_)
* Add create_temp_directory filesystem helper (`#126 <https://github.com/ros2/rcpputils/issues/126>`_)
* Use new noexcept specifier. (`#123 <https://github.com/ros2/rcpputils/issues/123>`_)
* Contributors: Chen Lihui, Emerson Knapp

2.0.4 (2021-01-25)
------------------
* Add stream operator for paths to make it easier to log (`#120 <https://github.com/ros2/rcpputils/issues/120>`_)
* Path join operator is const (`#119 <https://github.com/ros2/rcpputils/issues/119>`_)
* No windows.h in header files (`#118 <https://github.com/ros2/rcpputils/issues/118>`_)
* Fix rcpputils::SharedLibrary tests. (`#117 <https://github.com/ros2/rcpputils/issues/117>`_)
* Contributors: Emerson Knapp, Ivan Santiago Paunovic, Michel Hidalgo

2.0.3 (2020-12-08)
------------------
* Update QD to QL 1 (`#114 <https://github.com/ros2/rcpputils/issues/114>`_)
* Make sure to not try to index into an empty path. (`#113 <https://github.com/ros2/rcpputils/issues/113>`_)
* Contributors: Chris Lalancette, Stephen Brawner

2.0.2 (2020-11-16)
------------------
* Fix working with filesystem parent paths. (`#112 <https://github.com/ros2/rcpputils/issues/112>`_)
* Cleanup mislabeled BSD license (`#37 <https://github.com/ros2/rcpputils/issues/37>`_)
* overload functions for has_symbol and get_symbol with raw string literal (`#110 <https://github.com/ros2/rcpputils/issues/110>`_)
* Add an ASSERT to the pointer traits tests. (`#111 <https://github.com/ros2/rcpputils/issues/111>`_)
* replace custom get env login into rcutils_get_env(). (`#99 <https://github.com/ros2/rcpputils/issues/99>`_)
* Removed Github Actions (`#105 <https://github.com/ros2/rcpputils/issues/105>`_)
* Update the package.xml files with the latest Open Robotics maintainers (`#102 <https://github.com/ros2/rcpputils/issues/102>`_)
* Contributors: Alejandro Hernández Cordero, Chen Lihui, Chris Lalancette, Michael Jeronimo, Tully Foote, tomoya

2.0.1 (2020-10-05)
------------------
* Make sure that an existing path is a directory for create_directories (`#98 <https://github.com/ros2/rcpputils/issues/98>`_)
* Transfer ownership to Open Robotics (`#100 <https://github.com/ros2/rcpputils/issues/100>`_)
* Ensure -fPIC is used when building a static lib (`#93 <https://github.com/ros2/rcpputils/issues/93>`_)
* Contributors: Christophe Bedard, Dirk Thomas, Louise Poubel, William Woodall

2.0.0 (2020-07-21)
------------------
* Removed doxygen warnings (`#86 <https://github.com/ros2/rcpputils/issues/86>`_) (`#87 <https://github.com/ros2/rcpputils/issues/87>`_)
* Add clamp header (`#85 <https://github.com/ros2/rcpputils/issues/85>`_)
* Removed doxygen warnings (`#86 <https://github.com/ros2/rcpputils/issues/86>`_)
* Split get_env_var() into header and implementation (`#83 <https://github.com/ros2/rcpputils/issues/83>`_)
* Add cstring include for strcmp (`#81 <https://github.com/ros2/rcpputils/issues/81>`_)
* filesystem helpers: adding remove_all to remove non-empty directories (`#79 <https://github.com/ros2/rcpputils/issues/79>`_)
* Contributors: Alejandro Hernández Cordero, Christophe Bedard, Hunter L. Allen, Karsten Knese, Victor Lopez

1.2.0 (2020-06-26)
------------------
* Add scope_exit helper (`#78 <https://github.com/ros2/rcpputils/issues/78>`_)
* Bump setup-ros to 0.0.23, action-ros-lint to 0.0.6, action-ros-ci to 0.0.17 (`#77 <https://github.com/ros2/rcpputils/issues/77>`_)
* Contributors: Devin Bonnie, Michel Hidalgo

1.1.0 (2020-06-22)
------------------
* Fix parent_path() for empty paths and paths of length one (`#73 <https://github.com/ros2/rcpputils/issues/73>`_)
* Add get_executable_name() function (`#70 <https://github.com/ros2/rcpputils/issues/70>`_)
* Address memory leak in remove pointer test (`#72 <https://github.com/ros2/rcpputils/issues/72>`_)
* Add current_path to filesystem_helpers (`#63 <https://github.com/ros2/rcpputils/issues/63>`_)
* Align path combine behavior with C++17 (`#68 <https://github.com/ros2/rcpputils/issues/68>`_)
* Update quality declaration to QL 2 (`#71 <https://github.com/ros2/rcpputils/issues/71>`_)
* Contributors: Jacob Perron, Scott K Logan, Stephen Brawner

1.0.1 (2020-06-03)
------------------
* Include stdexcept in get_env.hpp (`#69 <https://github.com/ros2/rcpputils/issues/69>`_)
* Update quality declaration for version stability (`#66 <https://github.com/ros2/rcpputils/issues/66>`_)
* Handle empty paths in is_absolute (`#67 <https://github.com/ros2/rcpputils/issues/67>`_)
* Add Security Vulnerability Policy pointing to REP-2006 (`#65 <https://github.com/ros2/rcpputils/issues/65>`_)
* Contributors: Chris Lalancette, Scott K Logan, Steven! Ragnarök

1.0.0 (2020-05-26)
------------------
* Remove mention of random file from temporary_directory_path doc (`#64 <https://github.com/ros2/rcpputils/issues/64>`_)
* Contributors: Scott K Logan

0.3.1 (2020-05-08)
------------------
* Fix Action CI by using released upload-artifact instead of master (`#61 <https://github.com/ros2/rcpputils/issues/61>`_)
* Quality declaration (`#47 <https://github.com/ros2/rcpputils/issues/47>`_)
* Contributors: Emerson Knapp, brawner

0.3.0 (2020-04-24)
------------------
* Added shared library to feature list (`#58 <https://github.com/ros2/rcpputils/issues/58>`_)
* export targets in a addition to include directories / libraries (`#57 <https://github.com/ros2/rcpputils/issues/57>`_)
* remove pointer for smart pointer (`#56 <https://github.com/ros2/rcpputils/issues/56>`_)
* Added shared library class description to readme (`#53 <https://github.com/ros2/rcpputils/issues/53>`_)
* Increased shared library tests (`#51 <https://github.com/ros2/rcpputils/issues/51>`_)
* Removed duplicated split function (`#54 <https://github.com/ros2/rcpputils/issues/54>`_)
* Exposed get_env_var (`#55 <https://github.com/ros2/rcpputils/issues/55>`_)
* Added debug version for library names (`#52 <https://github.com/ros2/rcpputils/issues/52>`_)
* Added unload_library method to shared_library (`#50 <https://github.com/ros2/rcpputils/issues/50>`_)
* Included abstraction for rcutils::shared_library (`#49 <https://github.com/ros2/rcpputils/issues/49>`_)
* Add more documentation and include doxyfile (`#46 <https://github.com/ros2/rcpputils/issues/46>`_)
* Update README.md with license and build badges. (`#45 <https://github.com/ros2/rcpputils/issues/45>`_)
* Update README to mention assertion helper functions (`#43 <https://github.com/ros2/rcpputils/issues/43>`_)
* Add rcpputils::fs::file_size and rcpputils::fs::is_directory (`#41 <https://github.com/ros2/rcpputils/issues/41>`_)
* Make assert functions accept an optional string. (`#42 <https://github.com/ros2/rcpputils/issues/42>`_)
* Add functions for C++ assertions (`#31 <https://github.com/ros2/rcpputils/issues/31>`_)
* remove reference for pointer traits (`#38 <https://github.com/ros2/rcpputils/issues/38>`_)
* code style only: wrap after open parenthesis if not in one line (`#36 <https://github.com/ros2/rcpputils/issues/36>`_)
* Bug fixes for rcpputils::fs API (`#35 <https://github.com/ros2/rcpputils/issues/35>`_)
  * Ensure rcpputils::fs::create_directories works with absolute paths.
  * Implement temp_directory_path() for testing purposes.
  * Fix rcpputils::fs::path::parent_path() method.
* Add build and test workflow (`#33 <https://github.com/ros2/rcpputils/issues/33>`_)
* Add linting workflow (`#32 <https://github.com/ros2/rcpputils/issues/32>`_)
* Fix filesystem helpers for directory manipulation. (`#30 <https://github.com/ros2/rcpputils/issues/30>`_)
* Contributors: Alejandro Hernández Cordero, Dirk Thomas, Emerson Knapp, Karsten Knese, Michel Hidalgo, Zachary Michaels

0.2.1 (2019-11-12)
------------------
* add new function to remove the extension of a file (`#27 <https://github.com/ros2/rcpputils/pull/27>`_)
* Contributors: Anas Abou Allaban

0.2.0 (2019-09-24)
------------------
* find_library: Centralize functionality here (`#25 <https://github.com/ros2/rcpputils/issues/25>`_)
* Implement join() (`#20 <https://github.com/ros2/rcpputils/issues/20>`_)
* Rename test (`#21 <https://github.com/ros2/rcpputils/issues/21>`_)
* use _WIN32 instead of WIN32 (`#24 <https://github.com/ros2/rcpputils/issues/24>`_)
* Update README.md and package.xml (`#22 <https://github.com/ros2/rcpputils/issues/22>`_)
* Fix typo (`#23 <https://github.com/ros2/rcpputils/issues/23>`_)
* type trait rcpputils::is_pointer<T>` (`#19 <https://github.com/ros2/rcpputils/issues/19>`_)
* File extension addition for camera calibration parser (`#18 <https://github.com/ros2/rcpputils/issues/18>`_)
* Add endian helper until C++20 (`#16 <https://github.com/ros2/rcpputils/issues/16>`_)
* use iterators for split (`#14 <https://github.com/ros2/rcpputils/issues/14>`_)
* Add function 'find_and_replace' (`#13 <https://github.com/ros2/rcpputils/issues/13>`_)
* Contributors: Andreas Klintberg, Dirk Thomas, Jacob Perron, Karsten Knese, Michael Carroll, Michel Hidalgo, Tully Foote

0.1.0 (2019-04-13)
------------------
* Fixed leak in test_basic.cpp. (`#9 <https://github.com/ros2/rcpputils/issues/9>`_)
* Added CODEOWNERS file. (`#10 <https://github.com/ros2/rcpputils/issues/10>`_)
* Added commonly-used filesystem helper to utils. (`#5 <https://github.com/ros2/rcpputils/issues/5>`_)
* Fixed thread_safety_annotation filename to .hpp. (`#6 <https://github.com/ros2/rcpputils/issues/6>`_)
* Added section about DCO to CONTRIBUTING.md.
* Added thread annotation macros. (`#2 <https://github.com/ros2/rcpputils/issues/2>`_)
* Contributors: Dirk Thomas, Emerson Knapp, Michael Carroll, Thomas Moulard
