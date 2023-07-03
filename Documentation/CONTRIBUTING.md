# Analog Devices MSDK Development and Contribution Guidelines

## Development Flow

Development for the MSDK also follows the official [GitHub development flow guidelines](https://docs.github.com/en/get-started/quickstart/github-flow).

For beginners, [learngitbranching.js.org](https://learngitbranching.js.org/) is a great hands-on starting resource.

## Contribution Guidelines

The MSDK follows the [GitHub contribution guidelines](https://docs.github.com/en/get-started/quickstart/contributing-to-projects).  

External contributions from outside the [Analog Devices organization](https://github.com/Analog-Devices-MSDK) should be made via a Pull Request opened from a fork.  Internal contributions should also preferrably use a fork where possible.

If a direct branch on the mainline MSDK repo is made, the following branch naming conventions should be used when possible:

- Bugfix/ticket: `fix/ticketnumber`
- New feature: `feature/branchname`
- Generic development branch: `dev/branchname`
- New and/or modified example branch: `example/branchname`

## PR Title - Subject Responsibilites for Reviewers/Mergers/PR Owners

Format: 
`type(scope)<!>: Subject`

For more information of the format, please review the [pull_request_template.md](https://github.com/Analog-Devices-MSDK/msdk/blob/main/Documentation/pull_request_template.md).

These are the rules for the `Subject`.

1.  The first character is capitalized and the subject must not end with any punctuation (!,.?) 
2.  Use imperative mood in the Subjects - meaning the tone of the Subject should be like giving an order or request.
    * “Add” instead of “Added” or “Adding”.
    * “Fix” instead of “Fixed” or “Fixing”.
    * “Update” instead of “Updated” or “Updating”.
    * “Delete” instead of “Deleted” or “Deleting”.
3.  The Subject contents should have useful and concise wording.
    * No long descriptive subjects - not concise if it gets too descriptive. 
        * You can be more descriptive in the body or footer of the commit by running:
        `git commit -m “Title contents” -m “Body contents” -m “Footer contents”` 
    * No simple titles like “Add small SPI change”
4.  The Subject should include what parts are affected, if any. Only the parts’ CHIP NAME should be used in the Subject. No die names are allowed.
    * For the most readability, append the affected parts list at the end of the Subject, so the important, beginning part of the commit message is visible when traversing through the MSDK repo on GitHub.
    * The list of chip names should be in ascending numerical order in the Subject.
    * Examples:
        1. `fix(CMSIS): Rename TRIMSIR registers for MAX32670 and MAX32675`
        2. `fix(Third-Party): Remove USB DMA support for MAX32655, MAX32665, MAX32690, MAX78000, and MAX78002`
        3. `feat(Examples): Add console output in Hello_World READMEs for all parts`
        4. `fix(Examples,PeriphDrivers): Deprecate MXC\_RTC\_GetSecond and MXC\_RTC\_GetSubSecond for all parts except for MAX32520`
            * Use this type of wording if all but a few parts were affected to shorten the Subject.
    * **TIP**: The auto-labeler workflow adds part labels to a PR depending on what specific files were updated. Use the list of labels to figure out what parts were affected by the PR's changes.
5.  If the PR title is related to a Jira ticket or an issue, the Subject should begin with the ticket designation.
    * Examples:
        1. `fix(CMSIS): MSDK-123: Add missing I2C Target registers for MAX32670, MAX32672, and MAX32675` 
        2. `fix(PeriphDrivers): Ticket-321: Fix SPI hanging on 9-bit wide messages for all parts` 
        3. `fix(Documentation): Issue #123: Revise steps for MSDK installation in the user guides`
6.  When a merger “Squash and Merges” an approved PR to main, do not delete the PR number that is automatically appended to the title. Ideally, the PR title should be following our rules before approval, so the merger just needs to press the “Squash and Merge” button.
    * Using previous examples from rule 4, this is what the commit should look like in the main branch:
        1. `fix(CMSIS): MSDK-123: Add missing I2C Target registers for all parts (#412)`
        2. `fix(PeriphDrivers): Ticket-321: Fix SPI hanging on 9-bit wide messages for all parts (#646)`
        3. `feat(Examples): Add console output in Hello_World READMEs for all parts (#342)`
        4. `fix(Examples,PeriphDrivers): Deprecate MXC\_RTC\_GetSecond and MXC\_RTC\_GetSubSecond for all parts except for MAX32520 (#248)`

Please ensure all actions have passed or successfully completed before merging a PR into `main`. MSDK maintainers have permissions to merge approved PRs even with a failing action.

## PR Format Rules (Workflow Enforced)
1.  The type is case-sensitive and must be one of the listed types.
2.  The scope is case-sensitive and must be one of the listed scopes.
3.  The first word of the Subject must be capitalized.
4.  The Subject must not end with any punctuation (periods, commas, exclamation marks).

## Style Guide

The MSDK code-base, for the most-part, follows the [Linux Kernel coding style](https://www.kernel.org/doc/html/v4.10/process/coding-style.html#linux-kernel-coding-style) _and_ [Google's C++ Style Guide](https://google.github.io/styleguide/cppguide.html) with the following exception(s):

- Indentations are 4 spaces.

Formatting and styling is enforced via [clang-format](https://www.kernel.org/doc/html/latest/process/clang-format.html) **version 14** and [cpplint](https://github.com/cpplint/cpplint), which automatically run checks against all PRs.  A PR cannot be merged until it passes these checks.

### Running the Linter & Formatter

Both utilities can be run locally.  cpplint should be run first, then clang-format.  Additionally, both should be run from the root directory of the MSDK repo so that their config files are loaded properly.

clang-format rules are loaded from the **.clang-format** and cpplint rules are loaded from **CPPLINT.cfg** in the root directory of the repo.  As a result, the utilities must also be run from the root directory of the repo.

#### cpplint

[cpplint](https://github.com/cpplint/cpplint) enforces good code practices by scanning for common mistakes and ensuring certain higher-level code patterns are followed.  It's a good idea to resolve the errors found by cpplint before before running clang-format, which deals with lower-level code styling and syntax patterns.

1. `cd` into the root directory of the MSDK repo.

2. Run cpplint.

    To run cpplint on a **file**, use `cpplint <filepath>`.

        $ cpplint Examples/MAX78000/Hello_World/main.c
        Done processing Examples/MAX78000/Hello_World/main.c

    To recursively run cpplint on an **entire directory**, use `cpplint --recursive <filepath>`.

        $ cpplint --recursive Examples/MAX78000

        Examples/MAX78000/ADC/example_config.h:1:  #ifndef header guard has wrong style, please use: EXAMPLES_MAX78000_ADC_EXAMPLE_CONFIG_H_  [build/header_guard] [5]
        Examples/MAX78000/ADC/example_config.h:14:  #endif line should be "#endif  // EXAMPLES_MAX78000_ADC_EXAMPLE_CONFIG_H_"  [build/header_guard] [5]
        Done processing Examples/MAX78000/ADC/example_config.h
        Done processing Examples/MAX78000/ADC/main.c
        Done processing Examples/MAX78000/AES/main.c
        Done processing Examples/MAX78000/ARM-DSP/arm_bayes_example/arm_bayes_example_f32.c
        ...

3. Resolve any errors.

4. `git add` and `git commit` any changes to your code.

#### clang-format

[clang-format](https://www.kernel.org/doc/html/latest/process/clang-format.html) is a code formatter and style checker that enforces a common style for the code-base.
**The MSDK uses version 14**, which is sometimes not available by default on some Linux distributions.  It can be retrieved from [https://apt.llvm.org/](https://apt.llvm.org/). 

1. `cd` into the root directory of the MSDK repo.

2. Run clang-format.

    To run a clang-format **_check_** on a file, use the `-n` "dry-run" flag.  

    `clang-format-14 --style=file --Werror --verbose -n <filename>`

    For example:

        $ clang-format-14 --style=file --Werror --verbose -n Examples/MAX78000/CRC/main.c

        Formatting [1/1] Examples/MAX78000/CRC/main.c
        Examples/MAX78000/CRC/main.c:86:40: error: code should be clang-formatted [-Wclang-format-violations]
            for (i = 0; i < DATA_LENGTH; i++) { array[i] = i; }
                                            ^

    To _apply_ the formatter to automatically format a file, use the `-i` flag.

    `clang-format-14 --style=file --Werror --verbose -i <filename>`

    For example:

        $ clang-format-14 --style=file --verbose -i Examples/MAX78000/CRC/main.c
        Formatting [1/1] Examples/MAX78000/CRC/main.c

    This will apply the formatter and overwrite the file.  Check the formatter's work using `git diff`.

        :::diff
        $ diff --git a/Examples/MAX78000/CRC/main.c b/Examples/MAX78000/CRC/main.c
        index 1dda1feed..c16ceb962 100644
        --- a/Examples/MAX78000/CRC/main.c
        +++ b/Examples/MAX78000/CRC/main.c
        @@ -83,7 +83,9 @@ void Test_CRC(int asynchronous)

            printf(asynchronous ? "TEST CRC ASYNC\n" : "TEST CRC SYNC\n");

        -    for (i = 0; i < DATA_LENGTH; i++) { array[i] = i; }
        +    for (i = 0; i < DATA_LENGTH; i++) {
        +        array[i] = i;
        +    }

    To apply the formatter to multiple files, the `*` and `**` wildcard characters can be used.  `*` matches any file or folder, and `**` _recursively_ matches any file or folder.

    To recursively run clang-format on all C files in an entire directory, use:

    `clang-format-14 --style=file --verbose -n <filepath>/**/*.c`.

    For example:

        $ clang-format-14 --style=file --verbose -i Examples/MAX78000/ARM-DSP/**/*.c

        Formatting [1/24] Examples/MAX78000/ARM-DSP/arm_bayes_example/arm_bayes_example_f32.c
        Formatting [2/24] Examples/MAX78000/ARM-DSP/arm_class_marks_example/arm_class_marks_example_f32.c
        Formatting [3/24] Examples/MAX78000/ARM-DSP/arm_convolution_example/arm_convolution_example_f32.c
        Formatting [4/24] Examples/MAX78000/ARM-DSP/arm_convolution_example/math_helper.c
        Formatting [5/24] Examples/MAX78000/ARM-DSP/arm_dotproduct_example_f32/arm_dotproduct_example_f32.c
        ...

    ... which runs the formatter for all C files in the `Examples/MAX78000/ARM-DSP` directory _and all its subdirectories_.  It should be noted that `**` is not supported on native Windows, but `*` is.

3. `git add` and `git commit` any changes to your code.  Now, it's ready for a PR!  The same checks will be automatically run against any PRs that are opened in the MSDK, and they must pass before the code can be approved.

## Contributing Examples

1. First, ensure that the example project has been linted and formatted to follow the [Style Guide](#style-guide)

2. Copy the example project into the [Examples](https://github.com/Analog-Devices-MSDK/msdk/tree/main/Examples) folder of the SDK for the applicable target microcontrollers.

3. `git add` and `git commit` the Example project.  **Commit your project files before running MSDKGen.**

4. Run the [MSDKGen](https://github.com/Analog-Devices-MSDK/MSDKGen) utility to ensure the example project's support files are updated to the latest version.

        python msdkgen.py update-all --projects yourprojectname --overwrite

5. Re-test the project if applicable.

6. If the updated files break any projects they can be restored to the previously working version using the `git restore` command.

    For example, the command below will restore all files in your current working directory.

        git restore **

    The `git diff` command can also be used to inspect local changes to help identify the root cause. Ex:

        git diff ./

## Contributing Libraries

Libraries should be added to the [Libraries](Libraries) sub-folder of the MSDK.

- All libraries should include a `libraryname.mk` file that can be added to `Libraries/libs.mk` via a toggle-switch.  The filename should match the name of library as closely as possible, and expose any required [configuration variables](https://github.com/Analog-Devices-MSDK/VSCode-Maxim/tree/develop#build-configuration).

- If necessary, a library may also include a "core" Makefile or set of Makefiles to build it as a standalone static library file.  The naming convention is `lib<libraryname>.a`.

### Self-Locating Makefile

The first thing that the `libraryname.mk` file should do is locate its own directory and store it in a variable.  The code snippet can be used to achieve this.

    :::Makefile
    ifeq "$(LIBRARYNAME_DIR)" ""
    # If LIBRARYNAME_DIR is not specified, this Makefile will locate itself.
    LIBRARYNAME_DIR := $(dir $(abspath $(lastword $(MAKEFILE_LIST))))
    endif

All filepaths for the library should then use this `$(LIBRARYNAME_DIR)`-type variable as their "root" for all filepaths.  This is a safe and reliable way to self-reference internal library files.

For an example, see the `Libraries/PeriphDrivers/periphdriver.mk` file.

### Simple Libraries

For simple libraries, it may be sufficient to just add the library's source files to the build using `VPATH` and `IPATH`.

For example:

    :::Makefile
    IPATH += $(LIBRARY_NAME_DIR)/include
    VPATH += $(LIBRARY_NAME_DIR)/src
    SRCS += libfile1.c
    SRCS += libfile2.c

An example of this is [MiscDrivers](Libraries/MiscDrivers/), which is a simple source-file-only library.  It gets its source code selectively added to the build via `Libraries/Boards/MAX78000/EvKit_V1/board.mk` files.

### Advanced Libraries

More advanced libraries (including those with a large number of source files) should include a rule to build as a static library file with a [recursive Make call](https://www.gnu.org/software/make/manual/make.html#Recursion).  

This type of library should also set up the appropriate [configuration variables](https://github.com/Analog-Devices-MSDK/VSCode-Maxim/tree/develop#build-configuration) to include that static library to the build.

## Contributing Documentation

### Code Maintainers

MSDK code should be documented using Doxygen syntax on all public functions, data structures, and variables.  See the [**DoxyGen Manual**](https://www.doxygen.nl/manual/docblocks.html) for more details on syntax for C-like languages.

DoxyGen is automatically run across the MSDK code as part of the User Guide's build process.  A Peripheral API reference is generated for each target microcontroller using the Doxygen files located in `Libraries/PeriphDrivers/Documentation`, and the output is packaged as a sub-component of the User Guide when it's built.  For code maintainers no action is needed other than maintaining up to date Doxygen documentation for all source code.

### User Guide

An MSDK User Guide is maintained in the [USERUIDE.md](USERGUIDE.md) file.  This document contains higher-level usage info for the MSDK.  If a part, IDE, or library is supported by the MSDK then there should be some relevant info in the User Guide covering its setup, configuration, and usage.

When writing markdown links, relative paths should always be used.  Additionally, links to local files on the user's filesystem **cannot** be used, since the online copy of the docs will throw a 404 on them.  See [Writing Your Docs](https://www.mkdocs.org/user-guide/writing-your-docs/) for more details.

Static resources such as images should be placed in the `res` folder.

### Building the Documentation

#### Local Builds

The `Documentation/build.py` script can be used to build the MSDK User Guide and supporting documentation locally.  This script:

- Builds all the Peripheral API references using Doxygen and copies them into `Documentation`
- Copies any markdown (.md) files from the root of the repo into `Documentation`
- Builds the MSDK User Guide using [Mkdocs](https://www.mkdocs.org/), which packages everything in `Documentation` into a static HTML/CSS/JavaScript site in the `docs` folder.

To **build** the docs:

1. Install [doxygen](https://www.doxygen.nl/download.html)
2. Install Python 3
3. `pip install -r Documentation/requirements.txt`
4. `python Documentation/build.py`
5. The site will be built in the `docs` folder of the repo.

To **preview** the generated site:

`mkdocs serve`

This will open a localhost test server with live re-loading.

To **configure** the generated site, use `mkdocs.yml`.  See [Mkdocs Configuration](https://www.mkdocs.org/user-guide/configuration/) for more details.

#### Auto Builds and Deployment

The **"Build & Deploy Docs"** Github Action will perform all the local steps above.  Additionally, it will deploy the static site to the `gh-pages` branch of the repository for use with [Github Pages](https://pages.github.com/).  When the `gh-pages` branch is updated, Pages will refresh the online copy of the documentation within ~5-10 mins.

This auto-deploy procedure is triggered on every push to the `release` branch.
