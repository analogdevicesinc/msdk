# Analog Devices MSDK Development and Contribution Guidelines

## Development Flow

Development for the MSDK also follows the official [GitHub development flow guidelines](https://docs.github.com/en/get-started/quickstart/github-flow).

For beginners, [learngitbranching.js.org](https://learngitbranching.js.org/) is a great hands-on starting resource.

## Contribution Guidelines

The MSDK follows the [GitHub contribution guidelines](https://docs.github.com/en/get-started/quickstart/contributing-to-projects).  

External contributions from outside the [Analog Devices organization](https://github.com/Analog-Devices-MSDK) should be made via a Pull Request opened from a fork.  Internal contributions should also preferrably use a fork where possible.

If a direct branch on the mainline MSDK repo is made, the following branch naming conventions should be used when possible:

* Bugfix/ticket: `fix/branchname`
    * For Jira tickets, it's recommended to use `fix/ticketnumber` so that the branch gets automatically tracked.  Ex: `fix/MSDK-670`
* New feature: `feature/branchname`
* Generic development branch: `dev/branchname`
* New and/or modified example branch: `example/branchname`

## Style Guide

The MSDK code-base, for the most-part, follows the [Linux Kernel coding style](https://www.kernel.org/doc/html/v4.10/process/coding-style.html#linux-kernel-coding-style) _and_ [Google's C++ Style Guide](https://google.github.io/styleguide/cppguide.html) with the following exception(s):

* Indentations are 4 spaces.

Formatting and styling is enforced via [clang-format](https://www.kernel.org/doc/html/latest/process/clang-format.html) and [cpplint](https://github.com/cpplint/cpplint), which automatically run checks against all PRs.  A PR cannot be merged until it passes these checks.

### Running the Linter & Formatter

Both utilities can be run locally.  cpplint should be run first, then clang-format.  Additionally, both should be run from the root directory of the MSDK repo so that their config files are loaded properly.

clang-format rules are loaded from [.clang-format](.clang-format) and cpplint rules are loaded from [CPPLINT.cfg](CPPLINT.cfg).

#### cpplint

[cpplint](https://github.com/cpplint/cpplint) enforces good code practices by scanning for common mistakes and ensuring certain higher-level code patterns are followed.  It's a good idea to resolve the errors found by cpplint before before running clang-format, which deals with lower-level code styling and syntax patterns.

1. `cd` into the root directory of the MSDK repo.

2. Run cpplint.

    To run cpplint on a **file**, use `cpplint <filepath>`.

    ```shell
    $ cpplint Examples/MAX78000/Hello_World/main.c
    Done processing Examples/MAX78000/Hello_World/main.c
    ```

    To recursively run cpplint on an **entire directory**, use `cpplint --recursive <filepath>`.

    ```shell
    $ cpplint --recursive Examples/MAX78000

    Examples/MAX78000/ADC/example_config.h:1:  #ifndef header guard has wrong style, please use: EXAMPLES_MAX78000_ADC_EXAMPLE_CONFIG_H_  [build/header_guard] [5]
    Examples/MAX78000/ADC/example_config.h:14:  #endif line should be "#endif  // EXAMPLES_MAX78000_ADC_EXAMPLE_CONFIG_H_"  [build/header_guard] [5]
    Done processing Examples/MAX78000/ADC/example_config.h
    Done processing Examples/MAX78000/ADC/main.c
    Done processing Examples/MAX78000/AES/main.c
    Done processing Examples/MAX78000/ARM-DSP/arm_bayes_example/arm_bayes_example_f32.c
    ...
    ```

3. Resolve any errors.

4. `git add` and `git commit` any changes to your code.

#### clang-format

[clang-format](https://www.kernel.org/doc/html/latest/process/clang-format.html) is a code formatter and style checker that enforces a common style for the code-base.

1. `cd` into the root directory of the MSDK repo.

2. Run clang-format.

    The `--style=file --Werror --verbose` options are shared across all runs.

    To run a clang-format _check_ on a file, use the `-n` "dry-run" flag.  

    `clang-format --style=file --Werror --verbose -n <filename>`

    For example:

    ```shell
    $ clang-format --style=file --Werror --verbose -n Examples/MAX78000/CRC/main.c

    Formatting [1/1] Examples/MAX78000/CRC/main.c
    Examples/MAX78000/CRC/main.c:86:40: error: code should be clang-formatted [-Wclang-format-violations]
        for (i = 0; i < DATA_LENGTH; i++) { array[i] = i; }
                                           ^
    ```

    To _apply_ the formatter to automatically format a file, use the `-i` flag.

    `clang-format --style=file --Werror --verbose -i <filename>`

    For example:

    ```shell
    clang-format --style=file --verbose -i Examples/MAX78000/CRC/main.c
    Formatting [1/1] Examples/MAX78000/CRC/main.c
    ```

    This will apply the formatter and overwrite the file.  Check the formatter's work using `git diff`.

    ```diff
    diff --git a/Examples/MAX78000/CRC/main.c b/Examples/MAX78000/CRC/main.c
    index 1dda1feed..c16ceb962 100644
    --- a/Examples/MAX78000/CRC/main.c
    +++ b/Examples/MAX78000/CRC/main.c
    @@ -83,7 +83,9 @@ void Test_CRC(int asynchronous)

        printf(asynchronous ? "TEST CRC ASYNC\n" : "TEST CRC SYNC\n");

    -    for (i = 0; i < DATA_LENGTH; i++) { array[i] = i; }
    +    for (i = 0; i < DATA_LENGTH; i++) {
    +        array[i] = i;
    +    }
    ```

    To apply the formatter to multiple files, the `*` and `**` wildcard characters can be used.  `*` matches any file or folder, and `**` _recursively_ matches any file or folder.

    To recursively run clang-format on all C files in an entire directory, use:

    `clang-format --style=file --verbose -n <filepath>/**/*.c`.

    For example:

    ```shell
    $ clang-format --style=file --verbose -i Examples/MAX78000/ARM-DSP/**/*.c

    Formatting [1/24] Examples/MAX78000/ARM-DSP/arm_bayes_example/arm_bayes_example_f32.c
    Formatting [2/24] Examples/MAX78000/ARM-DSP/arm_class_marks_example/arm_class_marks_example_f32.c
    Formatting [3/24] Examples/MAX78000/ARM-DSP/arm_convolution_example/arm_convolution_example_f32.c
    Formatting [4/24] Examples/MAX78000/ARM-DSP/arm_convolution_example/math_helper.c
    Formatting [5/24] Examples/MAX78000/ARM-DSP/arm_dotproduct_example_f32/arm_dotproduct_example_f32.c
    ...
    ```

    ... which runs the formatter for all C files in the `Examples/MAX78000/ARM-DSP` directory _and all its subdirectories_.

4. `git add` and `git commit` any changes to your code.  Now, it's ready for a PR!  The same checks will be automatically run against any PRs that are opened in the MSDK, and they must pass before the code can be approved.

## Examples

### Updating and Adding Examples

1. First, ensure that the example project has been linted and formatted to follow the [Style Guide](#style-guide)

2. Copy the example project into the [Examples](https://github.com/Analog-Devices-MSDK/msdk/tree/main/Examples) folder of the SDK for the applicable target microcontrollers.

3. `git add` and `git commit` the Example project.  Use a commit message such as "Add xxx Example"

4. Run the [MSDKGen](https://github.com/Analog-Devices-MSDK/MSDKGen) utility to ensure the example project's support files are updated to the latest version.

    If you are coming from the **_legacy_ Makefile system** (no project.mk) it's best to migrate the old Makefile in its own commit...

    1. Generate the new Makefile first using the `--no-vscode`, `--no-eclipse`, and `--backup` options. Ex:

        ```shell
        python msdkgen.py update-all --projects yourprojectname --no-vscode --no-eclipse --overwrite --backup
        ```

    2. A project.mk file will be added, and the old Makefile will be replaced but _backed up_ to a file called `Makefile-backup.mk`.

    3. Migrate any project-specific build settings to project.mk.  Documentation on the project.mk system can be found [here](https://github.com/Analog-Devices-MSDK/VSCode-Maxim/tree/develop).

    4. `git add` and `git commit` the new Makefile and project.mk files.  Delete the backup file.

    5. Run a full `update-all`, which will then update the VSCode and Eclipse files as well.

    Otherwise, if the project is already running a [project.mk](https://github.com/Analog-Devices-MSDK/VSCode-Maxim/tree/develop#build-configuration) file then a full `update-all` command can be used.

    Ex:

    ```shell
    python msdkgen.py update-all --projects yourprojectname --overwrite
    ```

5. If the updated files break any projects they can be restored to the previously working version using the `git restore` command.

    For example, the command below will restore all files in your current working directory.

    ```shell
    git restore **
    ```

    The `git diff` command can also be used to inspect local changes to help identify the root cause. Ex:

    ```shell
    git diff ./
    ```

    If MSDKGen breaks a previously working project _that has already been migrated to the project.mk build system_ please report it to the [MSDKGen Issue Tracker](https://github.com/Analog-Devices-MSDK/MSDKGen/issues)

## Libraries

Libraries should be added to the [Libraries](Libraries) sub-folder of the MSDK.

* All libraries should include a `libraryname.mk` file that can be added to [libs.mk](Libraries/libs.mk) via a toggle-switch.  The filename should match the name of library as closely as possible, and expose any required [configuration variables](https://github.com/Analog-Devices-MSDK/VSCode-Maxim/tree/develop#build-configuration).

* If necessary, a library may also include a "core" Makefile or set of Makefiles to build it as a standalone static library file.  The naming convention is `lib<libraryname>.a`.

### Self-Locating Makefile

The first thing that the `libraryname.mk` file should do is locate its own directory and store it in a variable.  The code snippet can be used to achieve this.

```Makefile
ifeq "$(LIBRARYNAME_DIR)" ""
# If LIBRARYNAME_DIR is not specified, this Makefile will locate itself.
LIBRARYNAME_DIR := $(dir $(abspath $(lastword $(MAKEFILE_LIST))))
endif
```

All filepaths for the library should then use this `$(LIBRARYNAME_DIR)`-type variable as their "root" for all filepaths.  This is a safe and reliable way to self-reference internal library files.

For an example, see the [periphdriver.mk](Libraries/PeriphDrivers/periphdriver.mk) file.

### Simple Libraries

For simple libraries, it may be sufficient to just add the library's source files to the build using `VPATH` and `IPATH`.

For example:

```Makefile
IPATH += $(LIBRARY_NAME_DIR)/include
VPATH += $(LIBRARY_NAME_DIR)/src
SRCS += libfile1.c
SRCS += libfile2.c
```

An example of this is [MiscDrivers](Libraries/MiscDrivers/), which is a simple source-file-only library.  It gets its source code selectively added to the build via [board.mk](Libraries/Boards/MAX78000/EvKit_V1/board.mk) files.

### Advanced Libraries

More advanced libraries (including those with a large number of source files) should include a rule to build as a static library file with a [recursive Make call](https://www.gnu.org/software/make/manual/make.html#Recursion).  

This type of library should also set up the appropriate [configuration variables](https://github.com/Analog-Devices-MSDK/VSCode-Maxim/tree/develop#build-configuration) to include that static library to the build.
