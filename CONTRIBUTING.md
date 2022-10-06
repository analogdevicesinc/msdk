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
* New and/pr modified example branch: `example/branchname`

## Style Guide

The MSDK code-base, for the most-part, follows the [Linux Kernel coding style](https://www.kernel.org/doc/html/v4.10/process/coding-style.html#linux-kernel-coding-style) _and_ [Google's C++ Style Guide](https://google.github.io/styleguide/cppguide.html) with the following exception(s):

* Indentations are 4 spaces.

Formatting and styling is enforced via [clang-format](https://www.kernel.org/doc/html/latest/process/clang-format.html) and [cpplint](https://github.com/cpplint/cpplint), which automatically run checks against all PRs.  Before submitting your code to the MSDK and as the final step of development you should format and lint it with these tools.

### Running the Linter & Formatter

Both utilities can be run locally.  cpplint should be run first, then clang-format.  Additionally, both should be run from the root directory of the MSDK repo so that their config files are loaded properly.

clang-format rules are loaded from [.clang-format](.clang-format) and cpplint rules are loaded from [CPPLINT.cfg](CPPLINT.cfg).

1. `cd` into the root directory of the MSDK repo.

2. Run cpplint and resolve any errors.

    cpplint checks enforces good code practices, and deals with some common mistakes and higher-level code patterns.  It's a good idea to resolve its errors first before running clang-format, which deals with lower-level code styling and syntax patterns.

    To run cpplint on a file, use `cpplint <filepath>`.

    ```shell
    $ cpplint Examples/MAX78000/Hello_World/main.c
    Done processing Examples/MAX78000/Hello_World/main.c
    ```

    To recursively run cpplint on an entire directory, use `cpplint --recursive <filepath>`.

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

3. Run clang-format.

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

    This will show where any formatting errors lie.

    To _apply_ the formatter to automatically format a file, use the `-i` flag.

    `clang-format --style=file --Werror --verbose -i <filename>`

    For example:

    ```shell
    clang-format --style=file --verbose -i Examples/MAX78000/CRC/main.c
    Formatting [1/1] Examples/MAX78000/CRC/main.c
    ```

    This will apply the formatter and overwrite the file.  It's a good idea to check the formatter's work using `git diff`.

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

