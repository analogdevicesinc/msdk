# MSDK CLI Library

## Description

#### What is a Command Line Interface(CLI)?

A command-line interface or command language interpreter (CLI), also known as command-line user interface, console user interface, and character user interface (CUI), is a means of interacting with an embedded system where the user (or client) issues commands to the program in the form of successive lines of text (command lines).

This library provides an extensible command processor to:

- Quickly and easily wrap user functions with a CLI
- Allow developers to get diagnostics and change device parameters interactively
- Easily automate testing of the device, through PC-side scripting

The current CLI Library includes the following features:

- UART interface support
- Custom commands with support for multiple arguments
- No modification of functions required.  Users implement a command table and simple wrapper functions.
- Case insensitive commands & arguments.
- White-space insensitive.
- Easy built-in 'help' command and custom help strings.

## Usage Guide

To use the CLI library, users are expected to implement the following steps in their application:

1. Add the following line to your [project.mk](../../USERGUIDE.md#build-configuration-variables) file to enable the CLI library for your project:

        :::Makefile
        LIB_CLI = 1

2. `#include "cli.h"` in your application code.

3. Define an array of type `const command_t`.  This is your command table.  Include an array element for each command you want your CLI to support. Each element should define:

    1. The name of the command
    2. A string showing how to enter the command in the terminal
    3. A description of what the command does
    4. A function pointer to a wrapper function.

    For example, the following is subset of the SDHC example command set:

        :::C
        const command_t user_commands[] = {{ "format", "format", "Format the Card", handle_format },
                                        { "mkdir", "mkdir <directory name>", "Create a directory", handle_mkdir }}

4. Implement a handler function for each command.

    The handler functions provided to the `user_commands` table are "wrappers" around the code that should be run for each received command.  The handlers must conform to the following definition:

    Parameters

    - `int argc`   - This tells the handler function how many arguments were received.
    - `char* argv[]` - This array holds the received arguments (in order) as strings. (Note: argv[0] holds the command string, argv[1:argc] are the arguments (if any are passed), and the last element in the argument vector is always a NULL pointer.)

    Return Value

    - The function needs to return an integer type. It should return 0 (E_NO_ERROR) if the command was executed successfuly, otherwise it should return a negative integer as an error code.

    Below is a sample handler function for a "make directory" command, where `mkdir` is some function in the user's application code that does the work.

        :::C
        int handle_mkdir(int argc, char *argv[]) {
            mkdir(argv[1]);
        }

    As an example, suppose a user entered the command:

        :::C
        mkdir new_folder

    The CLI library will tokenize the command string "mkdir new_folder" into "mkdir" and "new_folder" and assigns them to argv[0] and argv[1] respectively. The library would then determine that this is the "make directory" command and would call "handle_mkdir" with argc=2 and a pointer to the argument vector.

5. Add a call to `MXC_CLI_Init` to the application's startup code.  

    Pass in the UART instance for the CLI to use, a pointer to the command table, and the number of commands in the command table.

6. (OPTIONAL) The CLI library will enable and handle UART interrupts automatically.  Users who would like more control over the interrupt handling have the option to disable the default handler and define their own.

    To do so, set the `LIB_CLI_USE_DEFAULT_HANDLER` [build configuration variable](../../USERGUIDE.md#build-configuration-variables) to `0`.  Users may now enable and handle the interrupt with a custom function.  However, users must call `MXC_CLI_Handler()` from the custom function for the CLI to work.
