#ifndef CLI_H_
#define CLI_H_

#include "lfs.h"

/*
 * @brief Function to receive next command from the command line.
 *
 * @param cmd 	Buffer to store command into.
 * @param size 	Size of the command buffer.
 *
 * @return The size of the command if successful, otherwise an error code.
 */
int cmd_get(char *cmd, size_t size);

/*
 * @brief Function to process command and call appropriate command handler.
 *
 * @param lfs 	Pointer to mounted filesystem instance
 * @param cmd 	Buffer containing characters read from the command line.
 * @param size 	Number of characters in the command buffer.
 *
 * @return E_NO_ERROR if command processed successfully, otherwise an error code.
 */
int cmd_process(lfs_t *lfs, char *cmd, size_t size);

#endif // CLI_H_
