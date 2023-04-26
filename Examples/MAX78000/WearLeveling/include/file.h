#ifndef FILE_H_
#define FILE_H_

#include <stdbool.h>
#include <stdint.h>
#include "lfs.h"

/***** Macros *****/
#define MAX_FILE_READ_SIZE 1024

/*
 * @brief Write data to a file.
 *
 * @param filesys 		Pointer to the LittleFS file system instance.
 * @param file 			Pointer to a LittleFS file instance.
 * @param filename 	 	Name of the file to write to.
 * @param write_buf 	Buffer containing the data to write to the file
 * @param len			Number of bytes to write to the file.
 * @param pos			Position within the file to start writing data at.
 * @param create 		Determines behavior if file doesn't already exist, "true" will create the
 * 						file and complete the write, and "false" will return an error.
 *
 * @return The number of bytes written to flash if successful, otherwise an error code.
 */
int file_write(lfs_t *filesys, lfs_file_t *file, const char *filename, char *write_buf,
               uint32_t len, uint32_t pos, bool create);

/*
 * @brief Read data from a file.
 *
 * @param filesys 		Pointer to the LittleFS file system instance.
 * @param file 			Pointer to a LittleFS file instance.
 * @param filename 		Name of the file to read from.
 * @param read_buf		Buffer to store data from the file.
 * @param len 			Number of bytes to read from the file.
 * @param pos			Position within the file to start reading data from.
 *
 * @return The number of bytes read if successful, otherwise an error code.
 */
int file_read(lfs_t *filesys, lfs_file_t *file, const char *filename, char *read_buf, uint32_t len,
              uint32_t pos);

#endif // FILE_H_
