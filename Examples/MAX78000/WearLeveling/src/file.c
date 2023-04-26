#include "file.h"
#include "lfs.h"
#include "mxc_errors.h"

int file_write(lfs_t *filesys, lfs_file_t *file, const char *filename, char *write_buf,
               uint32_t len, uint32_t pos, bool create)
{
    int err;
    int lfs_open_flags = LFS_O_WRONLY;

    // Check for bad params
    if (filesys == NULL || write_buf == NULL || filename == NULL) {
        return E_NULL_PTR;
    }

    // Set appropriate LFS flags
    if (create) {
        lfs_open_flags |= LFS_O_CREAT;
    }

    // Open up the file to write to
    err = lfs_file_open(filesys, file, filename, lfs_open_flags);
    if (err != LFS_ERR_OK) {
        printf("Unable to open file.\n");
        return err;
    }

    // Set write position within the file
    err = lfs_file_seek(filesys, file, pos, LFS_SEEK_SET);
    if (err < LFS_ERR_OK || err != pos) {
        printf("Unable to set write pointer to specified position.\n");
        lfs_file_close(filesys, file);
        return err;
    }

    // Write data to file
    err = lfs_file_write(filesys, file, write_buf, len);

    // Close the file. File not written to storage unless the file is explicitly closed.
    lfs_file_close(filesys, file);

    return err;
}

int file_read(lfs_t *filesys, lfs_file_t *file, const char *filename, char *read_buf, uint32_t len,
              uint32_t pos)
{
    int err;

    // Check for bad params
    if (filesys == NULL || read_buf == NULL || filename == NULL) {
        return E_NULL_PTR;
    }

    // Open up the file to read from
    err = lfs_file_open(filesys, file, filename, LFS_O_RDONLY);
    if (err != LFS_ERR_OK) {
        printf("Unable to open file.\n");
        return err;
    }

    // Set read position within the file
    err = lfs_file_seek(filesys, file, pos, LFS_SEEK_SET);
    if (err < LFS_ERR_OK || err != pos) {
        printf("Unable to set read pointer to specified position.\n");
        lfs_file_close(filesys, file);
        return err;
    }

    // Read from file
    err = lfs_file_read(filesys, file, (void *)read_buf, len);

    // Close the file.
    lfs_file_close(filesys, file);

    return err;
}
