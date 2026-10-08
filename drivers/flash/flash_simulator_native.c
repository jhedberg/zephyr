/*
 * Copyright (c) 2023 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Part of flash simulator which interacts with the host OS
 *
 * When building for the native simulator, this file is built in the
 * native simulator runner/host context, and not in Zephyr/embedded context.
 */

#undef _POSIX_C_SOURCE
/* Note: This is used only for interaction with the host C library, and is therefore exempt of
 * coding guidelines rule A.4&5 which applies to the embedded code using embedded libraries
 */
#define _POSIX_C_SOURCE 200809L

#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/mman.h>
#include <fcntl.h>
#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <nsi_tracing.h>

/* Create the folders of a file path which do not exist yet. Errors are left
 * for opening the file to report.
 */
static void create_folders(const char *path)
{
	char *folder;
	char *sep;

	folder = strdup(path);
	if (folder == NULL) {
		return;
	}

	for (sep = strchr(folder + 1, '/'); sep != NULL; sep = strchr(sep + 1, '/')) {
		*sep = '\0';
		(void)mkdir(folder, 0777);
		*sep = '/';
	}

	free(folder);
}

/*
 * Initialize the flash buffer.
 * And, if the content is to be kept on disk map it to the buffer to the file.
 *
 * Returns -1 on failure
 *	    0 on success
 */
int flash_mock_init_native(bool flash_in_ram, uint8_t **mock_flash, unsigned int size,
			   int *flash_fd, const char *flash_file_path,
			   unsigned int erase_value, bool flash_erase_at_start)
{
	struct stat f_stat;
	int rc;

	if (flash_in_ram == true) {
		*mock_flash = (uint8_t *)malloc(size);
		if (*mock_flash == NULL) {
			nsi_print_warning("Could not allocate flash in the process heap %s\n",
					    strerror(errno));
			return -1;
		}
	} else {
		/* As with the flash models of the nrf_bsim boards, the file can be in a
		 * folder of the simulation which is not there yet.
		 */
		create_folders(flash_file_path);

		*flash_fd = open(flash_file_path, O_RDWR | O_CREAT | O_CLOEXEC, (mode_t)0600);
		if (*flash_fd == -1) {
			nsi_print_warning("Failed to open flash device file "
					"%s: %s\n",
					flash_file_path, strerror(errno));
			return -1;
		}

		rc = fstat(*flash_fd, &f_stat);
		if (rc) {
			nsi_print_warning("Failed to get status of flash device file "
					"%s: %s\n",
					flash_file_path, strerror(errno));
			return -1;
		}

		if (ftruncate(*flash_fd, size) == -1) {
			nsi_print_warning("Failed to resize flash device file "
					"%s: %s\n",
					flash_file_path, strerror(errno));
			return -1;
		}

		*mock_flash = mmap(NULL, size,
				PROT_WRITE | PROT_READ, MAP_SHARED, *flash_fd, 0);
		if (*mock_flash == MAP_FAILED) {
			nsi_print_warning("Failed to mmap flash device file "
					"%s: %s\n",
					flash_file_path, strerror(errno));
			return -1;
		}
	}

	if ((flash_erase_at_start == true) || (flash_in_ram == true) || (f_stat.st_size == 0)) {
		/* Erase the memory unit by pulling all bits to the configured erase value */
		(void)memset(*mock_flash, erase_value, size);
	}

	return 0;
}

/*
 * If in RAM: Free the mock buffer
 * If in disk: unmap the flash file from RAM, close the file, and if configured to do so,
 * delete the file.
 */
void flash_mock_cleanup_native(bool flash_in_ram, int flash_fd, uint8_t *mock_flash,
			       unsigned int size, const char *flash_file_path,
			       bool flash_rm_at_exit)
{

	if (flash_in_ram == true) {
		if (mock_flash != NULL) {
			free(mock_flash);
		}
		return;
	}

	if ((mock_flash != MAP_FAILED) && (mock_flash != NULL)) {
		munmap(mock_flash, size);
	}

	if (flash_fd != -1) {
		close(flash_fd);
	}

	if ((flash_rm_at_exit == true) && (flash_file_path != NULL)) {
		/* We try to remove the file but do not error out if we can't */
		(void) remove(flash_file_path);
	}
}
