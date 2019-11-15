#include <stdio.h>
#include <string.h>
#include <errno.h>
#include <unistd.h>
#include <stdlib.h>
#include <sys/types.h>
#include <sys/stat.h>

#include "tools.h"

int inputAvailable() {
	struct timeval tv;
	fd_set fds;
	tv.tv_sec = 0;
	tv.tv_usec = 0;
	FD_ZERO(&fds);
	FD_SET(STDIN_FILENO, &fds);
	select(STDIN_FILENO + 1, &fds, NULL, NULL, &tv);
	return (FD_ISSET(0, &fds));
}

int stat_mkdir(const char *filepath, mode_t mode) {
	struct stat sb = { 0 };
	int rc = 0;

	rc = stat(filepath, &sb);
	if (rc == 0) {
		if (!S_ISDIR(sb.st_mode)) {
			printf("Error: Not a directory: %s\n", filepath);
			return (-1);
		}
		return (0);
	}

	rc = mkdir(filepath, mode);
	if (rc < 0) {
		printf("Error: mkdir(%d) %s: %s\n", errno, strerror(errno), filepath);
		return (-1);
	}

	return (0);
}

int mkdir_path(const char *filepath, mode_t mode) {
	char *p = NULL;
	char *buf = NULL;
	int rc = 0;
	int len = strlen(filepath);

	buf = (char*) malloc(len + 2);
	if (buf == NULL) {
		printf("Error: malloc(%d) %s\n", errno, strerror(errno));
		return (-1);
	}
	strcpy(buf, filepath);
	if(buf[len - 1] != '/'){
		buf[len] = '/';
		len++;
		buf[len] = '\0';
	}

	for (p = strchr(buf + 1, '/'); p; p = strchr(p + 1, '/')) {
		*p = '\0';
		rc = stat_mkdir(buf, mode);
		if (rc != 0) {
			free(buf);
			return (-1);
		}
		*p = '/';
	}

	free(buf);
	return (0);
}

void strchg(char *buf, const char *str1, const char *str2)
{
  char tmp[1024 + 1];
  char *p;

  while ((p = strstr(buf, str1)) != NULL) {
    *p = '\0';
    p += strlen(str1);
    strcpy(tmp, p);
    strcat(buf, str2);
    strcat(buf, tmp);
  }
}
