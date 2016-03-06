#include <stdio.h>
#include <stdlib.h>
#include <sys/dir.h>
#include <vector>
#include <string>
#include "dirlist.h"

std::vector<std::string> dir_to_list(const std::string &dirname) {
	std::vector<std::string> ret;
	DIR *dp = opendir(dirname.c_str());
	if (!dp) {
		perror(dirname.c_str());
		exit(EXIT_FAILURE);
	}
	struct dirent *de;
	while ((de = readdir(dp))) {
		if (de->d_name[0] != '.') {
			ret.push_back(de->d_name);
		}
	}
	closedir(dp);
	return ret;
}
