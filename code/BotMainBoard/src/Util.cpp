
#include <stdlib.h>
#include <chrono>
#include <unistd.h>
#include <thread>
#include <iomanip>

#include "setup.h"
#include "Util.h"
#include <cstdarg>


float roundValue(float x) {
	float roundedValue = sgn(x)*((int)(abs(x)*10.0f+.5f))/10.0f;
	return roundedValue;
}

std::string string_format(const std::string &fmt, ...) {
    int size=100;
    std::string str;
    va_list ap;

    while (1) {
        str.resize(size);
        va_start(ap, fmt);
        int n = vsnprintf(&str[0], size, fmt.c_str(), ap);
        va_end(ap);

        if (n > -1 && n < size)
            return str;
        if (n > -1)
            size = n + 1;
        else
            size *= 2;
    }
}

long mapLong(long x, long in_min, long in_max, long out_min, long out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

int randomInt(int min,int max) {
	int r = rand() % (max-min) + min;
	return r;
}

rational randomFloat (rational a, rational b) {
	return randomInt(a*1000, b*1000)/1000.;
}

bool randomBool() {
	return randomInt(0,100)>50;
}

int randomPosNeg() {
	return (randomInt(0,100)>50)?+1:-1;
}

unsigned long millis() {
    auto epoch = std::chrono::high_resolution_clock::from_time_t(0);
    auto now   = std::chrono::high_resolution_clock::now();
    auto mseconds = std::chrono::duration_cast<std::chrono::milliseconds>(now - epoch).count();
    return mseconds;
}


void delay(long ms) {
	std::this_thread::sleep_for(std::chrono::milliseconds(ms));
}

string to_string(rational number, int precision) {
	return
			static_cast< std::ostringstream & >(
					(std::ostringstream() << std::setprecision(precision) <<  number)
			).str();
}

vector<std::string> readDirectory(const string & dir, const string& ext) {
  string item;
  int i = 0;

  vector<string> result;
  string filepattern = "*.";
  filepattern += ext;

#ifdef _WIN32

  WIN32_FIND_DATA FN;
  HANDLE hFind;

  string path = "*." + ext;
  hFind = FindFirstFile(path.c_str(), &FN);
    if (hFind != INVALID_HANDLE_VALUE) {
      do {
        int len = strlen(FN.cFileName);
        string item;

        if (FN.dwFileAttributes & FILE_ATTRIBUTE_DIRECTORY) {
        	item = '\\';
        	item += FN.cFileName;
        } else {
        	item = FN.cFileName;
        }
        	result.push_back(item);
        i++;
      } while (FindNextFile(hFind, &FN) != 0);

      if (GetLastError() == ERROR_NO_MORE_FILES)
        FindClose(hFind);
      else
        LOG(ERROR) << "failed read dir";
    }

#elif defined(__GNUC__)

  DIR *dir;
  struct dirent *dirp;
  struct stat dr;

  if (list) {
    list->delete_all();
    if ((dir = opendir(d)) == NULL)
      LOG(ERROR) << "failed read dir";
    else {
      while ((dirp = readdir(dir)) != NULL)   /* open directory     */
      {
    	 string item;
        if (!lstat(dirp->d_name,&dr) && S_ISDIR(dr.st_mode)) /* dir is directory   */
          item = dirp->d_name + "/";
        else
          item = dirp->d_name;

        result.push_back(item);
        i++;
      }
      closedir(dir);
    }
  }
#endif

  return result;
}
