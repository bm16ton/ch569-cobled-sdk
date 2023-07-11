#include <sys/stat.h>
int _fstat(int file, struct stat *st) {
  st->st_mode = S_IFCHR;
  return 0;
}

int _lseek(int file, int offset, int whence) {
  return 0;
}

int _close(int fd) {
  return -1;
}

int _isatty(int file) {
  return 1;
}


int _read (int file, char * ptr, int len) {
  int read = 0;

  if (file != 0) {
    return -1;
  }

  return read;
}


