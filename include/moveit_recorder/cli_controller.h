#ifndef CONSOLE_CONTROL_H
#define CONSOLE_CONTROL_H

#include <termios.h>
#include <unistd.h>
#include <cstdio>
#include <sys/select.h>

#define NB_ENABLE 0
#define NB_DISABLE 1

namespace recorder_utils
{
  int getch();
}

#endif
