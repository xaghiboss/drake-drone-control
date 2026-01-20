#include "keyboard_input.h"

#include <unistd.h>
#include <sys/select.h>
#include <sys/time.h>
#include <cstdlib>  // atexit

namespace drake {

// Define the termios instance (one definition).
struct termios orig_termios;

void reset_terminal_mode() {
  tcsetattr(0, TCSANOW, &orig_termios);
}

void set_conio_terminal_mode() {
  struct termios new_termios;
  tcgetattr(0, &orig_termios);
  new_termios = orig_termios;
  atexit(reset_terminal_mode);
  cfmakeraw(&new_termios);
  tcsetattr(0, TCSANOW, &new_termios);
}

int kbhit() {
  struct timeval tv = {0L, 0L};
  fd_set fds;
  FD_ZERO(&fds);
  FD_SET(0, &fds);
  return select(1, &fds, NULL, NULL, &tv) > 0;
}

int getch() {
  int r;
  unsigned char c;
  if ((r = read(0, &c, sizeof(c))) < 0) {
    return r;
  } else {
    return c;
  }
}

int get_key() {
  if (!kbhit()) return 0;
  int c = getch();
  if (c == 27) {  // ESC
    if (!kbhit()) return 27;
    c = getch();
    if (c == '[') {
      if (!kbhit()) return 0;
      c = getch();
      switch (c) {
        case 'A': return KEY_ARROW_UP;
        case 'B': return KEY_ARROW_DOWN;
        case 'C': return KEY_ARROW_RIGHT;
        case 'D': return KEY_ARROW_LEFT;
      }
    }
  }
  return c;
}

}  // namespace drake
