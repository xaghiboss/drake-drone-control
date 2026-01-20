#ifndef KEYBOARD_INPUT_H_
#define KEYBOARD_INPUT_H_

#include <termios.h>

namespace drake {

// Only declare the global terminal state here — define it once in keyboard_input.cc.
extern struct termios orig_termios;

void reset_terminal_mode();

void set_conio_terminal_mode();

int kbhit();

int getch();

enum {
  KEY_ARROW_UP = 1001,
  KEY_ARROW_DOWN = 1002,
  KEY_ARROW_LEFT = 1003,
  KEY_ARROW_RIGHT = 1004
};

int get_key();

}  // namespace drake

#endif  // KEYBOARD_INPUT_H_
