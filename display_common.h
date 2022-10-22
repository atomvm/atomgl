#ifndef _DISPLAY_COMMON_H_
#define _DISPLAY_COMMON_H_

#include <globalcontext.h>
#include <term.h>

bool display_common_gpio_from_opts(term opts, const char *atom_str, int *gpio_num,
        GlobalContext *global);

#endif
