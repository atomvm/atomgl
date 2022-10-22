#include "display_common.h"

#include <interop.h>

bool display_common_gpio_from_opts(
    term opts, const char *atom_str, int *gpio_num, GlobalContext *global)
{
    int atom_index = globalcontext_insert_atom(global, atom_str);
    term gpio_atom = term_from_atom_index(atom_index);

    term gpio_term = interop_proplist_get_value(opts, gpio_atom);
    if (gpio_term == term_nil()) {
        return false;
    }

    *gpio_num = term_to_int(gpio_term);

    return true;
}
