#include "ssc_appl_singleton.h"
#include <stddef.h> // NULL


void empty_void_void_func(void) {
    // This function does nothing
}

struct SSC_appl_singleton SSC_appl_singleton_instance = {
    .APPL_Application = empty_void_void_func
};