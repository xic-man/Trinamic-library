#include <zephyr.h>
typedef union {
    uint32_t value;
    struct {
        uint32_t id   : 8,
                 axis : 8;
    };
} motor_map_t;
