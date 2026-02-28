#include <stdio.h>
#include <stdint.h>
#include <stddef.h>

void espTime_formatMmSsMs(uint32_t t_ms, char* out, size_t out_len) {
    uint32_t ms = t_ms % 1000;
    uint32_t total_s = t_ms / 1000;
    uint32_t s = total_s % 60;
    uint32_t m = total_s / 60;
    // mm:ss:ms
    snprintf(out, out_len, "%02lu:%02lu:%03lu",
             (unsigned long)m, (unsigned long)s, (unsigned long)ms);
}