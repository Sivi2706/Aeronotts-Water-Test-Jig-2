#pragma once
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// Format: mm:ss:ms (synced everywhere from the same millis() sample)
void espTime_formatMmSsMs(uint32_t t_ms, char* out, size_t out_len);

#ifdef __cplusplus
}
#endif