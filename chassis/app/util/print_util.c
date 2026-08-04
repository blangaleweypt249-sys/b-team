#include "print_util.h"
#include "usart.h"

void put_i32(int32_t v)
{
    char buf[12];
    char tmp[12];
    int bi = 0, tn = 0, k;
    uint32_t u;

    if (v < 0) { buf[bi++] = '-'; u = (uint32_t)(-v); }
    else       { u = (uint32_t)v; }

    if (u == 0U) { buf[bi++] = '0'; }
    else
    {
        while (u) { tmp[tn++] = (char)('0' + (u % 10U)); u /= 10U; }
        for (k = tn - 1; k >= 0; k--) { buf[bi++] = tmp[k]; }
    }
    buf[bi] = '\0';
    usart1_puts(buf);
}

void put_float(float v, uint8_t decimals)
{
    char buf[20];
    int bi = 0;
    uint32_t scale = 1U, ip, fp;
    int64_t scaled;
    char tmp[12];
    int tn = 0, k;
    uint8_t d;

    if (v < 0.0f) { buf[bi++] = '-'; v = -v; }
    for (d = 0; d < decimals; d++) { scale *= 10U; }
    scaled = (int64_t)(v * (float)scale + 0.5f);
    ip = (uint32_t)(scaled / (int64_t)scale);
    fp = (uint32_t)(scaled % (int64_t)scale);

    if (ip == 0U) { buf[bi++] = '0'; }
    else
    {
        while (ip) { tmp[tn++] = (char)('0' + (ip % 10U)); ip /= 10U; }
        for (k = tn - 1; k >= 0; k--) { buf[bi++] = tmp[k]; }
    }
    if (decimals > 0U)
    {
        buf[bi++] = '.';
        for (k = (int)decimals - 1; k >= 0; k--) { tmp[k] = (char)('0' + (fp % 10U)); fp /= 10U; }
        for (k = 0; k < (int)decimals; k++) { buf[bi++] = tmp[k]; }
    }
    buf[bi] = '\0';
    usart1_puts(buf);
}
