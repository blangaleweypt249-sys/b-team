#include <assert.h>
#include <stdint.h>

#include "fdcan_dlc.h"

/* 功能：验证 FDCAN 数据长度字段与字节数的换算；用途：防止 HAL DLC 编解码回归；返回 0 表示全部断言通过。 */
int main(void)
{
    uint32_t dlc;

    for (dlc = 0U; dlc <= 8U; dlc++)
    {
        assert(FdcanDlc_DecodeClassic(dlc) == (uint8_t)dlc);
        assert(FdcanDlc_EncodeClassic((uint8_t)dlc) == dlc);
    }
    assert(FdcanDlc_DecodeClassic(9U) == 8U);
    assert(FdcanDlc_DecodeClassic(15U) == 8U);
    return 0;
}
