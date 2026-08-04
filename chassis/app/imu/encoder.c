#include "encoder.h"
#include "usart.h"
#include "print_util.h"

typedef struct {
    int32_t  erpm;
    int16_t  current_x10;     /* current * 10  (A*10), raw from STATUS_1 */
    int16_t  duty_x1000;      /* duty * 1000,  raw from STATUS_1 */
    int32_t  tachometer;      /* from STATUS_5 */
    uint32_t last_rx_ms;
    uint8_t  has_data;
} encoder_state_t;

static encoder_state_t s_enc[CHASSIS_WHEEL_COUNT];

/* Per-wheel RX frame counters (incremented in the CAN RX interrupt). */
static volatile uint32_t s_s1_cnt[CHASSIS_WHEEL_COUNT];
static volatile uint32_t s_s5_cnt[CHASSIS_WHEEL_COUNT];
/* Diagnostic baseline: counts + time captured at the last 'D' query. */
static uint32_t s_diag_ms;
static uint32_t s_diag_s1[CHASSIS_WHEEL_COUNT];
static uint32_t s_diag_s5[CHASSIS_WHEEL_COUNT];
static uint8_t  s_diag_base;

static int32_t parse_i32_be(const uint8_t *d)
{
    return (int32_t)(((uint32_t)d[0] << 24) | ((uint32_t)d[1] << 16) |
                     ((uint32_t)d[2] << 8)  |  (uint32_t)d[3]);
}

static int16_t parse_i16_be(const uint8_t *d)
{
    return (int16_t)((uint16_t)(((uint16_t)d[0] << 8) | (uint16_t)d[1]));
}

void encoder_on_rx(uint8_t controller_id, uint8_t packet,
                   const uint8_t *data, uint8_t len)
{
    const chassis_motor_t *m = chassis_find_by_id(controller_id);
    uint8_t i;

    if (m == NULL || data == NULL)
    {
        return;
    }
    i = (uint8_t)m->wheel;   /* registry is ordered by wheel -> index */

    if (packet == ENC_STATUS1_PACKET)
    {
        if (len >= 8U)
        {
            s_enc[i].erpm        = parse_i32_be(data);
            s_enc[i].current_x10 = parse_i16_be(data + 4);
            s_enc[i].duty_x1000  = parse_i16_be(data + 6);
            s_s1_cnt[i]++;
        }
        else
        {
            return;
        }
    }
    else if (packet == ENC_STATUS5_PACKET)
    {
        if (len >= 4U)
        {
            s_enc[i].tachometer = parse_i32_be(data);
            s_s5_cnt[i]++;
        }
        else
        {
            return;
        }
    }
    else
    {
        return;
    }

    s_enc[i].last_rx_ms = HAL_GetTick();
    s_enc[i].has_data   = 1U;
}

int32_t encoder_get_erpm(chassis_wheel_t w)
{
    return ((uint8_t)w < CHASSIS_WHEEL_COUNT) ? s_enc[(uint8_t)w].erpm : 0;
}

float encoder_get_current_a(chassis_wheel_t w)
{
    if ((uint8_t)w < CHASSIS_WHEEL_COUNT)
    {
        return (float)s_enc[(uint8_t)w].current_x10 / 10.0f;
    }
    return 0.0f;
}

float encoder_get_duty(chassis_wheel_t w)
{
    if ((uint8_t)w < CHASSIS_WHEEL_COUNT)
    {
        return (float)s_enc[(uint8_t)w].duty_x1000 / 1000.0f;
    }
    return 0.0f;
}

int32_t encoder_get_tachometer(chassis_wheel_t w)
{
    return ((uint8_t)w < CHASSIS_WHEEL_COUNT) ? s_enc[(uint8_t)w].tachometer : 0;
}

float encoder_get_distance_m(chassis_wheel_t w)
{
    if ((uint8_t)w < CHASSIS_WHEEL_COUNT)
    {
        float mech_revs = (float)s_enc[(uint8_t)w].tachometer / ENC_TACH_PER_MECH_REV;
        return mech_revs * 2.0f * 3.14159265f * ENC_WHEEL_RADIUS_M;
    }
    return 0.0f;
}

uint32_t encoder_get_age_ms(chassis_wheel_t w)
{
    if ((uint8_t)w < CHASSIS_WHEEL_COUNT && s_enc[(uint8_t)w].has_data)
    {
        return HAL_GetTick() - s_enc[(uint8_t)w].last_rx_ms;
    }
    return 0xFFFFFFFFU;
}

uint8_t encoder_has_data(chassis_wheel_t w)
{
    return ((uint8_t)w < CHASSIS_WHEEL_COUNT) ? s_enc[(uint8_t)w].has_data : 0U;
}

static void print_wheel(chassis_wheel_t w)
{
    static const char *const k_name[CHASSIS_WHEEL_COUNT] = { "LF", "RF", "LR", "RR" };
    const chassis_motor_t *m = chassis_find_by_wheel(w);

    usart1_puts("  ");
    usart1_puts(k_name[(uint8_t)w]);
    usart1_puts("(id ");
    put_i32(m ? (int32_t)m->id : -1);
    usart1_puts("): ");
    if (!encoder_has_data(w))
    {
        usart1_puts("no data\r\n");
        return;
    }
    put_i32(encoder_get_erpm(w));
    usart1_puts(" erpm, ");
    put_float(encoder_get_current_a(w), 2);
    usart1_puts(" A, duty ");
    put_float(encoder_get_duty(w), 3);
    usart1_puts(", tach ");
    put_i32(encoder_get_tachometer(w));
    usart1_puts(" (");
    put_float(encoder_get_distance_m(w), 3);
    usart1_puts(" m), ");
    put_i32((int32_t)encoder_get_age_ms(w));
    usart1_puts(" ms ago\r\n");
}

void encoder_print_all(void)
{
    int i;
    usart1_puts("encoders:\r\n");
    for (i = 0; i < CHASSIS_WHEEL_COUNT; i++)
    {
        print_wheel((chassis_wheel_t)i);
    }
}

void encoder_print(uint8_t controller_id)
{
    const chassis_motor_t *m = chassis_find_by_id(controller_id);
    if (m == NULL)
    {
        usart1_puts("id ");
        put_i32((int32_t)controller_id);
        usart1_puts(" not registered\r\n");
        return;
    }
    print_wheel(m->wheel);
}

/* ---- RX frame-rate diagnostic ---- */
static void diag_update_baseline(void)
{
    uint8_t i;
    for (i = 0U; i < CHASSIS_WHEEL_COUNT; i++)
    {
        s_diag_s1[i] = s_s1_cnt[i];
        s_diag_s5[i] = s_s5_cnt[i];
    }
    s_diag_ms   = HAL_GetTick();
    s_diag_base = 1U;
}

static void print_diag_wheel(uint8_t i, uint32_t dt)
{
    static const char *const k_name[CHASSIS_WHEEL_COUNT] = { "LF", "RF", "LR", "RR" };
    const chassis_motor_t *m = chassis_find_by_wheel((chassis_wheel_t)i);
    uint32_t c1 = s_s1_cnt[i];
    uint32_t c5 = s_s5_cnt[i];

    usart1_puts("  ");
    usart1_puts(k_name[i]);
    usart1_puts("(id ");
    put_i32(m ? (int32_t)m->id : -1);
    usart1_puts("): STATUS_1 ");
    put_i32((int32_t)c1);
    if (s_diag_base)
    {
        uint32_t d1 = c1 - s_diag_s1[i];
        usart1_puts(" (+");
        put_i32((int32_t)d1);
        if (dt > 0U)
        {
            usart1_puts(", ");
            put_i32((int32_t)((d1 * 1000U) / dt));
            usart1_puts(" Hz");
        }
        usart1_puts(")");
    }
    usart1_puts(" | STATUS_5 ");
    put_i32((int32_t)c5);
    if (s_diag_base)
    {
        uint32_t d5 = c5 - s_diag_s5[i];
        usart1_puts(" (+");
        put_i32((int32_t)d5);
        if (dt > 0U)
        {
            usart1_puts(", ");
            put_i32((int32_t)((d5 * 1000U) / dt));
            usart1_puts(" Hz");
        }
        usart1_puts(")");
    }
    usart1_puts("\r\n");
}

void encoder_diag_print_all(void)
{
    uint32_t now = HAL_GetTick();
    uint32_t dt  = now - s_diag_ms;
    uint8_t i;

    usart1_puts("rx diag");
    if (s_diag_base)
    {
        usart1_puts(", window ");
        put_i32((int32_t)dt);
        usart1_puts(" ms");
    }
    else
    {
        usart1_puts(" (first call - send D again after ~1s for rates)");
    }
    usart1_puts(":\r\n");

    for (i = 0U; i < CHASSIS_WHEEL_COUNT; i++)
    {
        print_diag_wheel(i, dt);
    }

    diag_update_baseline();
}

void encoder_diag_print(uint8_t controller_id)
{
    const chassis_motor_t *m = chassis_find_by_id(controller_id);
    uint32_t now, dt;
    uint8_t i;

    if (m == NULL)
    {
        usart1_puts("id ");
        put_i32((int32_t)controller_id);
        usart1_puts(" not registered\r\n");
        return;
    }

    i   = (uint8_t)m->wheel;
    now = HAL_GetTick();
    dt  = now - s_diag_ms;

    usart1_puts("rx diag");
    if (s_diag_base)
    {
        usart1_puts(", window ");
        put_i32((int32_t)dt);
        usart1_puts(" ms");
    }
    else
    {
        usart1_puts(" (first call - send D again after ~1s for rates)");
    }
    usart1_puts(":\r\n");

    print_diag_wheel(i, dt);
    diag_update_baseline();
}
