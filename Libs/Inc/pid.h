#ifndef PID_H
#define PID_H

#include <stdint.h>

#define DEADBAND_TICKS  5

// Las funciones auxiliares definidas como macros puras
#define _constrain(amt, low, high) ((amt) < (low) ? (low) : ((amt) > (high) ? (high) : (amt)))
#define _isset(val) ((val) != 0)

// ============ FIXED-POINT Q16 (para PID sin floats) ============
// Q16: 16 bits decimales, 16 bits enteros
// 1.0 = 65536 (0x10000)
// 0.5 = 32768
// 0.001 = 65 (aprox)
#define Q16_SHIFT 16
#define Q16_ONE (1LL << Q16_SHIFT)     // 1.0 en Q16
#define Q16_FROM_INT(x) ((x) * Q16_ONE)
#define Q16_TO_INT(x) ((x + 0x8000) / Q16_ONE)
#define Q16_MUL(a, b) ((int64_t)(((a) * (b)) / Q16_ONE))
#define Q16_DIV(a, b) ((int64_t)(((a) * Q16_ONE) / (b)))

typedef uint32_t (*PID_GetTick_Fn)(void);

typedef struct {
    int64_t P;
    int64_t I;
    int64_t D;
    int64_t output_ramp;
    int64_t limit;
    int64_t Ts;
    int64_t error_prev;
    int64_t output_prev;
    int64_t integral_prev;
    uint32_t timestamp_prev;
    int64_t D_filtered; 
    int64_t alpha;      
    PID_GetTick_Fn get_tick_ms;
} PIDController;

// Funciones estándar en C
void PIDController_Init(PIDController* pid, int64_t P, int64_t I, int64_t D, int64_t ramp, int64_t limit, int64_t sampling_time, PID_GetTick_Fn get_tick_fn);
int64_t PIDController_Update(PIDController* pid, int64_t error);
void PIDController_Reset(PIDController* pid);

#endif // PID_H