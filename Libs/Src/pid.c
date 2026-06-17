#include "pid.h"

void PIDController_Init(PIDController* pid, int64_t P, int64_t I, int64_t D, int64_t ramp, int64_t limit, int64_t sampling_time, PID_GetTick_Fn get_tick_fn) {
    pid->P = P;
    pid->I = I;
    pid->D = D;
    pid->output_ramp = ramp;
    pid->limit = limit;
    pid->Ts = sampling_time;
    
    pid->error_prev = 0;
    pid->output_prev = 0;
    pid->integral_prev = 0;
    pid->D_filtered = 0; // Inicializar estado del filtro derivativo
    pid->alpha = 10923; // 0.5 en Q16 para filtro
    pid->get_tick_ms = get_tick_fn;
    pid->timestamp_prev = pid->get_tick_ms();
}

int64_t PIDController_Update(PIDController* pid, int64_t error) {
    int64_t dt = pid->Ts;
    int64_t limit_q16 = Q16_FROM_INT(pid->limit);

    
    if (!_isset(dt)) {
        uint32_t timestamp_now = pid->get_tick_ms();
        // dt en ticks del sistema
        dt = (int64_t)(timestamp_now - pid->timestamp_prev);
        
        if (dt <= 0 || dt > 500000) dt = 1000; // 1ms por defecto si hay overflow (500ms max)
        pid->timestamp_prev = timestamp_now;
    }

    // Convertir dt_ms a fracción de segundo en formato Q16.16 (dt = ms / 1000)
    int64_t dt_secs_q16 = Q16_DIV(dt, 1000);
    if (dt_secs_q16 == 0) dt_secs_q16 = 1; // Failsafe anti división por cero

    // u_p = P * e(k)
    int64_t proportional = Q16_MUL(pid->P, Q16_FROM_INT(error));

    // u_ik = u_ik_1 + I * Ts * 0.5 * (ek + ek_1)
    int64_t integral;
    int64_t half = 32768; // 0.5 en Q16
    integral = pid->integral_prev + Q16_MUL(Q16_MUL(Q16_MUL(pid->I, dt_secs_q16), Q16_FROM_INT(error + pid->error_prev)),half);

    if (_isset(pid->limit)) integral = _constrain(integral, -limit_q16, limit_q16);

    // Derivada cruda (como ya la tenés)
    int64_t derivative_raw = Q16_DIV(Q16_MUL(pid->D, Q16_FROM_INT(error - pid->error_prev)), dt_secs_q16);

    // Filtro pasabajos exponencial sobre el término derivativo
    // pid->D_filtered es estado persistente, debe inicializarse en 0
    int64_t delta = derivative_raw - pid->D_filtered;
    pid->D_filtered = pid->D_filtered + Q16_MUL(pid->alpha, delta);

    int64_t derivative = pid->D_filtered;        

    // Sumar componentes
    int64_t output = proportional + integral + derivative;
    if (_isset(pid->limit)) output = _constrain(output, -limit_q16, limit_q16);

    // Límite de rampa
    if (_isset(pid->output_ramp) && pid->output_ramp > 0) {
        int64_t output_rate;
        output_rate = Q16_DIV((output - pid->output_prev), dt_secs_q16);
        
        int64_t output_ramp_q16 = Q16_FROM_INT(pid->output_ramp); // en Q16
        if (output_rate > output_ramp_q16) {
            output = pid->output_prev + Q16_MUL(output_ramp_q16, dt_secs_q16);
        } else if (output_rate < -output_ramp_q16) {
            output = pid->output_prev - Q16_MUL(output_ramp_q16, dt_secs_q16);
        }
    }

    // Guardar estados
    pid->integral_prev = integral;
    pid->output_prev = output;
    pid->error_prev = error;

    int64_t output_int = Q16_TO_INT(output);
    return output_int;
}

void PIDController_Reset(PIDController* pid) {
    pid->integral_prev = 0;
    pid->output_prev = 0;
    pid->error_prev = 0;
    pid->D_filtered = 0;
    pid->timestamp_prev = pid->get_tick_ms();
}