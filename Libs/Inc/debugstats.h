#ifndef DEBUGSTATS_H
#define DEBUGSTATS_H

#include <stdint.h>

/* ============= Estructura de Estadísticas de Debug ============= */

typedef struct {
    uint32_t heap_free;
    uint32_t heap_min;
    uint16_t stack_driver;
    uint16_t stack_sensor;
    uint16_t stack_control;
    uint16_t stack_input_hid;
    uint16_t stack_output_hid;
    uint8_t memory_warning;
    // Timer2 Debug Info
    uint32_t timer_counter;
    uint32_t cc2_value;
    uint32_t tim_overflow_count;
    uint8_t tim_error_flags;
    uint32_t isr_frequency;
    // Motor Debug Info
    uint32_t motor_requested_steps;
    uint32_t motor_duration_ms;
    uint32_t motor_isr_count;
    uint32_t motor_steps_remaining;
    uint8_t motor_completed;
} DebugStats_t;

#endif /* DEBUGSTATS_H */
