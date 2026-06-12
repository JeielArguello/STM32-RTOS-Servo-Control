/*
 * Encoder_types.h
 * Created on: May 11, 2026
 * Author: ricardo
 */

#ifndef ENCODER_TYPES_H_
#define ENCODER_TYPES_H_

#include "main.h"

#define AS5600_ADDR       (0x36 << 1)
#define ANGLE_REG_MSB     0x0E
#define AS5600_RESOLUTION 4096
#define REG_STATUS        0x0B
#define REG_AGC           0x1A

// --- FLAGS DE ERROR PARA EL BYTE DE STATUS ---
#define ENCODER_OK            0x00  // Todo en orden
#define ENCODER_ERR_I2C       (1 << 0) // 0x01 - Fallo catastrófico de bus
#define ENCODER_ERR_NO_MAG    (1 << 1) // 0x02 - El chip no ve el imán (MD=0)
#define ENCODER_ERR_MAG_LOW   (1 << 2) // 0x04 - Imán muy lejos (ML=1)
#define ENCODER_ERR_MAG_HIGH  (1 << 3) // 0x08 - Imán muy cerca (MH=1)

typedef struct {
    int32_t  rotations;      // 4 bytes - Cantidad de vueltas completas
    int32_t  angle_deg;      // 4 bytes - Ángulo absoluto en grados
    int32_t  speed_rpm;      // 4 bytes - Velocidad en RPM
    uint16_t raw_position;   // 2 bytes - Valor crudo del registro (0-4095)
    int64_t  total_ticks;    // 8 bytes
    uint8_t  buffer[2];      // 2 bytes - Buffer para el DMA (I2C)
    uint8_t  status;         // 1 byte  - Flags de error estructurados
    uint8_t  reserved;       // 1 byte  - Padding manual para alinear a 32 bits
} EncoderData_t;

// Modificamos la firma para pasarle la estructura y que actualice sus flags internos
uint8_t Diagnosticar_AS5600(I2C_HandleTypeDef *hi2c, EncoderData_t *encoder);

#endif /* ENCODER_TYPES_H_ */