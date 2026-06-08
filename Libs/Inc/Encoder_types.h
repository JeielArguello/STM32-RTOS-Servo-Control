/*
 * Encoder_types.h
 *
 *  Created on: May 11, 2026
 *      Author: ricardo
 */


#ifndef ENCODER_TYPES_H_
#define ENCODER_TYPES_H_

#define AS5600_ADDR (0x36 << 1)
#define ANGLE_REG_MSB 0x0E
#define AS5600_RESOLUTION 4096
#define SENSOR_SAMPLES 10


typedef struct {
    int32_t  rotations;      // 4 bytes - Cantidad de vueltas completas
    int32_t  angle_deg;      // 4 bytes - Ángulo absoluto en grados (rotations × 360 + sensor)
    int32_t  speed_rpm;      // 4 bytes - Velocidad en RPM
    uint16_t raw_position;   // 2 bytes - Valor crudo del registro (0-4095)
    int64_t  total_ticks;
    uint8_t  buffer[SENSOR_SAMPLES+1][2];      // 2 bytes - Buffer para el DMA (I2C)
    uint8_t  index;         // 1 byte  - Índice para el buffer circular
    uint8_t  status;         // 1 byte  - Flags de error o estado del sensor
    uint8_t  reserved;       // 1 byte  - Padding manual para alinear a 32 bits
} EncoderData_t;


#endif /* ENCODER_TYPES_H_ */
