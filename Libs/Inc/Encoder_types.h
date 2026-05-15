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


typedef struct {
    int32_t  rotations;      // 4 bytes - Cantidad de vueltas completas
    float    total_degrees;  // 4 bytes - Posición absoluta en grados
    float    velocity_dps;   // 4 bytes - Velocidad en grados/segundo
    uint16_t raw_angle;      // 2 bytes - Valor crudo del registro (0-4095)
    uint8_t  buffer[2];      // 2 bytes - Buffer para el DMA (I2C)
    uint8_t  status;         // 1 byte  - Flags de error o estado del sensor
    uint8_t  reserved;       // 1 byte  - Padding manual para alinear a 32 bits
} EncoderData_t;


#endif /* ENCODER_TYPES_H_ */
