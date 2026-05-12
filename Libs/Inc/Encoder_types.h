/*
 * Encoder_types.h
 *
 *  Created on: May 11, 2026
 *      Author: ricardo
 */

#define AS5600_ADDR (0x36 << 1)
#define ANGLE_REG_MSB 0x0E

#ifndef ENCODER_TYPES_H_
#define ENCODER_TYPES_H_



typedef struct {
	uint8_t buffer[2];
    uint16_t raw_angle;
    int32_t rotations;
    float total_degrees;
}EncoderData_t;


#endif /* ENCODER_TYPES_H_ */
