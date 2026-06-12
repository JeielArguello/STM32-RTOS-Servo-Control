#include "Encoder_types.h"
#include <stdint.h>

uint8_t Diagnosticar_AS5600(I2C_HandleTypeDef *hi2c, EncoderData_t *encoder) {
    uint8_t reg_status = 0;
    
    // 1. Limpiamos errores previos antes de evaluar
    encoder->status = ENCODER_OK;


	// Escáner rápido de I2C para el AS5600
	HAL_StatusTypeDef resultado;

	// Enviamos un ping de prueba para ver si el chip responde en el bus
	resultado = HAL_I2C_IsDeviceReady(&ENCODER_I2C_HANDLE, AS5600_ADDR, 3, 100);

	if (resultado != HAL_OK) { 
        encoder->status |= ENCODER_ERR_I2C;
        return encoder->status;
	}

    // 2. Intentamos leer el registro de STATUS por I2C (Bloqueante corto, timeout de 10ms)
    // Usamos el AS5600_ADDR que ya tiene el shift de bits hecho.
    if (HAL_I2C_Mem_Read(hi2c, AS5600_ADDR, REG_STATUS, I2C_MEMADD_SIZE_8BIT, &reg_status, 1, 10) != HAL_OK) {
        encoder->status |= ENCODER_ERR_I2C;
        return encoder->status; // Si el bus murió, salimos volando de acá
    }

    // 3. Mapeo de bits del Datasheet del AS5600
    // Bit 5: MD (Magnet Detected), Bit 4: ML (Too Low), Bit 3: MH (Too High)
    uint8_t md = (reg_status >> 5) & 0x01;
    uint8_t ml = (reg_status >> 4) & 0x01;
    uint8_t mh = (reg_status >> 3) & 0x01;

    // 4. Cargamos nuestra estructura según las alertas
    if (md == 0) {
        encoder->status |= ENCODER_ERR_NO_MAG;
    }
    if (ml == 1) {
        encoder->status |= ENCODER_ERR_MAG_LOW;
    }
    if (mh == 1) {
        encoder->status |= ENCODER_ERR_MAG_HIGH;
    }

    return encoder->status;
}