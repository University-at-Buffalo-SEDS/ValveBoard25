#ifndef LTC2990_H
#define LTC2990_H

#include "stm32g4xx_hal.h"
#include <math.h>

#include "main.h"

// I2C Register Addresses
#define STATUS_REG			(0x00)
#define CONTROL_REG			(0x01)
#define TRIGGER_REG			(0x02)
#define V1_MSB_REG			(0x06)
#define V1_LSB_REG			(0x07)
#define V2_MSB_REG			(0x08)
#define V2_LSB_REG			(0x09)
#define V3DV4_MSB_REG		(0x0A)
#define V3DV4_LSB_REG		(0x0B)

// Control Register Settings
#define VOLTAGE_MODE_MASK	(0x07)
#define TEMP_MEAS_MODE_MASK (0x18)

//Modes - Control register config values
#define V1_V2_V3_V4			(0x1F) //00011111 - This is repeat acquisition, all singled ended voltages enabled
#define V1DV2_V3DV4			(0x5E) //01011001 - This is single acquisition, V1-V2 and V3-V4 diff voltages enabled
#define CLEAR_ALL			(0xFF)
// Conversion Constants
#define CSINGLE_ENDED_LSB 	(0.318 / 16384.0)	//0.01554f//(60 / 16384.0)			//(0.318 / 16384.0) //should i try 0.00001942f // ex value (0.00000947581f)
#define VSINGLE_ENDED_LSB 	(5 / 16384.0)	//0.01554f//(60 / 16384.0)			//(0.318 / 16384.0) //should i try 0.00001942f // ex value (0.00000947581f)

#define RSENSE 0.02

// Timeout for data validity in milliseconds
#define TIMEOUT 1000

#define LTC2990_I2C_ADDRESS (0x4C)

typedef struct {
	I2C_HandleTypeDef *hi2c;
	uint8_t i2c_address;

	// Internal buffer for voltage readings
	float voltages[2];
	float current;
} LTC2990_Handle_t;

int LTC2990_Init(LTC2990_Handle_t *handle, I2C_HandleTypeDef *hi2c, uint8_t address);
void LTC2990_Step(LTC2990_Handle_t *handle);
float LTC2990_Get_Current(LTC2990_Handle_t *handle);
void LTC2990_Get_Single_Ended_Voltage(LTC2990_Handle_t *handle, float* volts);


int8_t LTC2990_Enable_V1DV2_V3DV4(LTC2990_Handle_t *handle);
int8_t LTC2990_Set_Mode(LTC2990_Handle_t *handle, uint8_t bits_to_set, uint8_t bits_to_clear);
int8_t LTC2990_Trigger_Conversion(LTC2990_Handle_t *handle);
uint8_t LTC2990_ADC_Read_New_Data(LTC2990_Handle_t *handle, uint8_t msb_register_address, uint16_t* adc_code, int8_t* data_valid);

float LTC2990_Code_To_Current(LTC2990_Handle_t *handle, uint16_t adc_code);
float LTC2990_Code_To_Diff_Voltage(LTC2990_Handle_t *handle, uint16_t adc_code);
float LTC2990_Code_To_Single_Ended_Voltage(LTC2990_Handle_t *handle, uint16_t adc_code);


//L2C Communication Helpers
int8_t LTC2990_Read_Register(LTC2990_Handle_t *handle, uint8_t reg_address, uint8_t* data);
int8_t LTC2990_Write_Register(LTC2990_Handle_t *handle, uint8_t reg_address, uint8_t data);

//For usb serial print
extern void CDC_Transmit_Print(const char * format, ...);

#endif
