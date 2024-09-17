/*
 * clk_div_8V79S680.h
 *
 * Renesas 8V79S680 programmable Clock divider
 *
 *  Created on: Sep 18, 2023
 *      Author: Jakob Essbüchl
 */

#ifndef INC_CLK_DIV_8V79S680_H_
#define INC_CLK_DIV_8V79S680_H_

#include "main.h"

//register addresses
#define CLK_DIV_REG_SYSREF_POWER 0x18
#define CLK_DIV_REG_BIAS_TYPE_DELAY_MULT 0x19
#define CLK_DIV_REG_DELAY_CAL_FEEDBACK_DIV 0x1A
#define CLK_DIV_REG_DELAY_CAL_INPUT_DIV 0x1B
#define CLK_DIV_REG_CHANNEL_A_OUTPUT_DIVIDER 0x20
#define CLK_DIV_REG_CHANNEL_A_DELAY_CLK_A 0x21
#define CLK_DIV_REG_CHANNEL_A_PD 0x22
#define CLK_DIV_REG_OUTPUT_STATE_QCLK_A0 0x24
#define CLK_DIV_REG_OUTPUT_STATE_QCLK_A1 0x25
#define CLK_DIV_REG_OUTPUT_STATE_QCLK_A2 0x26
#define CLK_DIV_REG_REF_A0_DELAY_MUX_PD 0x28
#define CLK_DIV_REG_REF_A1_DELAY_MUX_PD 0x29
#define CLK_DIV_REG_REF_A2_DELAY_MUX_PD 0x2A
#define CLK_DIV_REG_OUTPUT_STATE_QREF_A0 0x2C
#define CLK_DIV_REG_OUTPUT_STATE_QREF_A1 0x2D
#define CLK_DIV_REG_OUTPUT_STATE_QREF_A2 0x2E
#define CLK_DIV_REG_CHANNEL_B_OUTPUT_DIVIDER 0x30
#define CLK_DIV_REG_CHANNEL_B_DELAY_CLK_B 0x31
#define CLK_DIV_REG_CHANNEL_B_PD 0x32
#define CLK_DIV_REG_OUTPUT_STATE_QCLK_B0 0x34
#define CLK_DIV_REG_OUTPUT_STATE_QCLK_B1 0x35
#define CLK_DIV_REG_REF_B0_DELAY_MUX_PD 0x38
#define CLK_DIV_REG_REF_B1_DELAY_MUX_PD 0x39
#define CLK_DIV_REG_OUTPUT_STATE_QREF_B0 0x3C
#define CLK_DIV_REG_OUTPUT_STATE_QREF_B1 0x3D
#define CLK_DIV_REG_CHANNEL_C_OUTPUT_DIVIDER 0x40
#define CLK_DIV_REG_CHANNEL_C_DELAY_CLK_C 0x41
#define CLK_DIV_REG_CHANNEL_C_PD 0x42
#define CLK_DIV_REG_OUTPUT_STATE_QCLK_C0 0x44
#define CLK_DIV_REG_OUTPUT_STATE_QCLK_C1 0x45
#define CLK_DIV_REG_REF_C0_DELAY_MUX_PD 0x48
#define CLK_DIV_REG_REF_C1_DELAY_MUX_PD 0x49
#define CLK_DIV_REG_OUTPUT_STATE_QREF_C0 0x4C
#define CLK_DIV_REG_OUTPUT_STATE_QREF_C1 0x4D
#define CLK_DIV_REG_CHANNEL_D_OUTPUT_DIVIDER 0x50
#define CLK_DIV_REG_CHANNEL_D_DELAY_CLK_D 0x51
#define CLK_DIV_REG_CHANNEL_D_PD 0x52
#define CLK_DIV_REG_OUTPUT_STATE_QCLK_D 0x54
#define CLK_DIV_REG_REF_D_DELAY_MUX_PD 0x58
#define CLK_DIV_REG_OUTPUT_STATE_QREF_D 0x5C
#define CLK_DIV_REG_DAC_CODE 0x6C
#define CLK_DIV_REG_GENERAL_CONTROL 0x6E
#define CLK_DIV_REG_GENERAL_CONTROL_INIT_CLK 0x71
#define CLK_DIV_REG_GENERAL_CONTROL_DCB_CAL 0x72
#define CLK_DIV_REG_GENERAL_CONTROL_PB_CAL 0x73
#define CLK_DIV_REG_OUTPUT_STATE_QCLK_EN 0x74
#define CLK_DIV_REG_OUTPUT_STATE_QREF_EN 0x76


//structs

/**
  * Represents the registers of Renesas_8V79S680 configurable clock divider.
  * registers contains register contents, masks contains masks that say which
  * bits are reserved.
  */
typedef struct Renesas_8V79S680 {
	uint8_t registers[0x76+1]; // register contents
	uint8_t masks[0x76+1]; // bit masks representing reserved bits
} Renesas_8V79S680;


//functions

/**
  * @brief  Initiate the 8V79S680 chip via SPI. Set (nearly) all required registers.
  * @param  hspi Pointer to a SPI_HandleTypeDef structure that contains
  *                the configuration information for the specified SPI.
  * @retval uint8_t 0 if successful, 1 if failed
  */
uint8_t CLK_DIV_Init(SPI_HandleTypeDef *hspi);


/**
  * @brief  Start clock delay calibration. This needs to be called with appropriate
  *         values after each input clock frequency change or after powering up
  * @param  hspi Pointer to a SPI_HandleTypeDef structure that contains
  *                the configuration information for the specified SPI.
  * @param  DLC uint8_t for DLC, P_DCB and M_DCB see the chip datasheet unter "calibration"
  * @param  P_DCB uint8_t
  * @param  M_DCB uint8_t
  * @retval uint8_t 0 if successful, 1 if failed
  */
uint8_t CLK_DIV_Calibrate(SPI_HandleTypeDef *hspi, uint8_t DLC, uint8_t P_DCB, uint16_t M_DCB);

/**
  * @brief  Enable the outputs. Should only be called after the chip was initialized and calibrated
  * @param  hspi Pointer to a SPI_HandleTypeDef structure that contains
  *                the configuration information for the specified SPI.
  * @retval void none
  */
void CLK_DIV_Output_Enable(SPI_HandleTypeDef *hspi);

/**
  * @brief  Read a register from the 8V79S680 chip via SPI
  * @param  hspi Pointer to a SPI_HandleTypeDef structure that contains
  *                the configuration information for the specified SPI.
  * @param  addr 7-bit address of the register (right aligned)
  * @param  byte uint8_t byte that the register contents will be written in
  * @param  length uint8_t number of registers (bytes) to read
  * @retval HAL status
  */
HAL_StatusTypeDef ReadRegister(SPI_HandleTypeDef *, uint8_t, uint8_t *, uint8_t);

/**
  * @brief  Write to a register from the 8V79S680 chip via SPI
  * @param  hspi Pointer to a SPI_HandleTypeDef structure that contains
  *                the configuration information for the specified SPI.
  * @param  addr 7-bit address of the register (right aligned)
  * @param  byte uint8_t byte that is the data that will be written
  * @retval HAL status
  */
HAL_StatusTypeDef WriteRegister(SPI_HandleTypeDef *, uint8_t, uint8_t);


#endif /* INC_CLK_DIV_8V79S680_H_ */
