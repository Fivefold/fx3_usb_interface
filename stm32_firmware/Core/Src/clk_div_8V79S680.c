/*
 * clk_div_8V79S680.c
 *
 *  Renesas 8V79S680 programmable Clock divider
 *
 *  Created on: Sep 18, 2023
 *      Author: Jakob Essbüchl
 */

#include "clk_div_8V79S680.h"
#include "main.h"
#include <stdint.h>
#include <string.h>

Renesas_8V79S680 CLK_DIV_t = {
	.registers = {0},
	.masks = {0}};

/**
 * @brief The function `compareBits` compares the bits of a given register content with a
 * given comparison string and returns a status code indicating whether the bits
 * match or not.
 *
 * @param register_content The register_content parameter is an 8-bit unsigned
 * integer that represents the content of a register.
 * @param compare The `compare` parameter is a string that represents a binary
 * pattern to compare against the `register_content`. Each character in the string
 * represents a bit, where '0' represents a bit that must be 0, '1' represents a
 * bit that must be 1, and 'x' represents a bit that can be any state
 *
 * @retval uint8_t 0 if successful, 1 or 2 if failed
 */
uint8_t compareBits(uint8_t register_content, const char *compare)
{
	int len = strlen(compare);

	// Ensure that the length of the compare string is 8 bits
	if (len != 8)
	{
		return 2; // Invalid comparison string
	}

	for (int i = 0; i < 8; i++)
	{
		char c = compare[i];
		if (c == '0' && !(register_content & (1 << (7 - i))))
		{
			continue; // Bit match
		}
		else if (c == '1' && (register_content & (1 << (7 - i))))
		{
			continue; // Bit match
		}
		else if (c == 'x')
		{
			continue; // Wildcard, don't care about the bit
		}
		else
		{
			return 1; // Bits don't match
		}
	}

	return 0; // All bits match
}

/**
 * The function sets the content and mask of a specific register in a
 * Renesas_8V79S680 struct.
 *
 * @param CLK_DIV_t CLK_DIV_t is a pointer to a structure of type Renesas_8V79S680.
 * @param address uint8_t The address parameter is used to specify the index of the
 * register in the registers array of the Renesas_8V79S680 structure. This index
 * determines which register will be modified.
 * @param mask uint8_t The "mask" parameter is used to specify which bits in the register
 * @param register_content uint8_t The register_content parameter is the value that you
 * want to write to the register at the specified address.
 * should be modified. It is a bitmask where each bit represents a specific bit in
 * the register. By setting a bit to 1 in the mask, you indicate that the
 * corresponding bit in the register should be modified.
 */
void setClkDivRegister(Renesas_8V79S680 *CLK_DIV_t, uint8_t address, uint8_t mask, uint8_t register_content)
{
	CLK_DIV_t->registers[address] = register_content;
	CLK_DIV_t->masks[address] = mask;
}

/*

char test[9] = "00000000\0";

printf("%s\n\n",test);
binaryToXString(0b00101111, test);
printf("%s\n\n",test);

 */

void binaryToXString(uint8_t num, char* result) {
    if (!result) {
        return;
    }

    if (strlen(result) < 8) {
        result[0] = '\0'; // Clear the result string if it's too short.
        return;
    }

    for (int i = 0; i < 8; i++) {
        if (num & (1 << (7 - i))) {
            result[i] = 'x';
        } else {
            result[i] = '0';
        }
    }

    result[8] = '\0'; // Null-terminate the string.
}

void InitializeClkDiv(Renesas_8V79S680 *CLK_DIV_t)
{
	/*----- set Dividers to 1:16 -----*/
	// setClkDivRegister(CLK_DIV_t, CLK_DIV_REG_CHANNEL_A_OUTPUT_DIVIDER, 0b11111000, 0x06);
	setClkDivRegister(CLK_DIV_t, CLK_DIV_REG_CHANNEL_B_OUTPUT_DIVIDER, 0b11111000, 0x06);
	setClkDivRegister(CLK_DIV_t, CLK_DIV_REG_CHANNEL_C_OUTPUT_DIVIDER, 0b11111000, 0x06);
	setClkDivRegister(CLK_DIV_t, CLK_DIV_REG_CHANNEL_D_OUTPUT_DIVIDER, 0b11111000, 0x06);

	/*----- set CLK phase delay -----*/
	uint8_t clk_delay = 0x00;
	// setClkDivRegister(CLK_DIV_t, CLK_DIV_REG_CHANNEL_A_DELAY_CLK_A, 0x00, clk_delay);
	setClkDivRegister(CLK_DIV_t, CLK_DIV_REG_CHANNEL_B_DELAY_CLK_B, 0x00, clk_delay);
	setClkDivRegister(CLK_DIV_t, CLK_DIV_REG_CHANNEL_C_DELAY_CLK_C, 0x00, clk_delay);
	setClkDivRegister(CLK_DIV_t, CLK_DIV_REG_CHANNEL_D_DELAY_CLK_D, 0x00, clk_delay);

	/*----- set 1:32 CLK (REF CLK) phase delay -----*/
	uint8_t ref_clk_delay = 0x00;
	// setClkDivRegister(CLK_DIV_t, CLK_DIV_REG_REF_A0_DELAY_MUX_PD, 0b11110001, ref_clk_delay << 1);
	// setClkDivRegister(CLK_DIV_t, CLK_DIV_REG_REF_A1_DELAY_MUX_PD, 0b11110001, ref_clk_delay << 1);
	// setClkDivRegister(CLK_DIV_t, CLK_DIV_REG_REF_A2_DELAY_MUX_PD, 0b11110001, ref_clk_delay << 1);
	setClkDivRegister(CLK_DIV_t, CLK_DIV_REG_REF_B0_DELAY_MUX_PD, 0b11110001, ref_clk_delay << 1);
	setClkDivRegister(CLK_DIV_t, CLK_DIV_REG_REF_B1_DELAY_MUX_PD, 0b11110001, ref_clk_delay << 1);
	setClkDivRegister(CLK_DIV_t, CLK_DIV_REG_REF_C0_DELAY_MUX_PD, 0b11110001, ref_clk_delay << 1);
	setClkDivRegister(CLK_DIV_t, CLK_DIV_REG_REF_C1_DELAY_MUX_PD, 0b11110001, ref_clk_delay << 1);
	setClkDivRegister(CLK_DIV_t, CLK_DIV_REG_REF_D_DELAY_MUX_PD, 0b11110001, ref_clk_delay << 1);

	/*----- disable unused channel A -----*/
	setClkDivRegister(CLK_DIV_t, CLK_DIV_REG_CHANNEL_A_PD, 0b01111111, 1 << 7);

	/*----- Set channel output style and amplitude -----*/
	uint8_t clk_output_state = 0b00011000;
	uint8_t clk_output_state_disabled = 0b10011000;
	setClkDivRegister(CLK_DIV_t, CLK_DIV_REG_OUTPUT_STATE_QCLK_A0, 0b01100011, clk_output_state_disabled); // Ax outputs are disabled
	setClkDivRegister(CLK_DIV_t, CLK_DIV_REG_OUTPUT_STATE_QCLK_A1, 0b01100011, clk_output_state_disabled);
	setClkDivRegister(CLK_DIV_t, CLK_DIV_REG_OUTPUT_STATE_QCLK_A2, 0b01100011, clk_output_state_disabled);
	setClkDivRegister(CLK_DIV_t, CLK_DIV_REG_OUTPUT_STATE_QCLK_B0, 0b01100011, clk_output_state);
	setClkDivRegister(CLK_DIV_t, CLK_DIV_REG_OUTPUT_STATE_QCLK_B1, 0b01100011, clk_output_state);
	setClkDivRegister(CLK_DIV_t, CLK_DIV_REG_OUTPUT_STATE_QCLK_C0, 0b01100011, clk_output_state);
	setClkDivRegister(CLK_DIV_t, CLK_DIV_REG_OUTPUT_STATE_QCLK_C1, 0b01100011, clk_output_state);
	setClkDivRegister(CLK_DIV_t, CLK_DIV_REG_OUTPUT_STATE_QCLK_D, 0b01100011, clk_output_state);

	setClkDivRegister(CLK_DIV_t, CLK_DIV_REG_OUTPUT_STATE_QREF_A0, 0b01100011, clk_output_state_disabled);
	setClkDivRegister(CLK_DIV_t, CLK_DIV_REG_OUTPUT_STATE_QREF_A1, 0b01100011, clk_output_state_disabled);
	setClkDivRegister(CLK_DIV_t, CLK_DIV_REG_OUTPUT_STATE_QREF_A2, 0b01100011, clk_output_state_disabled);
	setClkDivRegister(CLK_DIV_t, CLK_DIV_REG_OUTPUT_STATE_QREF_B0, 0b01100011, clk_output_state);
	setClkDivRegister(CLK_DIV_t, CLK_DIV_REG_OUTPUT_STATE_QREF_B1, 0b01100011, clk_output_state);
	setClkDivRegister(CLK_DIV_t, CLK_DIV_REG_OUTPUT_STATE_QREF_C0, 0b01100011, clk_output_state);
	setClkDivRegister(CLK_DIV_t, CLK_DIV_REG_OUTPUT_STATE_QREF_C1, 0b01100011, clk_output_state);
	setClkDivRegister(CLK_DIV_t, CLK_DIV_REG_OUTPUT_STATE_QREF_D, 0b01100011, clk_output_state);

	/*----- Power up 1:32 CLK (REF CLK) outputs -----*/
	setClkDivRegister(CLK_DIV_t, CLK_DIV_REG_SYSREF_POWER, 0b01111111, 1 << 7);
}

/**
 * @brief  Read a register from the 8V79S680 chip via SPI
 * @param  hspi Pointer to a SPI_HandleTypeDef structure that contains
 *                the configuration information for the specified SPI.
 * @param  addr 7-bit address of the register (right aligned)
 * @param  byte uint8_t byte that the register contents will be written in
 * @retval HAL status
 */
HAL_StatusTypeDef ReadRegister(SPI_HandleTypeDef *hspi, uint8_t addr, uint8_t *byte, uint8_t length)
{
	HAL_StatusTypeDef SPI_status;

	uint8_t tx_data[1];
	uint8_t rx_data[length];

	tx_data[0] = (addr << 1) | 0x01; // read operation

	HAL_GPIO_WritePin(SPI1_nCS_GPIO_Port, SPI1_nCS_Pin, GPIO_PIN_RESET);
	SPI_status = HAL_SPI_Transmit(hspi, tx_data, 1, SPI_TIMEOUT);
	if (SPI_status != HAL_OK)
	{
		HAL_GPIO_WritePin(SPI1_nCS_GPIO_Port, SPI1_nCS_Pin, GPIO_PIN_SET);
		return SPI_status;
	}

	SPI_status = HAL_SPI_Receive(hspi, rx_data, length, SPI_TIMEOUT);
	HAL_GPIO_WritePin(SPI1_nCS_GPIO_Port, SPI1_nCS_Pin, GPIO_PIN_SET);

	if (SPI_status == HAL_OK)
	{
		for (uint8_t i = 0; i < length; i++)
		{
			byte[i] = rx_data[i];
		}
	}
	return SPI_status;
}

/**
 * @brief  Write to a register from the 8V79S680 chip via SPI
 * @param  hspi Pointer to a SPI_HandleTypeDef structure that contains
 *                the configuration information for the specified SPI.
 * @param  addr 7-bit address of the register (right aligned)
 * @param  byte uint8_t byte that is the data that will be written
 * @retval HAL status
 */
HAL_StatusTypeDef WriteRegister(SPI_HandleTypeDef *hspi, uint8_t addr, uint8_t byte)
{
	HAL_StatusTypeDef SPI_status;

	uint8_t tx_data[2];

	tx_data[0] = (addr << 1) & 0xFE; // write operation
	tx_data[1] = byte;				 // dummy byte for response

	HAL_GPIO_WritePin(SPI1_nCS_GPIO_Port, SPI1_nCS_Pin, GPIO_PIN_RESET);
	SPI_status = HAL_SPI_Transmit(hspi, tx_data, 2, SPI_TIMEOUT);
	HAL_GPIO_WritePin(SPI1_nCS_GPIO_Port, SPI1_nCS_Pin, GPIO_PIN_SET);

	return SPI_status;
}

/**
 * @brief  Write to a register from the 8V79S680 chip via SPI
 * 		Works by utilizing Read-Modify-Write pattern to keep reserved bits
 * @param  hspi Pointer to a SPI_HandleTypeDef structure that contains
 *                the configuration information for the specified SPI.
 * @param  addr 7-bit address of the register (right aligned)
 * @param  mask uint8_t mask. 1s denote bits that keep the reserved state,
 *         0s get overwritten by the relevant bits in the 'byte' argument
 * @param  byte uint8_t byte that is the data that will be written
 * @retval HAL status
 */
HAL_StatusTypeDef UpdateRegister(SPI_HandleTypeDef *hspi, uint8_t addr, uint8_t mask, uint8_t byte)
{
	HAL_StatusTypeDef SPI_status;
	uint8_t register_content[1];
	uint8_t register_content_new = 0;
	SPI_status = ReadRegister(hspi, addr, register_content, 1);

	if (SPI_status != HAL_OK)
	{
		return SPI_status;
	}

	register_content_new = (register_content[0] & mask) | (byte & ~mask);

	SPI_status = WriteRegister(hspi, addr, register_content_new);

	return SPI_status;
}

uint8_t CLK_DIV_Init(SPI_HandleTypeDef *hspi)
{
	HAL_StatusTypeDef SPI_status;

	/*----- Deactivate CLock Frequency Divider Initializer Bypass  -----*/
	SPI_status = UpdateRegister(hspi, CLK_DIV_REG_GENERAL_CONTROL_DCB_CAL, 0b11111101, 1 << 1);

	/*----- set Dividers to 1:16 -----*/
	// SPI_status = UpdateRegister(hspi, CLK_DIV_REG_CHANNEL_A_OUTPUT_DIVIDER, 0b11111000, 0x06);
	SPI_status = UpdateRegister(hspi, CLK_DIV_REG_CHANNEL_B_OUTPUT_DIVIDER, 0b11111000, 0x00);
	SPI_status = UpdateRegister(hspi, CLK_DIV_REG_CHANNEL_C_OUTPUT_DIVIDER, 0b11111000, 0x00);
	SPI_status = UpdateRegister(hspi, CLK_DIV_REG_CHANNEL_D_OUTPUT_DIVIDER, 0b11111000, 0x06);

	/*----- set CLK phase delay -----*/
	uint8_t clk_delay = 0x00;
	// SPI_status = UpdateRegister(hspi, CLK_DIV_REG_CHANNEL_A_DELAY_CLK_A, 0x00, clk_delay);
	SPI_status = UpdateRegister(hspi, CLK_DIV_REG_CHANNEL_B_DELAY_CLK_B, 0x00, clk_delay);
	SPI_status = UpdateRegister(hspi, CLK_DIV_REG_CHANNEL_C_DELAY_CLK_C, 0x00, clk_delay);
	SPI_status = UpdateRegister(hspi, CLK_DIV_REG_CHANNEL_D_DELAY_CLK_D, 0x00, clk_delay);

	/*----- set 1:32 CLK (REF CLK) phase delay -----*/
	// shift 0: C1 | shift 1: C0 | shift 3: B1 | shift 4: B0
	uint8_t ref_clk_delay = 0x0;
	uint8_t ref_clk_delay2 = 0x6;
	// SPI_status = UpdateRegister(hspi, CLK_DIV_REG_REF_A0_DELAY_MUX_PD, 0b11110001, ref_clk_delay << 1);
	// SPI_status = UpdateRegister(hspi, CLK_DIV_REG_REF_A1_DELAY_MUX_PD, 0b11110001, ref_clk_delay << 1);
	// SPI_status = UpdateRegister(hspi, CLK_DIV_REG_REF_A2_DELAY_MUX_PD, 0b11110001, ref_clk_delay << 1);
	SPI_status = UpdateRegister(hspi, CLK_DIV_REG_REF_B0_DELAY_MUX_PD, 0b11110001, 0x2 << 1); // channel 4
	SPI_status = UpdateRegister(hspi, CLK_DIV_REG_REF_B1_DELAY_MUX_PD, 0b11110001, 0x2 << 1); // channel 3
	SPI_status = UpdateRegister(hspi, CLK_DIV_REG_REF_C0_DELAY_MUX_PD, 0b11110001, 0x2 << 1); // channel 2
	SPI_status = UpdateRegister(hspi, CLK_DIV_REG_REF_C1_DELAY_MUX_PD, 0b11110001, 0x2 << 1); // channel 1
	SPI_status = UpdateRegister(hspi, CLK_DIV_REG_REF_D_DELAY_MUX_PD, 0b11110001, ref_clk_delay << 1);

	/*----- disable unused channel A -----*/
	SPI_status = UpdateRegister(hspi, CLK_DIV_REG_CHANNEL_A_PD, 0b01111111, 1 << 7);

	/*----- Set channel output style and amplitude -----*/
	uint8_t clk_output_state = 0b00011100;
	uint8_t clk_output_state_lvds = 0b00001000;
	uint8_t clk_output_state_disabled = 0b10011100;
	SPI_status = UpdateRegister(hspi, CLK_DIV_REG_OUTPUT_STATE_QCLK_A0, 0b01100011, clk_output_state_disabled); // Ax outputs are disabled
	SPI_status = UpdateRegister(hspi, CLK_DIV_REG_OUTPUT_STATE_QCLK_A1, 0b01100011, clk_output_state_disabled);
	SPI_status = UpdateRegister(hspi, CLK_DIV_REG_OUTPUT_STATE_QCLK_A2, 0b01100011, clk_output_state_disabled);
	SPI_status = UpdateRegister(hspi, CLK_DIV_REG_OUTPUT_STATE_QCLK_B0, 0b01100011, clk_output_state);
	SPI_status = UpdateRegister(hspi, CLK_DIV_REG_OUTPUT_STATE_QCLK_B1, 0b01100011, clk_output_state);
	SPI_status = UpdateRegister(hspi, CLK_DIV_REG_OUTPUT_STATE_QCLK_C0, 0b01100011, clk_output_state);
	SPI_status = UpdateRegister(hspi, CLK_DIV_REG_OUTPUT_STATE_QCLK_C1, 0b01100011, clk_output_state);
	SPI_status = UpdateRegister(hspi, CLK_DIV_REG_OUTPUT_STATE_QCLK_D, 0b01100011, clk_output_state_lvds);

	SPI_status = UpdateRegister(hspi, CLK_DIV_REG_OUTPUT_STATE_QREF_A0, 0b01100011, clk_output_state_disabled);
	SPI_status = UpdateRegister(hspi, CLK_DIV_REG_OUTPUT_STATE_QREF_A1, 0b01100011, clk_output_state_disabled);
	SPI_status = UpdateRegister(hspi, CLK_DIV_REG_OUTPUT_STATE_QREF_A2, 0b01100011, clk_output_state_disabled);
	SPI_status = UpdateRegister(hspi, CLK_DIV_REG_OUTPUT_STATE_QREF_B0, 0b01100011, clk_output_state);
	SPI_status = UpdateRegister(hspi, CLK_DIV_REG_OUTPUT_STATE_QREF_B1, 0b01100011, clk_output_state);
	SPI_status = UpdateRegister(hspi, CLK_DIV_REG_OUTPUT_STATE_QREF_C0, 0b01100011, clk_output_state);
	SPI_status = UpdateRegister(hspi, CLK_DIV_REG_OUTPUT_STATE_QREF_C1, 0b01100011, clk_output_state);
	SPI_status = UpdateRegister(hspi, CLK_DIV_REG_OUTPUT_STATE_QREF_D, 0b01100011, clk_output_state);

	/*----- Power up 1:32 CLK (REF CLK) outputs -----*/
	SPI_status = UpdateRegister(hspi, CLK_DIV_REG_SYSREF_POWER, 0b01111111, 0 << 7);

	return 0;
}

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
uint8_t CLK_DIV_Calibrate(SPI_HandleTypeDef *hspi, uint8_t DLC, uint8_t P_DCB, uint16_t M_DCB)
{
	HAL_StatusTypeDef SPI_status;
	uint8_t DAC_reg[2] = {0};
	uint8_t DCB_CAL_reg[1] = {0xFF};
	uint16_t DAC_code = 0;
	uint8_t DCB_reg[3] = {0};

	uint16_t DLC_M_DCB_reg = M_DCB | (DLC << 13 & 0b00000011); // register content for addresses 0x19 and 0x1A. They span two 8-bit registers

	SPI_status = UpdateRegister(hspi, CLK_DIV_REG_BIAS_TYPE_DELAY_MULT, 0b10011110, (uint8_t)DLC_M_DCB_reg >> 8);
	SPI_status = UpdateRegister(hspi, CLK_DIV_REG_DELAY_CAL_FEEDBACK_DIV, 0xFF, (uint8_t)DLC_M_DCB_reg);
	SPI_status = UpdateRegister(hspi, CLK_DIV_REG_DELAY_CAL_INPUT_DIV, 0b01111111, P_DCB);


	/*----- Initiate precision bias calibration and deactivate internal bypass -----*/
	SPI_status = UpdateRegister(hspi, CLK_DIV_REG_GENERAL_CONTROL_PB_CAL, 0b01111101, 0xF0);
	/*----- Initiate delay calibration block (DCB) calibration -----*/
	SPI_status = UpdateRegister(hspi, CLK_DIV_REG_GENERAL_CONTROL_DCB_CAL, 0b01111111, 1 << 7);
	/*----- Initiate clock divider and clock phase delay functions -----*/
	SPI_status = UpdateRegister(hspi, CLK_DIV_REG_GENERAL_CONTROL_INIT_CLK, 0b01111111, 1 << 7);

	/*----- Verify correct calibration -----*/
	while((DCB_CAL_reg[0] & 0b10000000) != 0) {
		SPI_status = ReadRegister(hspi, CLK_DIV_REG_GENERAL_CONTROL_DCB_CAL, DCB_CAL_reg, 1);
	}

	SPI_status = ReadRegister(hspi, CLK_DIV_REG_DAC_CODE, DAC_reg, 2);
	DAC_code = ((uint16_t)DAC_reg[0] << 8) | DAC_reg[1];

	/*
	SPI_status = ReadRegister(hspi, CLK_DIV_REG_DAC_CODE, DAC_reg, 2);
	DAC_code = ((uint16_t)DAC_reg[0] << 8) | DAC_reg[1];

	SPI_status = ReadRegister(hspi, CLK_DIV_REG_BIAS_TYPE_DELAY_MULT, DCB_reg, 3);*/

	if (DAC_code < 1 || DAC_code >= 32767)
	{
		return 1; // DAC calibration wasn't successful
	}

	return 0;
}

/**
 * @brief  Enable the outputs. Should only be called after the chip was initialized and calibrated
 * @param  hspi Pointer to a SPI_HandleTypeDef structure that contains
 *                the configuration information for the specified SPI.
 */
void CLK_DIV_Output_Enable(SPI_HandleTypeDef *hspi)
{
	HAL_StatusTypeDef SPI_status;

	/*----- Enable CLK outputs -----*/
	SPI_status = UpdateRegister(hspi, CLK_DIV_REG_OUTPUT_STATE_QCLK_EN, 0x00, 0xFF);
	/*----- Enable 1:32 CLK (REF CLK) outputs -----*/
	SPI_status = UpdateRegister(hspi, CLK_DIV_REG_OUTPUT_STATE_QREF_EN, 0x00, 0xFF);
}
