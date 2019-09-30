/*  -*-  mode: c; tab-width: 4; indent-tabs-mode: t; c-basic-offset: 4; coding: utf-8  -*-  */
/************************************************************************************
 **                                                                                **
 **                                        UHSDR                                   **
 **               a powerful firmware for STM32 based SDR transceivers             **
 **                                                                                **
 **--------------------------------------------------------------------------------**
 **                                                                                **
 **  File name:		osc_FPGA_DDC.h                                                 **
 **  Description:   oscillator for FPGA DDC management                             **
 **  Licence:		GNU GPLv3                                                      **
 **  Author: 		Slawomir Balon/SP9BSL                                          **
 ************************************************************************************/

#ifndef UI_OSCILLATOR_OSC_FPGA_DDC_H_
#define UI_OSCILLATOR_OSC_FPGA_DDC_H_
#define oscDDC_f_sample 122880000

//TODO: move this structure from header file to source file!!
typedef struct
{
	bool is_present;
	uint8_t version_major;
	uint8_t version_minor;
	uint32_t current_frequency;
	uint32_t next_frequency;
	float32_t ppm;
	uint32_t RegConfig;
	uint8_t Nyquist_Zone;       //number of Nyquist zone for current frequency
	uint32_t AntiAliasFilterSeting;
	uint32_t prevAntiAliasFilterSeting;

}DDCboard_t;

enum DDCboard_{DDCboard_OK=0,DDCboard_Fail};
bool DDCboard_IsPresent(void);
void osc_FPGA_DDC_Init();

#endif /* UI_OSCILLATOR_OSC_FPGA_DDC_H_ */
