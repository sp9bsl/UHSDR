/*  -*-  mode: c; tab-width: 4; indent-tabs-mode: t; c-basic-offset: 4; coding: utf-8  -*-  */
/************************************************************************************
 **                                                                                **
 **                                        UHSDR                                   **
 **               a powerful firmware for STM32 based SDR transceivers             **
 **                                                                                **
 **--------------------------------------------------------------------------------**
 **                                                                                **
 **  File name:		osc_FPGA_DDC.c                                                 **
 **  Description:   oscillator for FPGA DDC management                             **
 **  Licence:		GNU GPLv3                                                      **
 **  Author: 		Slawomir Balon/SP9BSL                                          **
 ************************************************************************************/
#include "main.h"
#include "uhsdr_board.h"
#ifdef USE_OSC_DDC
#include "stm32f7xx_hal.h"


#include <math.h>
#include "osc_FPGA_DDC.h"
#include <spi.h>

extern SAI_HandleTypeDef hsai_BlockA2;
extern SAI_HandleTypeDef hsai_BlockB2;
extern DMA_HandleTypeDef hdma_sai2_a;
extern DMA_HandleTypeDef hdma_sai2_b;

#define FPGA_CS_PIN GPIO_PIN_13
#define FPGA_CS_PORT GPIOC
#define hspiFPGA (hspi6)

#define DDCboard_REG_STAT 0
#define DDCboard_REG_CTRL 1
	#define DDCboard_REG_CTRL_SAIen 	0b00000001	//enable SAI outputs in FPGA
	#define DDCboard_REG_CTRL_SAITEST 	0b00000010	//test pattern for receiver (FPGA transmits: L:0x12345678 R:0x90123456 on its SAI output)
    #define DDCboard_REG_CTRL_RCV1revIQ 0b00000100  //exchange Receiver1 IQ (when working in even Nyquist Zone)

#define DDCboard_REG_RXfreq 	2
#define DDCboard_REG_TXfreq 	3
#define DDCboard_REG_ATT    	4
#define DDCboard_REG_TXlvl  	5


DDCboard_t DDCboard;

static inline void DDCboard_CSon(void)
{
	GPIO_ResetBits(FPGA_CS_PORT,FPGA_CS_PIN);
}

static inline void DDCboard_CSoff(void)
{
	GPIO_SetBits(FPGA_CS_PORT,FPGA_CS_PIN);
}

static uint32_t DDCboard_read(uint8_t regaddr)
{
    uint8_t tx_data[5];
    uint8_t rx_data[5];
    uint32_t retval=0;

    tx_data[0] = regaddr;
    tx_data[1] = 0;
    tx_data[2] = 0;
    tx_data[3] = 0;
    tx_data[4] = 0;

    DDCboard_CSon();
    if (HAL_SPI_TransmitReceive(&hspiFPGA,tx_data,rx_data, 5,100) == HAL_OK)
    {
    		retval = rx_data[1] << 24 | rx_data[2] << 16 | rx_data[3] << 8 |rx_data[4];
    }
    DDCboard_CSoff();
    return retval;
}

static uint32_t DDCboard_write(uint8_t regaddr,uint32_t data)
{
    uint8_t tx_data[5];
    uint32_t retval=DDCboard_Fail;

    tx_data[0] = regaddr;
    tx_data[1] = (uint8_t)(data>>24);
    tx_data[2] = (uint8_t)(data>>16);
    tx_data[3] = (uint8_t)(data>>8);
    tx_data[4] = (uint8_t)data;

    DDCboard_CSon();
    if (HAL_SPI_Transmit(&hspiFPGA,tx_data,5,100) == HAL_OK)
    {
    	retval=DDCboard_OK;
    }
    DDCboard_CSoff();
    return retval;
}


bool DDCboard_IsPresent(void)
{
	return DDCboard.is_present;
}

//we use SPI6 for communicating with the DDC board, if DDC Tag is received after checking REG0, function returns true
bool DDCboard_CheckPresence(void)
{
	HAL_GPIO_WritePin(FPGA_CS_PORT, FPGA_CS_PIN, GPIO_PIN_SET);

	GPIO_InitTypeDef GPIO_InitStruct;

    GPIO_InitStruct.Pin = FPGA_CS_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(FPGA_CS_PORT, &GPIO_InitStruct);

    // setup SPI with correct data
    hspiFPGA.Init.Mode = SPI_MODE_MASTER;
    hspiFPGA.Init.Direction = SPI_DIRECTION_2LINES;
    hspiFPGA.Init.DataSize = SPI_DATASIZE_8BIT;
    hspiFPGA.Init.CLKPolarity = SPI_POLARITY_HIGH;
    hspiFPGA.Init.CLKPhase = SPI_PHASE_2EDGE;
    hspiFPGA.Init.NSS = SPI_NSS_SOFT;
    hspiFPGA.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_64;
    hspiFPGA.Init.FirstBit = SPI_FIRSTBIT_MSB;

    if (HAL_SPI_Init(&hspi6) != HAL_OK)
    {
    	Error_Handler();
    }
    __HAL_SPI_ENABLE(&hspi6);

	DDCboard.version_major=0;
	DDCboard.version_minor=0;

	uint32_t stat=DDCboard_read(DDCboard_REG_STAT);
	uint16_t marker=stat>>16;

	if(marker==0xa55a)
	{
		DDCboard.version_major=stat>>8;
		DDCboard.version_minor=stat;
		return true;
	}


	return false;
}


//Reconfiguration of peripherials for DDC board usage
//this is necessary to keep compatibility with "conventional" analog RF boards.
//The main difference is that the source of I2S(SAI) clock is provided by FPGA board as the source of RF signal.
static void DDCboard_ConfigureSAI(void)
{
	GPIO_InitTypeDef GPIO_InitStruct;

	/**SAI2_B_Block_B GPIO Configuration
	    PE6     ------> SAI2_MCLK_B
	    PC0     ------> SAI2_FS_B
	    PA2     ------> SAI2_SCK_B
	    PG10     ------> SAI2_SD_B
	 */
	GPIO_InitStruct.Pin = GPIO_PIN_6;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	GPIO_InitStruct.Alternate = GPIO_AF10_SAI2;
	HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

	GPIO_InitStruct.Pin = GPIO_PIN_0;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	GPIO_InitStruct.Alternate = GPIO_AF8_SAI2;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	GPIO_InitStruct.Pin = GPIO_PIN_2;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	GPIO_InitStruct.Alternate = GPIO_AF8_SAI2;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	GPIO_InitStruct.Pin = GPIO_PIN_10;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	GPIO_InitStruct.Alternate = GPIO_AF10_SAI2;
	HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);



	/*Configure GPIO pin : PC9 */
	GPIO_InitStruct.Pin = GPIO_PIN_9;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	GPIO_InitStruct.Alternate = GPIO_AF5_SPI1;	//strange alternate name, but there is no I2S_CKIN definition in HAL :)
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	__HAL_RCC_SAI1_CONFIG(RCC_SAI1CLKSOURCE_PIN);
	__HAL_RCC_SAI2_CONFIG(RCC_SAI1CLKSOURCE_PIN);

	hsai_BlockA2.Instance = SAI2_Block_A;
	hsai_BlockA2.Init.AudioMode = SAI_MODESLAVE_RX;
	hsai_BlockA2.Init.Synchro = SAI_SYNCHRONOUS;
	hsai_BlockA2.Init.OutputDrive = SAI_OUTPUTDRIVE_DISABLE;
	hsai_BlockA2.Init.FIFOThreshold = SAI_FIFOTHRESHOLD_EMPTY;
	hsai_BlockA2.Init.SynchroExt = SAI_SYNCEXT_DISABLE;
	hsai_BlockA2.Init.MonoStereoMode = SAI_STEREOMODE;
	hsai_BlockA2.Init.CompandingMode = SAI_NOCOMPANDING;
	hsai_BlockA2.Init.TriState = SAI_OUTPUT_NOTRELEASED;
	if (HAL_SAI_InitProtocol(&hsai_BlockA2, SAI_I2S_STANDARD, SAI_PROTOCOL_DATASIZE_32BIT, 2) != HAL_OK)
	{
	  Error_Handler();
	}

	hsai_BlockB2.Instance = SAI2_Block_B;
	hsai_BlockB2.Init.AudioMode = SAI_MODESLAVE_TX;		//SAI_MODEMASTER_TX;
	hsai_BlockB2.Init.Synchro = SAI_ASYNCHRONOUS; 		//SAI_ASYNCHRONOUS;
	hsai_BlockB2.Init.OutputDrive = SAI_OUTPUTDRIVE_DISABLE;
	hsai_BlockB2.Init.NoDivider = SAI_MASTERDIVIDER_ENABLE;
	hsai_BlockB2.Init.FIFOThreshold = SAI_FIFOTHRESHOLD_EMPTY;
	hsai_BlockB2.Init.AudioFrequency = SAI_AUDIO_FREQUENCY_48K;
	hsai_BlockB2.Init.SynchroExt = SAI_SYNCEXT_DISABLE;
	hsai_BlockB2.Init.MonoStereoMode = SAI_STEREOMODE;
	hsai_BlockB2.Init.CompandingMode = SAI_NOCOMPANDING;
	hsai_BlockB2.Init.TriState = SAI_OUTPUT_NOTRELEASED;
	if (HAL_SAI_InitProtocol(&hsai_BlockB2, SAI_I2S_STANDARD, SAI_PROTOCOL_DATASIZE_32BIT, 2) != HAL_OK)
	{
	  Error_Handler();
	}


	//changing default DMA settings generated by CubeMX and used by normal UI board, to setting required for DDC/DUC
	//most of the structure data is the same like the CUbeMX generated, but for some more readability copied here too

	hdma_sai2_a.Instance = DMA2_Stream2;
	hdma_sai2_a.Init.Channel = DMA_CHANNEL_10;
	hdma_sai2_a.Init.Direction = DMA_PERIPH_TO_MEMORY;
	hdma_sai2_a.Init.PeriphInc = DMA_PINC_DISABLE;
	hdma_sai2_a.Init.MemInc = DMA_MINC_ENABLE;
	hdma_sai2_a.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
	hdma_sai2_a.Init.MemDataAlignment = DMA_MDATAALIGN_WORD;
	hdma_sai2_a.Init.Mode = DMA_CIRCULAR;
	hdma_sai2_a.Init.Priority = DMA_PRIORITY_LOW;
	hdma_sai2_a.Init.FIFOMode = DMA_FIFOMODE_ENABLE;
	hdma_sai2_a.Init.FIFOThreshold = DMA_FIFO_THRESHOLD_FULL;
	hdma_sai2_a.Init.MemBurst = DMA_MBURST_SINGLE;
	hdma_sai2_a.Init.PeriphBurst = DMA_PBURST_SINGLE;
	if (HAL_DMA_Init(&hdma_sai2_a) != HAL_OK)
	{
		Error_Handler();
	}

	hdma_sai2_b.Instance = DMA2_Stream6;
	hdma_sai2_b.Init.Channel = DMA_CHANNEL_3;
	hdma_sai2_b.Init.Direction = DMA_MEMORY_TO_PERIPH;
	hdma_sai2_b.Init.PeriphInc = DMA_PINC_DISABLE;
	hdma_sai2_b.Init.MemInc = DMA_MINC_ENABLE;
	hdma_sai2_b.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
	hdma_sai2_a.Init.MemDataAlignment = DMA_MDATAALIGN_WORD;
	hdma_sai2_b.Init.Mode = DMA_CIRCULAR;
	hdma_sai2_b.Init.Priority = DMA_PRIORITY_LOW;
	hdma_sai2_b.Init.FIFOMode = DMA_FIFOMODE_ENABLE;
	hdma_sai2_b.Init.FIFOThreshold = DMA_FIFO_THRESHOLD_FULL;
	hdma_sai2_b.Init.MemBurst = DMA_MBURST_SINGLE;
	hdma_sai2_b.Init.PeriphBurst = DMA_PBURST_SINGLE;
	if (HAL_DMA_Init(&hdma_sai2_b) != HAL_OK)
	{
		Error_Handler();
	}


}

static void DDCboard_SetPPM(float32_t ppm)
{
	DDCboard.ppm=ppm;
}

static Oscillator_ResultCodes_t DDCboard_PrepareNextFrequency(uint32_t freq, int temp_factor)
{
    //determining the Nyquist zone and set frequency to fit in 1st Nyquist zone

    if(freq>=(oscDDC_f_sample))
    {
        freq-=oscDDC_f_sample;
        DDCboard.Nyquist_Zone=3;
    }
    else if(freq>=(oscDDC_f_sample/2))
    {
        freq-=oscDDC_f_sample/2;
        DDCboard.Nyquist_Zone=2;
    }
    else
    {
        DDCboard.Nyquist_Zone=1;
    }

	DDCboard.next_frequency=freq;
	return OSC_OK;
}

static Oscillator_ResultCodes_t DDCboard_ChangeToNextFrequency()
{
	Oscillator_ResultCodes_t retval = OSC_OK;

	DDCboard_write(DDCboard_REG_RXfreq,DDCboard.next_frequency);

	return retval;
}

static bool DDCboard_IsNextStepLarge()
{
	return false;
}

/**
 * @brief Checks if all oscillator resources are available for switching frequency
 * It basically checks if the SPI is currently in use
 * This function must be called before changing the oscillator in interrupts
 * otherwise deadlocks may happen
 * @return true if it is safe to call oscillator functions in an interrupt
 */
static bool DDCboard_ReadyForIrqCall()
{
    return true;
}

static void DDCboard_UpdateConfig(uint8_t cfgValue, bool state)
{
	if(state)
		DDCboard.RegConfig|=cfgValue;
	else
		DDCboard.RegConfig&=~cfgValue;

	DDCboard_write(DDCboard_REG_CTRL,DDCboard.RegConfig);
}

const OscillatorInterface_t osc_FPGA_DDC =
{
		.init = osc_FPGA_DDC_Init,
		.isPresent = DDCboard_IsPresent,
		.setPPM = DDCboard_SetPPM,
		.prepareNextFrequency = DDCboard_PrepareNextFrequency,
		.changeToNextFrequency = DDCboard_ChangeToNextFrequency,
		.isNextStepLarge = DDCboard_IsNextStepLarge,
		.readyForIrqCall = DDCboard_ReadyForIrqCall,
};

void osc_FPGA_DDC_Init()
{
	DDCboard.current_frequency = 0;
	DDCboard.next_frequency = 0;

	DDCboard.is_present = DDCboard_CheckPresence();

	if (DDCboard.is_present)
	{
		DDCboard_ConfigureSAI();
		//DDCboard_UpdateConfig(DDCboard_REG_CTRL_SAIen | DDCboard_REG_CTRL_SAITEST,ENABLE);
		DDCboard_UpdateConfig(DDCboard_REG_CTRL_SAIen,ENABLE);

	}

	osc = DDCboard_IsPresent()?&osc_FPGA_DDC:NULL;

}
#endif


