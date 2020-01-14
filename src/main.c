/**
  ******************************************************************************
  * @file    main.c
  * @author  Ac6
  * @version V1.0
  * @date    01-December-2013
  * @brief   Default main function.
  ******************************************************************************
*/

#include <stdlib.h>
#include "stm32f4xx.h"

#include "../PDM2PCM_Library/Inc/pdm2pcm_glo.h"

//Function Declarations
void Config_DecimationFilter();
void Config_I2SInterface();
void Config_Clocks();
void Config_DMA();

//Global variables
uint16_t* pdm_buffer;
uint16_t* pcm_buffer;
int samplesReady = 0;
PDM_Filter_Handler_t pHandler; //Static Parameters Structure
PDM_Filter_Config_t pConfig; //Dynamic Parameters Structure


int main(void)
{
	pcm_buffer = (uint16_t*)malloc(256 * sizeof(uint16_t)); //receiving 256 PCM samples per call to decimation filter
	pdm_buffer = (uint16_t*)malloc(8192 * sizeof(uint16_t)); //Must cast as uint8_t* when passing to filtering library since it requires byte-packed
	Config_Clocks();
	Config_I2SInterface();
	Config_DecimationFilter();
	Config_DMA();

	while(1)
	{
		//Sample data from mic and load into PDM buffer

		if(samplesReady)
		{
			//Filter PDM data to PCM format:
			PDM_Filter(&pdm_buffer[0], &pcm_buffer[0], &pHandler);
			//Low pass filter to cutoff aliased frequencies above Nyquist frequency (use CMSIS library)
			//Take FFT of PCM data
			//Display data
		}
	}
}

void Config_Clocks()
{
	RCC_DeInit();
	//Default: HSE_Value = 8MHz in stm32f4xx.h
	RCC_HSEConfig(RCC_HSE_ON);
	while(RCC_WaitForHSEStartUp() != SUCCESS);
	//Set I2S_CLK frequency:
	//RCC_PLLConfig(RCC_PLLSource_HSE, 8, )
}

void Config_DecimationFilter()
{
	//Enable and reset CRC (Cyclic Redundancy check) --> For error detection
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_CRC, ENABLE);
	CRC_ResetDR();

	pHandler.in_ptr_channels = 1; //One input PDM stream
	pHandler.out_ptr_channels = 1; //One output PCM stream
	pHandler.bit_order = PDM_FILTER_BIT_ORDER_MSB;
	pHandler.high_pass_tap = 2147483647; //highest # of taps to ensure low frequency signals aren't attenuated
	pHandler.endianness = PDM_FILTER_ENDIANNESS_BE;
	//Set internal memory array
	PDM_Filter_Init(&pHandler);

	pConfig.decimation_factor = PDM_FILTER_DEC_FACTOR_64; //Based on desired output frequency of PCM waveform (16kHZ)
	pConfig.mic_gain = 12; //Amplify input signal by a factor of 12dB
	pConfig.output_samples_number = 16; //PCM samples per call to filter (1ms recording)
	PDM_Filter_setConfig(&pHandler, &pConfig);
}

void Config_I2SInterface()
{
	//Microphone CLK input (PB10)
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
	GPIO_InitTypeDef gpioInitStruct;
	gpioInitStruct.GPIO_Pin = GPIO_Pin_10;
	gpioInitStruct.GPIO_Mode = GPIO_Mode_AF; // I2S2_CK
	gpioInitStruct.GPIO_Speed = GPIO_Speed_50MHz; //Keep Slew Rate high since our output clock speed is of high frequency
	gpioInitStruct.GPIO_OType = GPIO_OType_PP; //Push-pull
	gpioInitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOB, &gpioInitStruct);
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource10, GPIO_AF_SPI2);

	//Microphone Serial Data Output (PC3)
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
	gpioInitStruct.GPIO_Pin = GPIO_Pin_3;
	gpioInitStruct.GPIO_Mode = GPIO_Mode_AF;
	gpioInitStruct.GPIO_OType = GPIO_OType_PP;
	gpioInitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
	gpioInitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOC, &gpioInitStruct);
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource3, GPIO_AF_SPI2);

	//I2S2 Interface Configuration (Ref: Page 900 in Reference Manual)
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2, ENABLE);
	I2S_InitTypeDef i2sInitStruct;
	i2sInitStruct.I2S_Mode= I2S_Mode_MasterRx;
	i2sInitStruct.I2S_DataFormat = I2S_DataFormat_16b;
	i2sInitStruct.I2S_Standard = I2S_Standard_MSB;
	i2sInitStruct.I2S_AudioFreq = I2S_AudioFreq_32k; //Bitrate = (32KHz sampling freq)*(16-bits/channel)*(2 channels) = 1,024,000 samples/s
	i2sInitStruct.I2S_MCLKOutput = I2S_MCLKOutput_Disable;
	i2sInitStruct.I2S_CPOL = I2S_CPOL_Low; //idle state of clock is low (i.e. active-high)
	I2S_Init(SPI2, &i2sInitStruct);
	I2S_Cmd(SPI2, ENABLE);
}

void Config_DMA()
{
	RCC_AHB1PeriphResetCmd(RCC_AHB1Periph_DMA1, ENABLE);
	DMA_InitTypeDef dma;
	dma.DMA_Channel = DMA_Channel_3;
	dma.DMA_DIR = DMA_DIR_PeripheralToMemory;
	dma.DMA_PeripheralBaseAddr = 0x40003800;
}

void DMA1_Stream3_IRQHandler()
{

}
