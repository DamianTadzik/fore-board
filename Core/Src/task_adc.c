/*
 * task_adc.c
 *
 *  Created on: Sep 1, 2025
 *      Author: brzan
 */
#include "task_adc.h"
#include "main.h"
#include "cmsis_os2.h"
#include "fore_board.h"

#include "tim.h"
#include "adc.h"

typedef enum {
	channel_1 = 0,
	channel_4,
	channel_Temperature_Sensor,
	channel_Vrefint,

	ADC_NUMBER_OF_CHANNELS,
} ADC_channels_t;

#define ADC_N_SAMPLES 8
#define ADC_READY_FLAG 0x01

/* Array for storing ADC results */
static volatile uint16_t s_adc_dma_buf[ADC_NUMBER_OF_CHANNELS];
static volatile uint16_t s_samples[ADC_NUMBER_OF_CHANNELS][ADC_N_SAMPLES];
static uint8_t s_sample_idx = 0;

/* Voltage results calculated from ADC results */
static volatile float s_voltages[ADC_NUMBER_OF_CHANNELS];
/* Dynamically calculated VDD value */
static volatile float VDD = 0.0;
static float TEMPERATURE = 0.0;

///* Various numbers used for calculations */
static const uint16_t ADC_resolution = 4096 - 1;	// 12 bit
static const float VREFINT = 1.2f;

/* This task id holder */
static osThreadId_t s_thisTask = NULL;

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
    if (hadc->Instance == ADC1)
    {
        for (uint8_t ch = 0; ch < ADC_NUMBER_OF_CHANNELS; ch++)
        {
            s_samples[ch][s_sample_idx] = s_adc_dma_buf[ch];
        }
        s_sample_idx++;
        if (s_sample_idx >= ADC_N_SAMPLES)
        {
            s_sample_idx = 0;

            osThreadFlagsSet(s_thisTask, ADC_READY_FLAG);
        }
    }
}

extern volatile uint32_t task_adc_alive;
void task_adc(void *argument)
{
	fore_board_t* fb_ptr = fore_board_get_ptr();
	s_thisTask = osThreadGetId();

	/* Required on F1 (idk why but ehhh...) */
	ADC1->CR2 |= ADC_CR2_TSVREFE;
	osDelay(1);

	/* ADC calibration */
	HAL_StatusTypeDef status = 0;
	status = HAL_ADCEx_Calibration_Start(&hadc1);

	/* Start of the ADC and timer */
	status = HAL_ADC_Start_DMA(&hadc1, (uint32_t*)s_adc_dma_buf, ADC_NUMBER_OF_CHANNELS);
	status = HAL_TIM_Base_Start(&htim3);
	UNUSED(status);

	while (1)
	{
		/* Wait for flag from ISR */
		osThreadFlagsWait(ADC_READY_FLAG, osFlagsWaitAny, osWaitForever);

		/* process obtained reading (averaging) */
		uint32_t sum[ADC_NUMBER_OF_CHANNELS] = {0};
		for (uint8_t ch = 0; ch < ADC_NUMBER_OF_CHANNELS; ch++)
		{
		    for (uint8_t i = 0; i < ADC_N_SAMPLES; i++) sum[ch] += s_samples[ch][i];
		}
		uint16_t avg[ADC_NUMBER_OF_CHANNELS];
		for (uint8_t ch = 0; ch < ADC_NUMBER_OF_CHANNELS; ch++) avg[ch] = sum[ch] / ADC_N_SAMPLES;

		/* Calculate the physical voltages */
		VDD = VREFINT * ADC_resolution / avg[channel_Vrefint];
		for (uint8_t ch = 0; ch < ADC_NUMBER_OF_CHANNELS; ch++)
		{
			s_voltages[ch] = VDD * avg[ch] / ADC_resolution;
		}
		TEMPERATURE = 25.0f + (s_voltages[channel_Temperature_Sensor] - 1.43f) / 0.0043f;

		/* Save what necessary in the fore_board_t structure */
		fb_ptr->left_servo_feedback.voltage  = s_voltages[channel_1];
		fb_ptr->right_servo_feedback.voltage = s_voltages[channel_4];

		task_adc_alive++;
	}
}
