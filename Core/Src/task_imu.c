/*
 * task_imu.c
 *
 *  Created on: Nov 2, 2025
 *      Author: brzan
 */

#include "task_imu.h"
#include "main.h"
#include "cmsis_os2.h"
#include "fore_board.h"

#include "spi.h"
#include "BNO085/bno085.h"
//#include "can-not/can_not.h"
//#include "can-messages-mini-celka/src/cmmc.h"

// konfiguracja twoich pinów
#define BNO_CS_PORT   GPIOA
#define BNO_CS_PIN    GPIO_PIN_12
#define BNO_RST_PORT  GPIOA
#define BNO_RST_PIN   GPIO_PIN_11
#define BNO_INT_PORT  GPIOA
#define BNO_INT_PIN   GPIO_PIN_15

// Debug only
volatile imu_debug_t imu_dbg = {0};

// uchwyt do semafora z EXTI
extern osSemaphoreId_t imuIntSem;

// Exported
void exti_task_imu_callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == BNO_INT_PIN) {
        imu_dbg.exti_trig++;

        // Spi recieve 4 bytes
    }
}



extern SPI_HandleTypeDef hspi1;
static bno085_t bno;
static volatile bno085_sample_t bno_result;

static void imu_packet_ready(const bno085_sample_t *s)
{
    // tymczasowo
	bno_result = *s;
	UNUSED(s);
}


extern volatile uint32_t task_imu_alive;
void task_imu(void *argument)
{
	fore_board_t* fb_ptr = fore_board_get_ptr();
	(void)fb_ptr;

    bno.hspi     = &hspi1;
    bno.cs_port  = BNO_CS_PORT;
    bno.cs_pin   = BNO_CS_PIN;
    bno.rst_port = BNO_RST_PORT;
    bno.rst_pin  = BNO_RST_PIN;
    bno.int_port = BNO_INT_PORT;
    bno.int_pin  = BNO_INT_PIN;

    while (bno085_init(&bno) != HAL_OK) {
        // możesz tu zrobić retry albo error log
    	osDelay(8*1000);
    }

	while (1)
	{
		if (osSemaphoreAcquire(imuIntSem, osWaitForever) == osOK) {
		    imu_dbg.sem_take++;
//		    do {
		        HAL_StatusTypeDef st = bno085_handle_int(&bno, imu_packet_ready);
		        if (st == HAL_OK)
		        	imu_dbg.spi_read_ok++;
		        else
		        	imu_dbg.spi_err++;
		        imu_dbg.int_state = HAL_GPIO_ReadPin(BNO_INT_PORT, BNO_INT_PIN);
//		        osDelay(1);
//		    } while (HAL_GPIO_ReadPin(BNO_INT_PORT, BNO_INT_PIN) == GPIO_PIN_RESET);
		}
		task_imu_alive++;
	}
}
