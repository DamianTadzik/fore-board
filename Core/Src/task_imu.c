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
//#include "BNO085/bno085.h"
#include <string.h>

//#include "can-not/can_not.h"
//#include "can-messages-mini-celka/src/cmmc.h"

// --- piny ---
#define BNO_RST_PORT  GPIOA
#define BNO_RST_PIN	  GPIO_PIN_11
#define BNO_CS_PORT   GPIOA
#define BNO_CS_PIN    GPIO_PIN_12
#define BNO_INT_PORT  GPIOA
#define BNO_INT_PIN   GPIO_PIN_15

// --- konfiguracja ---
#define BNO_SPI_MAX_LEN   300
#define BNO_TASK_FLAG_RX_DONE  (1 << 0)
#define BNO_TASK_FLAG_RX_ERR   (1 << 1)

typedef enum {
    BNO_STATE_IDLE = 0,
    BNO_STATE_HEADER_RX,
    BNO_STATE_PAYLOAD_RX
} bno_spi_state_t;

typedef enum {
    BNO_CHAN_COMMAND        = 0,  // Komendy do bootloadera lub systemowe
    BNO_CHAN_EXECUTABLE     = 1,  // Firmware download / execution
    BNO_CHAN_CONTROL        = 2,  // Komunikacja kontrolna (Feature Reports, Product ID, itd.)
    BNO_CHAN_INPUT_REPORTS  = 3,  // Dane z sensorów (accelerometer, gyro, rotation vector)
    BNO_CHAN_WAKE_REPORTS   = 4,  // Jak kanał 3, ale z wybudzaniem hosta
    BNO_CHAN_GYRO           = 5,  // Zarezerwowany / debug (w niektórych firmware’ach)
    BNO_CHAN_DEBUG          = 14, // Kanał debugowy (opcjonalny)
} bno_shtp_channel_t;


typedef struct {
	uint32_t exti_callbacks;
	uint32_t exti_callbacks_unhandled;
	uint32_t exti_callbacks_non_idle;

	uint32_t task_rx_done;
	uint32_t task_rx_err;
	uint32_t task_timeout_err;

	uint32_t packets_sent;
} bno_debug_t;

typedef struct {
	uint8_t handle_interrupts;

    SPI_HandleTypeDef *hspi;
    volatile bno_spi_state_t state;
    uint16_t total_len;
    uint16_t payload_len;
    bno_shtp_channel_t channel;
    osThreadId_t task_id;

    uint8_t rx_buf[BNO_SPI_MAX_LEN];
    uint8_t tx_dummy[BNO_SPI_MAX_LEN];
    uint8_t rx_dummy[BNO_SPI_MAX_LEN];
} bno_spi_ctx_t;

static bno_spi_ctx_t bno_spi = {0};
volatile bno_debug_t bno_dbg = {0};

// --- forward declarations ---
static void bno_rst_low(void)  { HAL_GPIO_WritePin(BNO_RST_PORT, BNO_RST_PIN, GPIO_PIN_RESET); }
static void bno_rst_high(void) { HAL_GPIO_WritePin(BNO_RST_PORT, BNO_RST_PIN, GPIO_PIN_SET); }
static void bno_cs_low(void)  { HAL_GPIO_WritePin(BNO_CS_PORT, BNO_CS_PIN, GPIO_PIN_RESET); }
static void bno_cs_high(void) { HAL_GPIO_WritePin(BNO_CS_PORT, BNO_CS_PIN, GPIO_PIN_SET); }

// --- EXTI z pinu INT ---
void exti_task_imu_callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == BNO_INT_PIN && bno_spi.state == BNO_STATE_IDLE && bno_spi.handle_interrupts == 1)
    {
        bno_cs_low();

        bno_spi.state = BNO_STATE_HEADER_RX;
        bno_dbg.exti_callbacks++;

        memset(bno_spi.rx_buf, 0x00, 300);
        HAL_SPI_TransmitReceive_IT(bno_spi.hspi, bno_spi.tx_dummy, bno_spi.rx_buf, 4);
    }
    else if (GPIO_Pin == BNO_INT_PIN && bno_spi.state == BNO_STATE_IDLE && bno_spi.handle_interrupts == 0)
    {
    	bno_dbg.exti_callbacks_unhandled++;
    }
    else if (GPIO_Pin == BNO_INT_PIN && bno_spi.state != BNO_STATE_IDLE)
    {
    	bno_dbg.exti_callbacks_non_idle++;
    }
}

// --- callback po zakończeniu transmisji ---
void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
    if (hspi != bno_spi.hspi) return;

    switch (bno_spi.state)
    {
        case BNO_STATE_HEADER_RX:
        {
            bno_spi.total_len = (uint16_t)(bno_spi.rx_buf[0] | (bno_spi.rx_buf[1] << 8));
            if (bno_spi.total_len < 4 || bno_spi.total_len > BNO_SPI_MAX_LEN) {
                bno_spi.state = BNO_STATE_IDLE;
                bno_cs_high();
                osThreadFlagsSet(bno_spi.task_id, BNO_TASK_FLAG_RX_ERR);
                return;
            }

            bno_spi.channel = bno_spi.rx_buf[2];
            bno_spi.payload_len = bno_spi.total_len - 4;
            if (bno_spi.payload_len == 0) {
                // tylko header
                bno_spi.state = BNO_STATE_IDLE;
                bno_cs_high();
                osThreadFlagsSet(bno_spi.task_id, BNO_TASK_FLAG_RX_DONE);
            } else {
                bno_spi.state = BNO_STATE_PAYLOAD_RX;
//                memset(bno_spi.rx_buf+4, 0x00, bno_spi.payload_len);
                HAL_SPI_TransmitReceive_IT(bno_spi.hspi,
                                           bno_spi.tx_dummy,
                                           &bno_spi.rx_buf[4],
                                           bno_spi.payload_len);
            }
            break;
        }

        case BNO_STATE_PAYLOAD_RX:
        {
            bno_spi.state = BNO_STATE_IDLE;
            bno_cs_high();
            osThreadFlagsSet(bno_spi.task_id, BNO_TASK_FLAG_RX_DONE);
            break;
        }

        default:
            bno_spi.state = BNO_STATE_IDLE;
            bno_cs_high();
            osThreadFlagsSet(bno_spi.task_id, BNO_TASK_FLAG_RX_ERR);
            break;
    }
}

// TEMPORARY TEST FUNCTION XD
void SPI_Transmit(uint8_t tx_data) {
  uint8_t rx_data;  // receive buffer
  HAL_SPI_TransmitReceive(&hspi1, &tx_data, &rx_data, 1, HAL_MAX_DELAY);
}

// --- inicjalizacja kontekstu ---
static void bno_spi_init(SPI_HandleTypeDef *hspi)
{
    bno_spi.hspi = hspi;
    bno_spi.task_id = osThreadGetId();
    bno_spi.state = BNO_STATE_IDLE;
    memset(bno_spi.tx_dummy, 0, sizeof(bno_spi.tx_dummy));
    memset(bno_spi.rx_dummy, 0, sizeof(bno_spi.rx_dummy));
}

// --- wysyłka ---
static HAL_StatusTypeDef bno_spi_send(uint8_t *tx_buf, uint16_t tx_len)
{
    // poczekaj aż INT = HIGH (BNO gotowy na nowy transfer)
    uint32_t t0 = osKernelGetTickCount();
    while (HAL_GPIO_ReadPin(BNO_INT_PORT, BNO_INT_PIN) == GPIO_PIN_RESET) {
        if ((osKernelGetTickCount() - t0) > 1000) return HAL_TIMEOUT;
        osDelay(2);
    }

//    // dodatkowy mały delay ~50 µs żeby upewnić się, że linie SPI się ustabilizowały
//    for (volatile int i = 0; i < 7*20; i++) __NOP();  // ok. 50 µs przy 72 MHz

    // sprawdź, czy maszyna stanów jest wolna
    if (bno_spi.state != BNO_STATE_IDLE)
        return HAL_BUSY;

    bno_cs_low();
//    for (volatile int i = 0; i < 7*20; i++) __NOP();  // N/7 ~= N us

    memset(bno_spi.rx_dummy, 0, sizeof(bno_spi.rx_dummy));
    HAL_StatusTypeDef st = HAL_SPI_TransmitReceive(bno_spi.hspi,
                                                   tx_buf,
                                                   bno_spi.rx_dummy,
                                                   tx_len,
                                                   100);

    bno_cs_high();
    bno_dbg.packets_sent++;
//    for (volatile int i = 0; i < 100; i++) __NOP();  // po CS high mały odstęp
    return st;
}


extern volatile uint32_t task_imu_alive;
void task_imu(void *argument)
{
	fore_board_t* fb_ptr = fore_board_get_ptr();
	(void)fb_ptr;

	bno_rst_low();
	bno_spi_init(&hspi1);
	osDelay(400);
	bno_rst_high();
	osDelay(200);
	// Time between rst high and first interrupt from the device is above 90 ms
	// If Host does not handle interrupts the device tries to repeat, hence the
	// exti_callbacks_unhandled rises by a few.
	bno_spi.handle_interrupts = 1;
	// Then when we decide to handle the incoming interrupt the device will send three packets
//
//	{
//		uint8_t start_exec[] = {
//		    0x08, 0x00,    // total length = 8
//		    0x00,          // channel 0 (COMMAND)
//		    0x00,          // sequence 0
//		    0xF2, 0x00, 0x00, 0x00
//		};
//		bno_spi_send(start_exec, sizeof(start_exec));
//	}

	/* Odbiór pierwszych trzech pakietów
	 * 1. BNO_CHAN_COMMAND z total_len 276
	 * 2. BNO_CHAN_CONTROL z total_len 20
	 * 3. BNO_CHAN_EXECUTABLE z total_len 5
	 */
	for (int i = 0; i < 3; i++)
	{
        uint32_t flags = osThreadFlagsWait(BNO_TASK_FLAG_RX_DONE | BNO_TASK_FLAG_RX_ERR,
                                           osFlagsWaitAny,
                                           osWaitForever);
        if (flags == BNO_TASK_FLAG_RX_DONE)
        {
        	bno_dbg.task_rx_done++;
            // Tu masz gotowy pakiet w bno_spi.rx_buf o długości bno_spi.total_len
            // Możesz go sparsować lub przekazać dalej
        }
        else if (flags == BNO_TASK_FLAG_RX_ERR)
        {
        	bno_dbg.task_rx_err++;
            // Obsługa błędu
        }
        else
        {
        	bno_dbg.task_timeout_err++;
        }
	}


//	{
//		volatile int8_t xd = HAL_GPIO_ReadPin(BNO_INT_PORT, BNO_INT_PIN);
//		volatile int8_t xdxd = HAL_GPIO_ReadPin(BNO_CS_PORT, BNO_CS_PIN);
//
//		bno_cs_low();
//
//		xd = HAL_GPIO_ReadPin(BNO_INT_PORT, BNO_INT_PIN);
//		xdxd = HAL_GPIO_ReadPin(BNO_CS_PORT, BNO_CS_PIN);
//
////		uint8_t product_id_req[6] = {
////		    0x06, 0x00,    // total length = 8
//////			BNO_CHAN_COMMAND, //0
////			BNO_CHAN_CONTROL, //2
//////			BNO_CHAN_EXECUTABLE,
////		    0x00,          // sequence (może być 0)
////		    0xF9, 0x00,  // payload: Product ID Request
////		};
////	    HAL_StatusTypeDef st = HAL_SPI_TransmitReceive(bno_spi.hspi,
////	    											   product_id_req,
////	                                                   bno_spi.rx_dummy,
////	                                                   6,
////	                                                   100);
//		SPI_Transmit(0x06);
//		SPI_Transmit(0x00);
//		SPI_Transmit(0x02);
//		SPI_Transmit(0x00);
//		SPI_Transmit(0xF9);
//		SPI_Transmit(0x00);
//
//		bno_cs_high();
//
//	    xd = HAL_GPIO_ReadPin(BNO_INT_PORT, BNO_INT_PIN);
////		HAL_StatusTypeDef st = bno_spi_send(product_id_req, sizeof(product_id_req));
////		if (st != HAL_OK) {
////		    osDelay(1);
////		}
//	}


//	{
//		uint8_t enable_rotation_vector[] = {
//		    0x11, 0x00,        // total_len = 17 B (4 B header + 13 B payload)
//		    0x02,              // channel CONTROL
//		    0x01,              // sequence (kolejny po Product ID)
//		    0xFD,              // report ID = Set Feature Command
//		    0x05,              // Feature Report ID = Rotation Vector
//		    0x00,              // reserved
//		    0x00,              // Feature Flags
//		    0x00, 0x00, 0x00, 0x00,  // Change Sensitivity = 0
//		    0x0A, 0x00,        // Report Interval = 10 ms (100 Hz)
//		    0x00, 0x00,        // Batch Interval = 0
//		    0x00, 0x00         // Sensor-Specific Config = 0
//		};
//		bno_spi_send(enable_rotation_vector, sizeof(enable_rotation_vector));
//	}

	while (1)
	{
        uint32_t flags = osThreadFlagsWait(BNO_TASK_FLAG_RX_DONE | BNO_TASK_FLAG_RX_ERR,
                                           osFlagsWaitAny,
                                           osWaitForever);
        if (flags == BNO_TASK_FLAG_RX_DONE)
        {
        	bno_dbg.task_rx_done++;
            // Tu masz gotowy pakiet w bno_spi.rx_buf o długości bno_spi.total_len
            // Możesz go sparsować lub przekazać dalej
        }
        else if (flags == BNO_TASK_FLAG_RX_ERR)
        {
        	bno_dbg.task_rx_err++;
            // Obsługa błędu
        }
        else
        {
        	bno_dbg.task_timeout_err++;
        }


		task_imu_alive++;
	}
}
