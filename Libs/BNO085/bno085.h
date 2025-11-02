/*
 * bno085.h
 *
 *  Created on: Nov 2, 2025
 *      Author: brzan
 */

#ifndef BNO085_BNO085_H_
#define BNO085_BNO085_H_

//#include "stm32fxxx_hal.h"
#include "stm32f1xx_hal.h"
#include <stdint.h>
#include <stddef.h>

#define BNO_SPI_TIMEOUT_MS      5
#define BNO_MAX_PACKET_SIZE     300    // wystarczy na kilka raportów
#define BNO_SHTP_HEADER_SIZE    4

// kanały SHTP
#define BNO_CHAN_COMMAND        0
#define BNO_CHAN_EXECUTABLE     1
#define BNO_CHAN_CONTROL        2
#define BNO_CHAN_INPUT_REPORTS  3

// raporty SH-2
#define BNO_REPORT_ACCELEROMETER        0x01
#define BNO_REPORT_GYROSCOPE            0x02   // calibrated gyro input report :contentReference[oaicite:2]{index=2}
#define BNO_REPORT_ROTATION_VECTOR      0x05   // quaternion :contentReference[oaicite:3]{index=3}
#define BNO_REPORT_SET_FEATURE          0xFD   // host -> BNO :contentReference[oaicite:4]{index=4}
#define BNO_REPORT_PRODUCT_ID_REQUEST   0xF9

typedef struct {
    SPI_HandleTypeDef *hspi;
    GPIO_TypeDef *cs_port;
    uint16_t cs_pin;
    GPIO_TypeDef *rst_port;
    uint16_t rst_pin;
    GPIO_TypeDef *int_port;
    uint16_t int_pin;
    uint8_t seq[6];   // sekwencje per kanał
    uint8_t rx_buf[BNO_MAX_PACKET_SIZE];
    uint8_t tx_buf[BNO_MAX_PACKET_SIZE];
    uint8_t tx_dummy_buf[BNO_MAX_PACKET_SIZE];
} bno085_t;

typedef struct {
    uint32_t t_us;        // z timebase (opcjonalnie)
    int16_t accel[3];     // w jednostkach BNO (najczęściej 1/100 m/s^2)
    int16_t gyro[3];      // w jednostkach BNO (najczęściej 1/16 dps)
    int16_t quat[4];      // Q14 (w/w/x/y/z) – tu zostawiamy jako int16
    uint8_t valid_accel;
    uint8_t valid_gyro;
    uint8_t valid_quat;
} bno085_sample_t;

typedef struct {
    uint32_t exti_trig;     // zlicza każde wywołanie exti_task_imu_callback()
    uint32_t sem_give;      // tyle razy osSemaphoreRelease()
    uint32_t sem_take;      // tyle razy osSemaphoreAcquire() zwróciło OK
    uint32_t spi_read_ok;   // bno085_handle_int zakończone HAL_OK
    uint32_t spi_err;       // błędy HAL_ERROR
    uint32_t total_len;     // ostatnia długość pakietu
    uint32_t int_state;     // ostatni stan pinu INT
} imu_debug_t;

HAL_StatusTypeDef bno085_init(bno085_t *dev);
HAL_StatusTypeDef bno085_handle_int(bno085_t *dev, void (*cb)(const bno085_sample_t*));

#endif /* BNO085_BNO085_H_ */
