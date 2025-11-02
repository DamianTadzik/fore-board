///*
// * bno085.c
// *
// *  Created on: Nov 2, 2025
// *      Author: brzan
// */
//
//#include "bno085.h"
//#include "cmsis_os2.h"
//#include <string.h>
//
//
//extern volatile imu_debug_t imu_dbg;
//extern osSemaphoreId_t imuIntSem;
//
//static inline void bno_short_delay(void)
//{
//    for (volatile int i = 0; i < 16; i++) {
//        __NOP();   // 50 NOPs ≈ 5–8 µs on 72 MHz STM32F1
//    }
//}
//
//// forward
//static HAL_StatusTypeDef bno_spi_read(bno085_t *dev, uint8_t *buf, uint16_t len);
//static HAL_StatusTypeDef bno_spi_write(bno085_t *dev, const uint8_t *buf, uint16_t len);
//static void bno_cs_low(bno085_t *dev)
//{
//	HAL_GPIO_WritePin(dev->cs_port,  dev->cs_pin,  GPIO_PIN_RESET);
//	bno_short_delay();
//}
//static void bno_cs_high(bno085_t *dev)
//{
//	bno_short_delay();
//	HAL_GPIO_WritePin(dev->cs_port,  dev->cs_pin,  GPIO_PIN_SET);
//}
//
//static void bno_reset(bno085_t *dev)
//{
//    HAL_GPIO_WritePin(dev->rst_port, dev->rst_pin, GPIO_PIN_RESET);
//    osDelay(500);
//    HAL_GPIO_WritePin(dev->rst_port, dev->rst_pin, GPIO_PIN_SET);
////    osDelay(20); // BNO po resecie sam pociągnie INT w dół
//}
//
//static uint8_t next_seq(bno085_t *dev, uint8_t chan)
//{
//    uint8_t s = dev->seq[chan]++;
//    return s;
//}
//
//static HAL_StatusTypeDef wait_int_low(bno085_t *dev, uint32_t tout_ms)
//{
//    uint32_t t0 = osKernelGetTickCount();
//    while (HAL_GPIO_ReadPin(dev->int_port, dev->int_pin) == GPIO_PIN_SET) {
//        if ((osKernelGetTickCount() - t0) > tout_ms) return HAL_TIMEOUT;
//        osDelay(1);
//    }
//    return HAL_OK;
//}
//
//static HAL_StatusTypeDef wait_int_high(bno085_t *dev, uint32_t tout_ms)
//{
//    uint32_t t0 = osKernelGetTickCount();
//    while (HAL_GPIO_ReadPin(dev->int_port, dev->int_pin) == GPIO_PIN_RESET) {
//        if ((osKernelGetTickCount() - t0) > tout_ms) return HAL_TIMEOUT;
//        osDelay(1);
//    }
//    return HAL_OK;
//}
//
//
//// wysłanie SET FEATURE na dany raport
//static HAL_StatusTypeDef bno_enable_feature(bno085_t *dev, uint8_t report_id, uint32_t period_us)
//{
//    volatile uint8_t p[17] = {0};
//    p[0] = BNO_REPORT_SET_FEATURE;
//    p[1] = report_id;
//    p[2] = 0x01;  // flags
//    p[3] = 0; p[4] = 0; // change sensitivity
//    p[5] = (uint8_t)(period_us & 0xFF);
//    p[6] = (uint8_t)((period_us >> 8) & 0xFF);
//    p[7] = (uint8_t)((period_us >> 16) & 0xFF);
//    p[8] = (uint8_t)((period_us >> 24) & 0xFF);
//    // batch interval + sensor-spec = 0
//
//    uint16_t len = BNO_SHTP_HEADER_SIZE + sizeof(p);
//    dev->tx_buf[0] = len & 0xFF;
//    dev->tx_buf[1] = len >> 8;
//    dev->tx_buf[2] = BNO_CHAN_CONTROL;
//    dev->tx_buf[3] = next_seq(dev, BNO_CHAN_CONTROL);
//    memcpy(&dev->tx_buf[4], p, sizeof(p));
//
//    HAL_StatusTypeDef st = bno_spi_write(dev, dev->tx_buf, len);
//    return st;
//}
//
//static HAL_StatusTypeDef bno_cmd_initialize(bno085_t *dev)
//{
//    uint8_t p[12] = {0};
//    p[0] = 0xF2;      // SHTP_REPORT_COMMAND_REQUEST
//    p[1] = 0x00;      // seq – możesz inkrementować jak chcesz
//    p[2] = 0x04;      // SENSOR_COMMAND_INITIALIZE
//    p[3] = 0x01;      // P0 = subsystem 1 (motion)
//    // reszta = 0
//
//    uint16_t len = 4 + sizeof(p);
//    dev->tx_buf[0] = (uint8_t)(len & 0xFF);
//    dev->tx_buf[1] = (uint8_t)(len >> 8);
//    dev->tx_buf[2] = BNO_CHAN_CONTROL;           // 2
//    dev->tx_buf[3] = next_seq(dev, BNO_CHAN_CONTROL);
//    memcpy(&dev->tx_buf[4], p, sizeof(p));
//
//    return bno_spi_write(dev, dev->tx_buf, len);
//}
//
//
//
//HAL_StatusTypeDef bno085_init(bno085_t *dev)
//{
//    memset(dev->seq, 0, sizeof(dev->seq));
//
//    bno_reset(dev);
//
//    // 1) pierwszy INT (advertisement)
////    if (wait_int_low(dev, 1000) != HAL_OK) return HAL_TIMEOUT;
//    if (osSemaphoreAcquire(imuIntSem, osWaitForever) == osOK) {
//    	imu_dbg.sem_take++;
//    	bno085_handle_int(dev, NULL);   // wyrzuć
//    }
//
//    // 2) drugi INT (control advertisement)
////    if (wait_int_low(dev, 1000) != HAL_OK) return HAL_TIMEOUT;
//    if (osSemaphoreAcquire(imuIntSem, osWaitForever) == osOK) {
//    	imu_dbg.sem_take++;
//    	bno085_handle_int(dev, NULL);   // wyrzuć
//    }
////
//////     3) teraz BNO jest cicho → INT = HIGH → możemy NADAWAĆ
////    if (wait_int_high(dev, 1000) != HAL_OK) return HAL_TIMEOUT;
////
//////     4) wyślij Product ID Request
////    uint8_t p[4] = { BNO_REPORT_PRODUCT_ID_REQUEST, 0, 0, 0 };
////    uint16_t len = BNO_SHTP_HEADER_SIZE + sizeof(p);
////    dev->tx_buf[0] = (uint8_t)(len & 0xFF);
////    dev->tx_buf[1] = (uint8_t)(len >> 8);
////    dev->tx_buf[2] = BNO_CHAN_CONTROL;
////    dev->tx_buf[3] = next_seq(dev, BNO_CHAN_CONTROL);
////    memcpy(&dev->tx_buf[4], p, sizeof(p));
////    if (bno_spi_write(dev, dev->tx_buf, len) != HAL_OK) return HAL_ERROR;
////
//////     5) czekamy aż on nam odpowie Product ID
//////    if (wait_int_low(dev, 1000) != HAL_OK) return HAL_TIMEOUT;
////    if (osSemaphoreAcquire(imuIntSem, osWaitForever) == osOK) {
////    	imu_dbg.sem_take++;
////    	bno085_handle_int(dev, NULL);   // wyrzuć
////    }
//
////    6)
//    if (bno_cmd_initialize(dev) != HAL_OK) return HAL_ERROR;
//
//
//    // 7) dopiero teraz FEATURE’y
//    if (bno_enable_feature(dev, BNO_REPORT_ACCELEROMETER, 10000) != HAL_OK) return HAL_ERROR;
//	if (osSemaphoreAcquire(imuIntSem, osWaitForever) == osOK) {
//		imu_dbg.sem_take++;
//		bno085_handle_int(dev, NULL);   // wyrzuć
//	}
//
//    if (bno_enable_feature(dev, BNO_REPORT_GYROSCOPE, 10000) != HAL_OK) return HAL_ERROR;
//	if (osSemaphoreAcquire(imuIntSem, osWaitForever) == osOK) {
//		imu_dbg.sem_take++;
//		bno085_handle_int(dev, NULL);   // wyrzuć
//	}
//
////    if (bno_enable_feature(dev, BNO_REPORT_ROTATION_VECTOR, 10000) != HAL_OK) return HAL_ERROR;
//
//    return HAL_OK;
//}
//
//
//// parsowanie jednego raportu z kanału 3
//static void bno_parse_report(const uint8_t *p, uint16_t len, bno085_sample_t *out)
//{
//    uint8_t rid = p[0];
//
//    if (rid == 0xFB) {
//        // timebase reference (pierwszy w pakiecie) – ignorujemy
//        return;
//    }
//    switch (rid) {
//    case BNO_REPORT_ACCELEROMETER:
//        // p[1] seq, p[2] status, p[3] delay
//        out->accel[0] = (int16_t)((p[5] << 8) | p[4]);
//        out->accel[1] = (int16_t)((p[7] << 8) | p[6]);
//        out->accel[2] = (int16_t)((p[9] << 8) | p[8]);
//        out->valid_accel = 1;
//        break;
//    case BNO_REPORT_GYROSCOPE:
//        out->gyro[0] = (int16_t)((p[5] << 8) | p[4]);
//        out->gyro[1] = (int16_t)((p[7] << 8) | p[6]);
//        out->gyro[2] = (int16_t)((p[9] << 8) | p[8]);
//        out->valid_gyro = 1;
//        break;
//    case BNO_REPORT_ROTATION_VECTOR:
//        // p[4..11] to quat i, j, k, real – Q14 :contentReference[oaicite:5]{index=5}
//        out->quat[0] = (int16_t)((p[5] << 8) | p[4]);
//        out->quat[1] = (int16_t)((p[7] << 8) | p[6]);
//        out->quat[2] = (int16_t)((p[9] << 8) | p[8]);
//        out->quat[3] = (int16_t)((p[11] << 8) | p[10]);
//        out->valid_quat = 1;
//        break;
//    default:
//        break;
//    }
//}
//
//HAL_StatusTypeDef bno085_handle_int(bno085_t *dev, void (*cb)(const bno085_sample_t*))
//{
//    // wyczyść bufor odbiorczy
//    memset(dev->rx_buf, 0, BNO_MAX_PACKET_SIZE);
//
//    // odczytaj pełny pakiet z czujnika (header + payload)
//    if (bno_spi_read(dev, dev->rx_buf, BNO_MAX_PACKET_SIZE) != HAL_OK)
//        return HAL_ERROR;
//
//    uint16_t total_len = dev->rx_buf[0] | (dev->rx_buf[1] << 8);
//    imu_dbg.total_len = total_len;
//
//    if (total_len < BNO_SHTP_HEADER_SIZE)
//        return HAL_ERROR; // pusty lub niepoprawny pakiet
//
//    uint8_t chan = dev->rx_buf[2];
//
//    if (chan == BNO_CHAN_INPUT_REPORTS) {
//        // w jednym pakiecie mogą być 2 raporty (timebase + sensor)
//        bno085_sample_t s = {0};
//        uint16_t idx = BNO_SHTP_HEADER_SIZE;
//
//        while (idx < total_len) {
//            uint8_t rid = dev->rx_buf[idx];
//            // parsujemy po znanym formacie
//            bno_parse_report(&dev->rx_buf[idx], total_len - idx, &s);
//            idx += 20;  // heurystyka – każdy raport ~20B
//        }
//
//        if (cb)
//            cb(&s);
//    }
//
//    // inne kanały – olewamy
//    return HAL_OK;
//}
//
//
//// --- SPI low level ---
//
//static HAL_StatusTypeDef bno_spi_read(bno085_t *dev, uint8_t *out, uint16_t out_len)
//{
//    uint8_t tx_dummy[4] = { 0x00 };
//    uint8_t rx_header[4] = { 0x00 };
//
//    bno_cs_low(dev);
//    if (HAL_SPI_TransmitReceive(dev->hspi, tx_dummy, rx_header, 4, BNO_SPI_TIMEOUT_MS)
//    		!= HAL_OK)
//    {
//        bno_cs_high(dev);
//        return HAL_ERROR;
//    }
//
//    uint16_t total_len = rx_header[0] | (rx_header[1] << 8);
//    if (total_len < 4 || total_len > out_len) {
//        bno_cs_high(dev);
//        return HAL_ERROR;
//    }
//
//    uint16_t payload_len = total_len - 4;
//    for (uint16_t i = 0; i < payload_len; i++) {
//        if (HAL_SPI_TransmitReceive(dev->hspi, dev->tx_dummy_buf, &out[4 + i], 1, BNO_SPI_TIMEOUT_MS) != HAL_OK) {
//            bno_cs_high(dev);
//            return HAL_ERROR;
//        }
//    }
//
//    // skopiuj header
//    out[0] = rx_header[0];
//    out[1] = rx_header[1];
//    out[2] = rx_header[2];
//    out[3] = rx_header[3];
//
//    bno_cs_high(dev);
//    return HAL_OK;
//}
//
//
//static HAL_StatusTypeDef bno_spi_write(bno085_t *dev, const uint8_t *buf, uint16_t len)
//{
//    // 1. czekamy aż BNO będzie gotowy na DUPEX (INT=HIGH)
//    if (wait_int_high(dev, 1000) != HAL_OK)
//        return HAL_TIMEOUT;
//
//    bno_cs_low(dev);
//    HAL_StatusTypeDef st = HAL_SPI_TransmitReceive(dev->hspi,
//                                                   (uint8_t*)buf,
//                                                   dev->rx_buf,   // odbieramy śmieci
//                                                   len,
//                                                   BNO_SPI_TIMEOUT_MS);
//    bno_cs_high(dev);
//
//    return st;
//}
//
