/*
 * bno085.c
 *
 *  Created on: Nov 2, 2025
 *      Author: brzan
 */

#include "bno085.h"
#include "cmsis_os2.h"
#include <string.h>

// forward
static HAL_StatusTypeDef bno_spi_read(bno085_t *dev, uint8_t *buf, uint16_t len);
static HAL_StatusTypeDef bno_spi_write(bno085_t *dev, const uint8_t *buf, uint16_t len);
static void bno_cs_low(bno085_t *dev)  { HAL_GPIO_WritePin(dev->cs_port,  dev->cs_pin,  GPIO_PIN_RESET); }
static void bno_cs_high(bno085_t *dev) { HAL_GPIO_WritePin(dev->cs_port,  dev->cs_pin,  GPIO_PIN_SET);   }

static void bno_reset(bno085_t *dev)
{
    HAL_GPIO_WritePin(dev->rst_port, dev->rst_pin, GPIO_PIN_RESET);
    osDelay(2);
    HAL_GPIO_WritePin(dev->rst_port, dev->rst_pin, GPIO_PIN_SET);
    osDelay(20); // BNO po resecie sam pociągnie INT w dół
}

static uint8_t next_seq(bno085_t *dev, uint8_t chan)
{
    uint8_t s = dev->seq[chan]++;
    return s;
}

// wysłanie SET FEATURE na dany raport
static HAL_StatusTypeDef bno_enable_feature(bno085_t *dev, uint8_t report_id, uint32_t period_us)
{
    uint8_t p[17] = {0};
    p[0] = BNO_REPORT_SET_FEATURE;
    p[1] = report_id;
    p[2] = 0;  // flags
    p[3] = 0; p[4] = 0; // change sensitivity
    p[5] = (uint8_t)(period_us & 0xFF);
    p[6] = (uint8_t)((period_us >> 8) & 0xFF);
    p[7] = (uint8_t)((period_us >> 16) & 0xFF);
    p[8] = (uint8_t)((period_us >> 24) & 0xFF);
    // batch interval = 0
    // sensor-spec = 0

    // zbuduj SHTP
    uint16_t len = BNO_SHTP_HEADER_SIZE + sizeof(p);
    dev->tx_buf[0] = (uint8_t)(len & 0xFF);
    dev->tx_buf[1] = (uint8_t)(len >> 8);
    dev->tx_buf[2] = BNO_CHAN_CONTROL;
    dev->tx_buf[3] = next_seq(dev, BNO_CHAN_CONTROL);
    memcpy(&dev->tx_buf[4], p, sizeof(p));

    return bno_spi_write(dev, dev->tx_buf, len);
}

HAL_StatusTypeDef bno085_init(bno085_t *dev)
{
    memset(dev->seq, 0, sizeof(dev->seq));

    bno_reset(dev);

    // poczekaj aż INT spadnie
    uint32_t t0 = osKernelGetTickCount();
    while (HAL_GPIO_ReadPin(dev->int_port, dev->int_pin) == GPIO_PIN_SET) {
        if ((osKernelGetTickCount() - t0) > 200) return HAL_TIMEOUT;
        osDelay(1);
    }

    // odbierz pakiet reklamowy
    uint8_t dump;
    bno085_sample_t dummy;
    (void)dump;
    bno085_handle_int(dev, NULL); // tylko wyczyść

    // włącz 100 Hz
    if (bno_enable_feature(dev, BNO_REPORT_ACCELEROMETER, 10000) != HAL_OK) return HAL_ERROR;
    if (bno_enable_feature(dev, BNO_REPORT_GYROSCOPE,     10000) != HAL_OK) return HAL_ERROR;
    if (bno_enable_feature(dev, BNO_REPORT_ROTATION_VECTOR, 10000) != HAL_OK) return HAL_ERROR;

    return HAL_OK;
}

// parsowanie jednego raportu z kanału 3
static void bno_parse_report(const uint8_t *p, uint16_t len, bno085_sample_t *out)
{
    uint8_t rid = p[0];

    if (rid == 0xFB) {
        // timebase reference (pierwszy w pakiecie) – ignorujemy
        return;
    }
    switch (rid) {
    case BNO_REPORT_ACCELEROMETER:
        // p[1] seq, p[2] status, p[3] delay
        out->accel[0] = (int16_t)((p[5] << 8) | p[4]);
        out->accel[1] = (int16_t)((p[7] << 8) | p[6]);
        out->accel[2] = (int16_t)((p[9] << 8) | p[8]);
        out->valid_accel = 1;
        break;
    case BNO_REPORT_GYROSCOPE:
        out->gyro[0] = (int16_t)((p[5] << 8) | p[4]);
        out->gyro[1] = (int16_t)((p[7] << 8) | p[6]);
        out->gyro[2] = (int16_t)((p[9] << 8) | p[8]);
        out->valid_gyro = 1;
        break;
    case BNO_REPORT_ROTATION_VECTOR:
        // p[4..11] to quat i, j, k, real – Q14 :contentReference[oaicite:5]{index=5}
        out->quat[0] = (int16_t)((p[5] << 8) | p[4]);
        out->quat[1] = (int16_t)((p[7] << 8) | p[6]);
        out->quat[2] = (int16_t)((p[9] << 8) | p[8]);
        out->quat[3] = (int16_t)((p[11] << 8) | p[10]);
        out->valid_quat = 1;
        break;
    default:
        break;
    }
}

HAL_StatusTypeDef bno085_handle_int(bno085_t *dev, void (*cb)(const bno085_sample_t*))
{
    // 1. zczytaj header
    bno_cs_low(dev);
    uint8_t tx_dummy[BNO_MAX_PACKET_SIZE];
    memset(tx_dummy, 0xFF, sizeof(tx_dummy));

    if (HAL_SPI_TransmitReceive(dev->hspi, tx_dummy, dev->rx_buf,
                                BNO_SHTP_HEADER_SIZE, BNO_SPI_TIMEOUT_MS) != HAL_OK) {
        bno_cs_high(dev);
        return HAL_ERROR;
    }

    uint16_t total_len = dev->rx_buf[0] | (dev->rx_buf[1] << 8);
    if (total_len == 0) {
        bno_cs_high(dev);
        return HAL_OK;
    }
    // 2. doślij resztę
    if (HAL_SPI_TransmitReceive(dev->hspi, tx_dummy,
                                dev->rx_buf + BNO_SHTP_HEADER_SIZE,
                                total_len - BNO_SHTP_HEADER_SIZE,
                                BNO_SPI_TIMEOUT_MS) != HAL_OK) {
        bno_cs_high(dev);
        return HAL_ERROR;
    }
    bno_cs_high(dev);

    uint8_t chan = dev->rx_buf[2];

    if (chan == BNO_CHAN_INPUT_REPORTS) {
        // w jednym pakiecie mogą być 2 raporty (timebase + sensor)
        bno085_sample_t s = {0};
        uint16_t idx = BNO_SHTP_HEADER_SIZE;
        while (idx < total_len) {
            uint8_t rid = dev->rx_buf[idx];
            // długość pojedynczego input report nie jest wprost podana,
            // ale dla accel/gyro/rotvec z SH-2 to zawsze 11 lub 12 bajtów – parsujemy “po znanym formacie”.
            bno_parse_report(&dev->rx_buf[idx], total_len - idx, &s);
            // heurystyka: wszystkie te raporty są <= 20 B
            idx += 20;
        }
        if (cb) cb(&s);
    }
    // inne kanały – olewamy

    return HAL_OK;
}

// --- SPI low level ---

static HAL_StatusTypeDef bno_spi_read(bno085_t *dev, uint8_t *buf, uint16_t len)
{
    uint8_t dummy[300];
    memset(dummy, 0xFF, len);
    return HAL_SPI_TransmitReceive(dev->hspi, dummy, buf, len, BNO_SPI_TIMEOUT_MS);
}

static HAL_StatusTypeDef bno_spi_write(bno085_t *dev, const uint8_t *buf, uint16_t len)
{
    bno_cs_low(dev);
    HAL_StatusTypeDef st = HAL_SPI_Transmit(dev->hspi, (uint8_t *)buf, len, BNO_SPI_TIMEOUT_MS);
    bno_cs_high(dev);
    return st;
}
