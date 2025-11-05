/*
 * MPU9250.h
 *
 *  Created on: Mar 13, 2022
 *      Author: MarkSherstan
 */

#ifndef INC_MPU9250_H_
#define INC_MPU9250_H_

// Libraries
#include <stdint.h>
#include <math.h>
#include "SPI.h"
#include "cmsis_os2.h"

// Constants
#define RAD2DEG 57.2957795131

// =========================================================
// MPU-9250 Register Map
// =========================================================

// Self-test
#define SELF_TEST_X_GYRO     0x00
#define SELF_TEST_Y_GYRO     0x01
#define SELF_TEST_Z_GYRO     0x02
#define SELF_TEST_X_ACCEL    0x0D
#define SELF_TEST_Y_ACCEL    0x0E
#define SELF_TEST_Z_ACCEL    0x0F

// Gyro offset
#define XG_OFFSET_H          0x13
#define XG_OFFSET_L          0x14
#define YG_OFFSET_H          0x15
#define YG_OFFSET_L          0x16
#define ZG_OFFSET_H          0x17
#define ZG_OFFSET_L          0x18

// Sample rate & config
#define SMPLRT_DIV           0x19
#define CONFIG               0x1A
#define GYRO_CONFIG          0x1B
#define ACCEL_CONFIG         0x1C
#define ACCEL_CONFIG2        0x1D
#define LP_ACCEL_ODR         0x1E
#define WOM_THR              0x1F

// FIFO & I2C Master
#define FIFO_EN              0x23
#define I2C_MST_CTRL         0x24
#define I2C_SLV0_ADDR        0x25
#define I2C_SLV0_REG         0x26
#define I2C_SLV0_CTRL        0x27
#define I2C_SLV1_ADDR        0x28
#define I2C_SLV1_REG         0x29
#define I2C_SLV1_CTRL        0x2A
#define I2C_SLV2_ADDR        0x2B
#define I2C_SLV2_REG         0x2C
#define I2C_SLV2_CTRL        0x2D
#define I2C_SLV3_ADDR        0x2E
#define I2C_SLV3_REG         0x2F
#define I2C_SLV3_CTRL        0x30
#define I2C_SLV4_ADDR        0x31
#define I2C_SLV4_REG         0x32
#define I2C_SLV4_DO          0x33
#define I2C_SLV4_CTRL        0x34
#define I2C_SLV4_DI          0x35
#define I2C_MST_STATUS       0x36

// Interrupts
#define INT_PIN_CFG          0x37
#define INT_ENABLE           0x38
#define INT_STATUS           0x3A

// Accelerometer readings
#define ACCEL_XOUT_H         0x3B
#define ACCEL_XOUT_L         0x3C
#define ACCEL_YOUT_H         0x3D
#define ACCEL_YOUT_L         0x3E
#define ACCEL_ZOUT_H         0x3F
#define ACCEL_ZOUT_L         0x40

// Temperature
#define TEMP_OUT_H           0x41
#define TEMP_OUT_L           0x42

// Gyroscope readings
#define GYRO_XOUT_H          0x43
#define GYRO_XOUT_L          0x44
#define GYRO_YOUT_H          0x45
#define GYRO_YOUT_L          0x46
#define GYRO_ZOUT_H          0x47
#define GYRO_ZOUT_L          0x48

// External sensor data
#define EXT_SENS_DATA_00     0x49
#define EXT_SENS_DATA_01     0x4A
#define EXT_SENS_DATA_02     0x4B
#define EXT_SENS_DATA_03     0x4C
#define EXT_SENS_DATA_04     0x4D
#define EXT_SENS_DATA_05     0x4E
#define EXT_SENS_DATA_06     0x4F
#define EXT_SENS_DATA_07     0x50
#define EXT_SENS_DATA_08     0x51
#define EXT_SENS_DATA_09     0x52
#define EXT_SENS_DATA_10     0x53
#define EXT_SENS_DATA_11     0x54
#define EXT_SENS_DATA_12     0x55
#define EXT_SENS_DATA_13     0x56
#define EXT_SENS_DATA_14     0x57
#define EXT_SENS_DATA_15     0x58
#define EXT_SENS_DATA_16     0x59
#define EXT_SENS_DATA_17     0x5A
#define EXT_SENS_DATA_18     0x5B
#define EXT_SENS_DATA_19     0x5C
#define EXT_SENS_DATA_20     0x5D
#define EXT_SENS_DATA_21     0x5E
#define EXT_SENS_DATA_22     0x5F
#define EXT_SENS_DATA_23     0x60

// I2C master delay & reset
#define I2C_SLV0_DO          0x63
#define I2C_SLV1_DO          0x64
#define I2C_SLV2_DO          0x65
#define I2C_SLV3_DO          0x66
#define I2C_MST_DELAY_CTRL   0x67
#define SIGNAL_PATH_RESET    0x68
#define MOT_DETECT_CTRL      0x69
#define USER_CTRL            0x6A
#define PWR_MGMT_1           0x6B
#define PWR_MGMT_2           0x6C

// FIFO and identity
#define FIFO_COUNTH          0x72
#define FIFO_COUNTL          0x73
#define FIFO_R_W             0x74
#define WHO_AM_I             0x75   // expected value: 0x71

// Accelerometer offsets
#define XA_OFFSET_H          0x77
#define XA_OFFSET_L          0x78
#define YA_OFFSET_H          0x7A
#define YA_OFFSET_L          0x7B
#define ZA_OFFSET_H          0x7D
#define ZA_OFFSET_L          0x7E

// =========================================================
// Magnetometer (AK8963) Register Map
// =========================================================
#define AK8963_WHO_AM_I      0x00  // expected 0x48
#define AK8963_INFO          0x01
#define AK8963_ST1           0x02
#define AK8963_HXL           0x03
#define AK8963_HXH           0x04
#define AK8963_HYL           0x05
#define AK8963_HYH           0x06
#define AK8963_HZL           0x07
#define AK8963_HZH           0x08
#define AK8963_ST2           0x09
#define AK8963_CNTL1         0x0A
#define AK8963_CNTL2         0x0B
#define AK8963_ASTC          0x0C
#define AK8963_TS1           0x0D
#define AK8963_TS2           0x0E
#define AK8963_I2CDIS        0x0F
#define AK8963_ASAX          0x10
#define AK8963_ASAY          0x11
#define AK8963_ASAZ          0x12

// WHO_AM_I answers
#define WHO_AM_I_9250_ANS    0x71
#define WHO_AM_I_AK8963_ANS  0x48

#define READWRITE         0x80
#define CS_SELECT         0
#define CS_DESELECT       1
#define SPI_TIMOUT_MS     1000

// Full scale ranges
enum gyroscopeFullScaleRange
{
    GFSR_250DPS,
    GFSR_500DPS,
    GFSR_1000DPS,
    GFSR_2000DPS
};

enum accelerometerFullScaleRange
{
    AFSR_2G,
    AFSR_4G,
    AFSR_8G,
    AFSR_16G
};

// Master structure
typedef struct MPU9250
{
    struct RawData
    {
        int16_t ax, ay, az, temp, gx, gy, gz;
    } rawData;

//    struct SensorData
//    {
//        float aScaleFactor, tempScaleFactor, gScaleFactor;
//        float ax, ay, az, temp, gx, gy, gz;
//    } sensorData;

    struct GyroOffs
    {
        int16_t x, y, z;
    } gyroOffs;
    struct AccOffs
    {
        int16_t x, y, z;
    } accOffs;

//    struct Attitude
//    {
//        float tau, dt;
//        float r, p, y;
//    } attitude;

    struct Settings
    {
    	GPIO_TypeDef *CS_PORT;
        uint16_t CS_PIN;
        GPIO_TypeDef *INT_PORT;
        uint16_t INT_PIN;
        osThreadId_t imuThreadId;
    } settings;
} MPU9250_t;

// Functions
uint8_t MPU_begin(SPI_HandleTypeDef *SPIx, MPU9250_t *mpuStruct);
void MPU_REG_READ(SPI_HandleTypeDef *SPIx, MPU9250_t *pMPU9250, uint8_t addr, uint8_t *pRxData, uint16_t RxSize);
void MPU_REG_WRITE(SPI_HandleTypeDef *SPIx, MPU9250_t *pMPU9250, uint8_t *pAddr, uint8_t *pVal);
//void MPU_writeGyroFullScaleRange(SPI_HandleTypeDef *SPIx,  MPU9250_t *pMPU9250, uint8_t gScale);
//void MPU_writeAccFullScaleRange(SPI_HandleTypeDef *SPIx, MPU9250_t *pMPU9250, uint8_t aScale);
void MPU_calibrateGyro(SPI_HandleTypeDef *SPIx, MPU9250_t *pMPU9250, uint16_t numCalPoints);
int16_t MPU_calibrateAcc(SPI_HandleTypeDef *SPIx, MPU9250_t *pMPU9250, uint16_t numCalPoints, int16_t *pDataSrc);
//void MPU_readProcessedData(SPI_HandleTypeDef *SPIx, MPU9250_t *pMPU9250);
//void MPU_calcAttitude(SPI_HandleTypeDef *SPIx, MPU9250_t *pMPU9250);
void MPU_readRawData(SPI_HandleTypeDef *SPIx, MPU9250_t *pMPU9250);
void MPU_CS(MPU9250_t *pMPU9250, uint8_t state);

#endif /* INC_MPU9250_H_ */
