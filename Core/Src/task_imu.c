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
#include "MPU9250/MPU9250.h"
#include "can-not/can_not.h"
#include "can-messages-mini-celka/src/cmmc.h"

#define IMU_INTERRUPT_FLAG (0x01 << 0)

MPU9250_t MPU9250;

void exti_task_imu_callback(uint16_t GPIO_Pin)
{
	if (GPIO_Pin == MPU9250.settings.INT_PIN)
	{
		// Start the SPI_IT transaction TODO
		osThreadFlagsSet(MPU9250.settings.imuThreadId, IMU_INTERRUPT_FLAG);
	}
}

//static void exti_spi_cplt_callback(void)
//{
//
//	osThreadFlagsSet(MPU9250.settings.imuThreadId, IMU_INTERRUPT_FLAG);
//}

extern volatile uint32_t task_imu_alive;
void task_imu(void *argument)
{
	fore_board_t* fb_ptr = fore_board_get_ptr();
	(void)fb_ptr;

	// Configuration settings
	MPU9250.settings.CS_PIN = SPI1_CS_IMU_Pin;
	MPU9250.settings.CS_PORT = SPI1_CS_IMU_GPIO_Port;
	MPU9250.settings.INT_PIN = EXTI_IMU_Pin;
	MPU9250.settings.INT_PORT = EXTI_IMU_GPIO_Port;
	MPU9250.settings.imuThreadId = osThreadGetId();

	// Enough time to boot
	osDelay(50);
    uint8_t addr, val;

    // -- PWR_MGMT_1 (0x6b) : We write 0x80 in order to reset the config
    addr = PWR_MGMT_1;
    val  = 0x80;
    MPU_REG_WRITE(&hspi1, &MPU9250, &addr, &val);

    // Enough time to reboot
    osDelay(50);

	// Check if IMU configured properly and block if it didn't enter SPI mode
	// (it sets the PWR_MGMT_1 to 0x00, reads WHO_AM_I and sets the USER_CTRL to 0x10)
	if (MPU_begin(&hspi1, &MPU9250) != 1)
	{
		while (1){ osDelay(1000); }
	}

	// >>> INSERT SELF-TEST HERE <<<


    // --- PWR_MGMT_2 (0x6C) : enable all gyro and accel axes
    // Bit[5:3] disable gyro XYZ, Bit[2:0] disable accel XYZ
    // 0x00 = all sensors ON (default after reset)
    addr = PWR_MGMT_2;
    val  = 0x00;
    MPU_REG_WRITE(&hspi1, &MPU9250, &addr, &val);

    // --- SMPLRT_DIV (0x19) : set output data rate to 100 Hz
    // SampleRate = GyroOutputRate / (1 + SMPLRT_DIV)
    // With DLPF enabled, GyroOutputRate = 1 kHz
    // → 1 kHz / (1 + 9) = 100 Hz new samples
    // This is the rate at which the data-ready interrupt will fire.
    addr = SMPLRT_DIV;
    val  = 0x09;
    MPU_REG_WRITE(&hspi1, &MPU9250, &addr, &val);

    // --- CONFIG (0x1A) : gyro DLPF bandwidth + FSYNC disable
    // Bits[2:0] DLPF_CFG = 1 → 184 Hz bandwidth, ≈2 ms delay
    // Bits[5:3] EXT_SYNC_SET = 0 → FSYNC input disabled
    // Bit[6] FIFO_MODE = 0 → normal FIFO (overwrite on overflow)
    // This gives a very fast response with some high-freq noise filtering.
    addr = CONFIG;
    val  = 0x01;
    MPU_REG_WRITE(&hspi1, &MPU9250, &addr, &val);

    // --- GYRO_CONFIG (0x1B) : full-scale range ±2000 °/s
    // Bits[4:3] = 11 → 2000 °/s  (LSB sensitivity = 16.4 LSB/°/s)
    // Bit[7] XG_ST, Bit[6] YG_ST, Bit[5] ZG_ST = 0 → self-test off
    // Bits[1:0] FCHOICE_B = 00 → DLPF enabled (we use CONFIG’s DLPF)
    addr = GYRO_CONFIG;
    val  = 0x18;
    MPU_REG_WRITE(&hspi1, &MPU9250, &addr, &val);

    // --- ACCEL_CONFIG (0x1C) : full-scale range ±2 g
    // Bits[4:3] = 00 → 2 g  (LSB sensitivity = 16384 LSB/g)
    // Self-test bits [7:5] = 0, so normal operation.
    addr = ACCEL_CONFIG;
    val  = 0x00;
    MPU_REG_WRITE(&hspi1, &MPU9250, &addr, &val);

    // --- ACCEL_CONFIG2 (0x1D) : accel DLPF bandwidth
    // Bits[2:0] A_DLPF_CFG = 1 → 218 Hz bandwidth, ≈1.9 ms delay
    // Bit[3] ACCEL_FCHOICE_B = 0 → use the DLPF (not bypass)
    // Keeps latency low while filtering high-frequency vibration noise.
    addr = ACCEL_CONFIG2;
    val  = 0x01;
    MPU_REG_WRITE(&hspi1, &MPU9250, &addr, &val);

    // --- INT_PIN_CFG (0x37) : configure interrupt pin behavior
    // Bit7 ACTL = 1 → interrupt is active-low  (falling edge for EXTI)
    // Bit6 OPEN = 0 → push-pull driver (OK without external pull-ups)
    // Bit5 LATCH_INT_EN = 0 → interrupt is a short pulse, not latched
    // Bit4 INT_ANYRD_2CLEAR = 0 → pulse automatically clears (no read needed)
    // Bit3 ACTL_FSYNC = 0 → FSYNC is active-high (but FSYNC disabled anyway)
    // Bit2 FSYNC_INT_MODE_EN = 0 → disable FSYNC as interrupt source
    // → 0b10010000 = 0x90  but we only need ACTL=1, others 0 → 0x80? Wait explanation.
    addr = INT_PIN_CFG;
    val  = 0x10;  // 0b00010000: active-low (ACTL=1) + push-pull + pulsed
    MPU_REG_WRITE(&hspi1, &MPU9250, &addr, &val);

    // --- INT_ENABLE (0x38) : enable “data-ready” interrupt
    // Bit0 DATA_RDY_EN = 1 → generates interrupt each time new data is ready
    // All other bits 0 (no motion detect, FIFO overflow, etc.)
    addr = INT_ENABLE;
    val  = 0x01;
    MPU_REG_WRITE(&hspi1, &MPU9250, &addr, &val);


    // Gyro offset measurement (enter with debugger)
    volatile uint8_t calibrate_gyro_offset = 0;
    if (calibrate_gyro_offset)
    {
        MPU9250.gyroOffs.x = 0;
        MPU9250.gyroOffs.y = 0;
        MPU9250.gyroOffs.z = 0;
        MPU_calibrateGyro(&hspi1, &MPU9250, 3000);
    }
    else
    {
        MPU9250.gyroOffs.x = -36; // -36 -37
        MPU9250.gyroOffs.y = 22;  // 24 20
        MPU9250.gyroOffs.z = -11; // -11 -11

        // ---------------------------------------------------------------------
        // The datasheet defines that the user gyro offset registers (XG/YG/ZG_OFFSET)
        // are scaled internally by:
        //      OffsetLSB = X_OFFS_USR * 4 / 2^FS_SEL
        // For FS_SEL = 3 (±2000 °/s full-scale) → multiply by 2.
        // ---------------------------------------------------------------------

        volatile uint8_t write_gyro_offset_to_registers = 1;
        if (write_gyro_offset_to_registers)
        {
        	uint8_t data;

        	int16_t offsx = -(MPU9250.gyroOffs.x * 2);
        	int16_t offsy = -(MPU9250.gyroOffs.y * 2);
        	int16_t offsz = -(MPU9250.gyroOffs.z * 2);

			// X axis
			addr = XG_OFFSET_H;
			data = (uint8_t)(offsx >> 8);
			MPU_REG_WRITE(&hspi1, &MPU9250, &addr, &data);
			addr = XG_OFFSET_L;
			data = (uint8_t)(offsx & 0xFF);
			MPU_REG_WRITE(&hspi1, &MPU9250, &addr, &data);

			// Y axis
			addr = YG_OFFSET_H;
			data = (uint8_t)(offsy >> 8);
			MPU_REG_WRITE(&hspi1, &MPU9250, &addr, &data);
			addr = YG_OFFSET_L;
			data = (uint8_t)(offsy & 0xFF);
			MPU_REG_WRITE(&hspi1, &MPU9250, &addr, &data);

			// Z axis
			addr = ZG_OFFSET_H;
			data = (uint8_t)(offsz >> 8);
			MPU_REG_WRITE(&hspi1, &MPU9250, &addr, &data);
			addr = ZG_OFFSET_L;
			data = (uint8_t)(offsz & 0xFF);
			MPU_REG_WRITE(&hspi1, &MPU9250, &addr, &data);

			// Do not compensate in the software then
			MPU9250.gyroOffs.x = 0;
			MPU9250.gyroOffs.y = 0;
			MPU9250.gyroOffs.z = 0;
        }
    }

    // Acc offset measurement (enter with debugger)
    volatile uint8_t calibrate_acc_offset = 0;
    if (calibrate_acc_offset)
    {
    	MPU9250.accOffs.x = 0;
    	MPU9250.accOffs.y = 0;
    	MPU9250.accOffs.z = 0;

    	// Step with debugger through following functions, each time place IMU on the correct face
    	volatile int16_t pos, neg;

    	pos = MPU_calibrateAcc(&hspi1, &MPU9250, 1<<11, &MPU9250.rawData.ax); // X+
    	neg = MPU_calibrateAcc(&hspi1, &MPU9250, 1<<11, &MPU9250.rawData.ax); // X-
    	MPU9250.accOffs.x = (pos + neg) / 2;

    	pos = MPU_calibrateAcc(&hspi1, &MPU9250, 1<<11, &MPU9250.rawData.ay); // Y+
    	neg = MPU_calibrateAcc(&hspi1, &MPU9250, 1<<11, &MPU9250.rawData.ay); // Y-
    	MPU9250.accOffs.y = (pos + neg) / 2;

    	pos = MPU_calibrateAcc(&hspi1, &MPU9250, 1<<11, &MPU9250.rawData.az); // Z+
    	neg = MPU_calibrateAcc(&hspi1, &MPU9250, 1<<11, &MPU9250.rawData.az); // Z-
    	MPU9250.accOffs.z = (pos + neg) / 2;
    }
    else
    {
    	MPU9250.accOffs.x = 340;	// 338 344 337
    	MPU9250.accOffs.y = 320;	// 321 317 321
    	MPU9250.accOffs.z = 2746;	// 2781 2739 2717

        volatile uint8_t write_acc_offset_to_registers = 1;
        if (write_acc_offset_to_registers)
        {
            int16_t offsx = -(MPU9250.accOffs.x / 16);
            int16_t offsy = -(MPU9250.accOffs.y / 16);
            int16_t offsz = -(MPU9250.accOffs.z / 16);

            // ---------------------------------------------------------------------
            // The datasheet defines that the accelerometer user offset registers
            // (XA/YA/ZA_OFFS) store a signed 15-bit value, where 1 LSB is ~0.98 mg.
            //
            // Since the sensor outputs are in raw counts (16384 LSB/g at ±2 g),
            // one hardware offset step corresponds to roughly (0.00098 g * 16384)=16 raw counts.
            //
            //     offset_register_value = –(raw / 16)
            //
            // The resulting 15-bit value is split as follows:
            //     High byte  = offset_register_value[14:7]
            //     Low byte   = offset_register_value[6:0] << 1   (bit 0 is reserved and kept 0)
            // ---------------------------------------------------------------------

        	uint8_t high, low, reserved_bit;
        	uint16_t reg, offs_reg;

        	// X axis read
            MPU_REG_READ(&hspi1, &MPU9250, XA_OFFSET_H, &high, 1);
            MPU_REG_READ(&hspi1, &MPU9250, XA_OFFSET_L, &low, 1);
            // reconstruct the register
            reg = (int16_t)((high << 8) | low);
            // get the offset and reserved bit from register
            reserved_bit = reg & 0x01;
            offs_reg = reg >> 1;
            // add offset to the register
            offs_reg = offs_reg + offsx;
            // reconstruct the register
            reg = (offs_reg << 1) | reserved_bit;
            // X axis write
            addr = XA_OFFSET_H;
            high = (uint8_t)(reg >> 8);
            MPU_REG_WRITE(&hspi1, &MPU9250, &addr, &high);
            addr = XA_OFFSET_L;
            low = (uint8_t)(reg);
            MPU_REG_WRITE(&hspi1, &MPU9250, &addr, &low);

        	// Y axis read
            MPU_REG_READ(&hspi1, &MPU9250, YA_OFFSET_H, &high, 1);
            MPU_REG_READ(&hspi1, &MPU9250, YA_OFFSET_L, &low, 1);
            // reconstruct the register
            reg = (int16_t)((high << 8) | low);
            // get the offset and reserved bit from register
            reserved_bit = reg & 0x01;
            offs_reg = reg >> 1;
            // add offset to the register
            offs_reg = offs_reg + offsy;
            // reconstruct the register
            reg = (offs_reg << 1) | reserved_bit;
            // X axis write
            addr = YA_OFFSET_H;
            high = (uint8_t)(reg >> 8);
            MPU_REG_WRITE(&hspi1, &MPU9250, &addr, &high);
            addr = YA_OFFSET_L;
            low = (uint8_t)(reg);
            MPU_REG_WRITE(&hspi1, &MPU9250, &addr, &low);

        	// Z axis read
            MPU_REG_READ(&hspi1, &MPU9250, ZA_OFFSET_H, &high, 1);
            MPU_REG_READ(&hspi1, &MPU9250, ZA_OFFSET_L, &low, 1);
            // reconstruct the register
            reg = (int16_t)((high << 8) | low);
            // get the offset and reserved bit from register
            reserved_bit = reg & 0x01;
            offs_reg = reg >> 1;
            // add offset to the register
            offs_reg = offs_reg + offsz;
            // reconstruct the register
            reg = (offs_reg << 1) | reserved_bit;
            // X axis write
            addr = ZA_OFFSET_H;
            high = (uint8_t)(reg >> 8);
            MPU_REG_WRITE(&hspi1, &MPU9250, &addr, &high);
            addr = ZA_OFFSET_L;
            low = (uint8_t)(reg);
            MPU_REG_WRITE(&hspi1, &MPU9250, &addr, &low);

        	// Do not compensate in the software then
        	MPU9250.accOffs.x = 0;
        	MPU9250.accOffs.y = 0;
        	MPU9250.accOffs.z = 0;
        }
    }

    // Load the DMP into the MPU memory
    // maybe todo

	while (1)
	{
		osStatus_t os = osThreadFlagsWait(IMU_INTERRUPT_FLAG, osFlagsWaitAny, osWaitForever);
		if (os == IMU_INTERRUPT_FLAG)
		{
			MPU_readRawData(&hspi1, &MPU9250);

			// Flipping all raw measurements due the fact that i've mounted the IMU in the negated NED position and i want to have NED in CAN

			{
				struct cmmc_accelerometer_t tmp = {
						.ax = -MPU9250.rawData.ax, // do not use encode here
						.ay = -MPU9250.rawData.ay, // do not use encode here
						.az = -MPU9250.rawData.az, // do not use encode here
				};
				cant_generic_struct_t msg = {
						.msg_dlc	= CMMC_ACCELEROMETER_LENGTH,
						.msg_id		= CMMC_ACCELEROMETER_FRAME_ID,
						.msg_payload = { 0U },
				};
				cmmc_accelerometer_pack(msg.msg_payload, &tmp, msg.msg_dlc);
				cant_transmit(&msg);
			}

			{
				struct cmmc_gyroscope_t tmp = {
						.gx = MPU9250.rawData.gx, // do not use encode here
						.gy = MPU9250.rawData.gy, // do not use encode here
						.gz = MPU9250.rawData.gz, // do not use encode here
				};
				cant_generic_struct_t msg = {
						.msg_dlc	= CMMC_GYROSCOPE_LENGTH,
						.msg_id		= CMMC_GYROSCOPE_FRAME_ID,
						.msg_payload = { 0U },
				};
				cmmc_gyroscope_pack(msg.msg_payload, &tmp, msg.msg_dlc);
				cant_transmit(&msg);
			}
		}
		task_imu_alive++;
	}
}
