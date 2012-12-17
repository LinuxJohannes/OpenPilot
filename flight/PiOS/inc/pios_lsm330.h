/**
 ******************************************************************************
 * @addtogroup PIOS PIOS Core hardware abstraction layer
 * @{
 * @addtogroup PIOS_LSM330 LSM330 Functions
 * @brief Deals with the hardware interface to the 3-axis gyro and 3-axis accel
 * @{
 *
 * @file       PIOS_LSM330.h
 * @author     The OpenPilot Team, http://www.openpilot.org Copyright (C) 2012.
 * @brief      LSM330 3-axis gyro and 3-axis accel function headers
 * @see        The GNU Public License (GPL) Version 3
 *
 ******************************************************************************
 */
/* 
 * This program is free software; you can redistribute it and/or modify 
 * it under the terms of the GNU General Public License as published by 
 * the Free Software Foundation; either version 3 of the License, or 
 * (at your option) any later version.
 * 
 * This program is distributed in the hope that it will be useful, but 
 * WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY 
 * or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License 
 * for more details.
 * 
 * You should have received a copy of the GNU General Public License along 
 * with this program; if not, write to the Free Software Foundation, Inc., 
 * 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
 */


#ifndef PIOS_LSM330_H
#define PIOS_LSM330_H

#include "pios.h"

/**
 ******************************************************************************
 *
 *   A C C E L E R O M E T E R   S E C T I O N
 *
 ******************************************************************************
 */

/* LSM330DL ACC Register Addresses */
#define PIOS_LSM330_WHOAMI_A            0x0F
#define PIOS_LSM330_CTRL_REG1_A         0x20
#define PIOS_LSM330_CTRL_REG2_A         0x21
#define PIOS_LSM330_CTRL_REG3_A         0x22
#define PIOS_LSM330_CTRL_REG4_A         0x23
#define PIOS_LSM330_CTRL_REG5_A         0x24
#define PIOS_LSM330_CTRL_REG6_A         0x25
#define PIOS_LSM330_DATACPTURE_A        0x26
#define PIOS_LSM330_STATUS_REG_A        0x27
#define PIOS_LSM330_OUT_X_L_A           0x28
#define PIOS_LSM330_OUT_X_H_A           0x29
#define PIOS_LSM330_OUT_Y_L_A           0x2A
#define PIOS_LSM330_OUT_Y_H_A           0x2B
#define PIOS_LSM330_OUT_Z_L_A           0x2C
#define PIOS_LSM330_OUT_Z_H_A           0x2D
#define PIOS_LSM330_FIFO_CTRL_REG_A     0x2E
#define PIOS_LSM330_FIFO_SRC_REG_A      0x2F
#define PIOS_LSM330_INT1_CFG_A          0x30
#define PIOS_LSM330_INT1_SRC_A          0x31
#define PIOS_LSM330_INT1_THS_A          0x32
#define PIOS_LSM330_INT1_DURATION_A     0x33
#define PIOS_LSM330_INT2_CFG_A          0x34
#define PIOS_LSM330_INT2_SRC_A          0x35
#define PIOS_LSM330_INT2_THS_A          0x36
#define PIOS_LSM330_INT_DURATION_A      0x37
#define PIOS_LSM330_CLICK_CFG_A         0x38
#define PIOS_LSM330_CLICK_SRC_A         0x39
#define PIOS_LSM330_CLICK_THS_A         0x3A
#define PIOS_LSM330_TIME_LIMIT_A        0x3B
#define PIOS_LSM330_TIME_LATENCY_A      0x3C
#define PIOS_LSM330_TIME_WINDOW_A       0x3D

enum pios_lsm330_acc_range {
	PIOS_LSM330_ACCEL_2G  = 0x00,
	PIOS_LSM330_ACCEL_4G  = 0x10,
	PIOS_LSM330_ACCEL_8G  = 0x20,
	PIOS_LSM330_ACCEL_16G = 0x30
};

// Defined by data rate, not BW
struct pios_lsm330_acc_data {
	int16_t acc_x;
	int16_t acc_y;
	int16_t acc_z;
};

struct pios_lsm330_acc_cfg {
	enum pios_lsm330_acc_range range;
};

/* Public Functions */
extern int32_t PIOS_LSM330_Acc_Init(uint32_t spi_id, uint32_t slave_num, const struct pios_lsm330_acc_cfg * cfg);
extern xQueueHandle PIOS_LSM330_Acc_GetQueue();
extern int32_t PIOS_LSM330_Acc_FifoElements();
extern int32_t PIOS_LSM330_ReadAccel(struct pios_lsm330_acc_data * buffer);
extern float PIOS_LSM330_GetAccelScale();
extern int32_t PIOS_LSM330_Acc_ReadID();
extern uint8_t PIOS_LSM330_Acc_Test();

/**
 ******************************************************************************
 *
 *   G Y R O S C O P E   S E C T I O N
 *
 ******************************************************************************
 */

/* LSM330DL GYRO Register Addresses */
#define PIOS_LSM330_WHOAMI_G            0x0F
#define PIOS_LSM330_CTRL_REG1_G         0x20
#define PIOS_LSM330_CTRL_REG2_G         0x21
#define PIOS_LSM330_CTRL_REG3_G         0x22
#define PIOS_LSM330_CTRL_REG4_G         0x23
#define PIOS_LSM330_CTRL_REG5_G         0x24
#define PIOS_LSM330_DATACAPTURE_G       0x25
#define PIOS_LSM330_OUT_TEMP_G          0x26
#define PIOS_LSM330_STATUS_REG_G        0x27
#define PIOS_LSM330_OUT_X_L_G           0x28
#define PIOS_LSM330_OUT_X_H_G           0x29
#define PIOS_LSM330_OUT_Y_L_G           0x2A
#define PIOS_LSM330_OUT_Y_H_G           0x2B
#define PIOS_LSM330_OUT_Z_L_G           0x2C
#define PIOS_LSM330_OUT_Z_H_G           0x2D
#define PIOS_LSM330_FIFO_CTRL_REG_G     0x2E
#define PIOS_LSM330_FIFO_SRC_REG_G      0x2F
#define PIOS_LSM330_INT1_CFG_G          0x30
#define PIOS_LSM330_INT1_SRC_G          0x31
#define PIOS_LSM330_INT1_THS_XH_G       0x32
#define PIOS_LSM330_INT1_THS_XL_G       0x33
#define PIOS_LSM330_INT1_THS_YH_G       0x34
#define PIOS_LSM330_INT1_THS_YL_G       0x35
#define PIOS_LSM330_INT1_THS_ZH_G       0x36
#define PIOS_LSM330_INT1_THS_ZL_G       0x37
#define PIOS_LSM330_INT1_DURATION_G     0x38

/* Ctrl1 flags */
#define PIOS_LSM330_CTRL1_FASTEST        0xF0
#define PIOS_LSM330_CTRL1_380HZ_100HZ    0xB0
#define PIOS_LSM330_CTRL1_PD             0x08
#define PIOS_LSM330_CTRL1_ZEN            0x04
#define PIOS_LSM330_CTRL1_YEN            0x02
#define PIOS_LSM330_CTRL1_XEN            0x01

/* FIFO enable for storing different values */
#define PIOS_LSM330_FIFO_TEMP_OUT        0x80
#define PIOS_LSM330_FIFO_GYRO_X_OUT      0x40
#define PIOS_LSM330_FIFO_GYRO_Y_OUT      0x20
#define PIOS_LSM330_FIFO_GYRO_Z_OUT      0x10
#define PIOS_LSM330_ACCEL_OUT            0x08

/* Interrupt Configuration */
#define PIOS_LSM330_INT_ACTL             0x80
#define PIOS_LSM330_INT_OPEN             0x40
#define PIOS_LSM330_INT_LATCH_EN         0x20
#define PIOS_LSM330_INT_CLR_ANYRD        0x10

#define PIOS_LSM330_INTEN_OVERFLOW       0x10
#define PIOS_LSM330_INTEN_DATA_RDY       0x01

/* Interrupt status */
#define PIOS_LSM330_INT_STATUS_FIFO_FULL 0x80
#define PIOS_LSM330_INT_STATUS_IMU_RDY   0X04
#define PIOS_LSM330_INT_STATUS_DATA_RDY  0X01

/* User control functionality */
#define PIOS_LSM330_USERCTL_FIFO_EN      0X40
#define PIOS_LSM330_USERCTL_FIFO_RST     0X02
#define PIOS_LSM330_USERCTL_GYRO_RST     0X01

/* Power management and clock selection */
#define PIOS_LSM330_PWRMGMT_IMU_RST      0X80
#define PIOS_LSM330_PWRMGMT_INTERN_CLK   0X00
#define PIOS_LSM330_PWRMGMT_PLL_X_CLK    0X01
#define PIOS_LSM330_PWRMGMT_PLL_Y_CLK    0X02
#define PIOS_LSM330_PWRMGMT_PLL_Z_CLK    0X03
#define PIOS_LSM330_PWRMGMT_STOP_CLK     0X07

enum pios_lsm330_gyro_range {
	PIOS_LSM330_SCALE_250_DEG  = 0x00,
	PIOS_LSM330_SCALE_500_DEG  = 0x10,
	PIOS_LSM330_SCALE_2000_DEG = 0x03
};

enum pios_lsm330_gyro_filter {
	PIOS_LSM330_LOWPASS_256_HZ = 0x00,
	PIOS_LSM330_LOWPASS_188_HZ = 0x01,
	PIOS_LSM330_LOWPASS_98_HZ  = 0x02,
	PIOS_LSM330_LOWPASS_42_HZ  = 0x03,
	PIOS_LSM330_LOWPASS_20_HZ  = 0x04,
	PIOS_LSM330_LOWPASS_10_HZ  = 0x05,
	PIOS_LSM330_LOWPASS_5_HZ   = 0x06
};

struct pios_lsm330_gyro_data {
	int16_t gyro_x;
	int16_t gyro_y;
	int16_t gyro_z;
	int16_t temperature;
};

struct pios_lsm330_gyro_cfg {
	const struct pios_exti_cfg * exti_cfg; /* Pointer to the EXTI configuration */
	enum pios_lsm330_gyro_range range;
};

/* Public Functions */
extern int32_t PIOS_LSM330_Gyro_Init(uint32_t spi_id, uint32_t slave_num, const struct pios_lsm330_gyro_cfg * cfg);
extern xQueueHandle PIOS_LSM330_Gyro_GetQueue();
extern int32_t PIOS_LSM330_ReadGyros(struct pios_lsm330_gyro_data * buffer);
extern float PIOS_LSM330_GetGyroScale();
extern int32_t PIOS_LSM330_Gyro_ReadID();
extern uint8_t PIOS_LSM330_Gyro_Test();
extern bool PIOS_LSM330_Gyro_IRQHandler();

#endif /* PIOS_LSM330_H */

/** 
  * @}
  * @}
  */
