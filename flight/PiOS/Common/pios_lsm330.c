/**
 ******************************************************************************
 * @addtogroup PIOS PIOS Core hardware abstraction layer
 * @{
 * @addtogroup PIOS_LSM330 LSM330 Functions
 * @brief Deals with the hardware interface to the 3-axis gyro and 3-axis accel
 * @{
 *
 * @file       pios_lsm330.c
 * @author     The OpenPilot Team, http://www.openpilot.org Copyright (C) 2012.
 * @brief      LSM330 3-axis gyro and 3-axis accel chip
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

/* Project Includes */
#include "pios.h"

#if defined(PIOS_INCLUDE_LSM330)

#include "fifo_buffer.h"

#define PIOS_LSM330_DELAY_MS 10

/**
 ******************************************************************************
 *
 *   A C C E L E R O M E T E R   S E C T I O N
 *
 ******************************************************************************
 */

 /* Global Variables */
enum pios_lsm330_acc_dev_magic {
	PIOS_LSM330_ACC_DEV_MAGIC = 0x9d39bcec,
};

struct lsm330_acc_dev {
	uint32_t spi_id;
	uint32_t slave_num;
	const struct pios_lsm330_acc_cfg * cfg;
	enum pios_lsm330_acc_dev_magic magic;
};

//! Global structure for this device device
static struct lsm330_acc_dev * acc_dev;

//! Private functions
static struct lsm330_acc_dev * PIOS_LSM330_acc_alloc(void);
static int32_t PIOS_LSM330_Acc_Validate(struct lsm330_acc_dev * dev);
static void PIOS_LSM330_Acc_Config(struct pios_lsm330_acc_cfg const * cfg);
static int32_t PIOS_LSM330_Acc_SetReg(uint8_t address, uint8_t buffer);
static int32_t PIOS_LSM330_Acc_GetReg(uint8_t address);
static int32_t PIOS_LSM330_Acc_ClaimBus();
static int32_t PIOS_LSM330_Acc_ReleaseBus();

volatile bool lsm330_acc_configured = false;

#define GRAV 9.81f

/**
 * @brief Allocate a new device
 */
static struct lsm330_acc_dev * PIOS_LSM330_acc_alloc(void)
{
	struct lsm330_acc_dev * lsm330_acc_dev;

	lsm330_acc_dev = (struct lsm330_acc_dev *)pvPortMalloc(sizeof(*lsm330_acc_dev));
	if (!lsm330_acc_dev) return (NULL);

	lsm330_acc_dev->magic = PIOS_LSM330_ACC_DEV_MAGIC;

	return(lsm330_acc_dev);
}

/**
 * @brief Validate the handle to the spi device
 * @returns 0 for valid device or -1 otherwise
 */
static int32_t PIOS_LSM330_Acc_Validate(struct lsm330_acc_dev * dev)
{
	if (dev == NULL) 
		return -1;
	if (dev->magic != PIOS_LSM330_ACC_DEV_MAGIC)
		return -2;
	if (dev->spi_id == 0)
		return -3;
	return 0;
}

/**
 * @brief Initialize the LSM330 3-axis acc sensor.
 * @return none
 */
#include <pios_board_info.h>
int32_t PIOS_LSM330_Acc_Init(uint32_t spi_id, uint32_t slave_num, const struct pios_lsm330_acc_cfg * cfg)
{
	acc_dev = PIOS_LSM330_acc_alloc();
	if(acc_dev == NULL)
		return -1;

	acc_dev->spi_id = spi_id;
	acc_dev->slave_num = slave_num;
	acc_dev->cfg = cfg;

	/* Configure the LSM330 Sensor */
	PIOS_LSM330_Acc_Config(cfg);

	return 0;
}

/**
 * @brief Initialize the LSM330 3-axis gyro sensor
 * \return none
 * \param[in] PIOS_LSM330_ConfigTypeDef struct to be used to configure sensor.
*
*/
static void PIOS_LSM330_Acc_Config(struct pios_lsm330_acc_cfg const * cfg)
{
	// Reboot chip memory
	//   | BOOT | FIFO_EN | -- | -- | LIR_INT1 | D4D_INT1 | 0 | 0 |
	//   BOOT = 1
	while (PIOS_LSM330_Acc_SetReg(PIOS_LSM330_CTRL_REG5_A, 0b10000000) != 0);
	PIOS_DELAY_WaitmS(PIOS_LSM330_DELAY_MS);

	// Disable interrupts
	while (PIOS_LSM330_Acc_SetReg(PIOS_LSM330_CTRL_REG3_A, 0b00000000) != 0);
	// Clear FIFO
	while (PIOS_LSM330_Acc_SetReg(PIOS_LSM330_FIFO_CTRL_REG_A, 0b00000000) != 0);
	PIOS_DELAY_WaitmS(PIOS_LSM330_DELAY_MS);

	// FIFO configuration
	//   | FM1 | FM0 | TR | FTH4 | FTH3 | FTH2 | FTH1 | FTH0 |
	//   FM = b10 (Stream), FTH = 0
	while (PIOS_LSM330_Acc_SetReg(PIOS_LSM330_FIFO_CTRL_REG_A, 0b10000000) != 0);

	// Configure FIFO and interrupt latching
	//   | BOOT | FIFO_EN | -- | -- | LIR_INT1 | D4D_INT1 | 0 | 0 |
	//   FIFO_EN = 1
	while (PIOS_LSM330_Acc_SetReg(PIOS_LSM330_CTRL_REG5_A, 0b01000000) != 0);

	// High-pass filter configuration
	//   | HPM1 | HPM0 | HPCF2 | HPCF1 | FDS | HPCLICK | HPIS2 | HPIS1 |
	//   Default
	while (PIOS_LSM330_Acc_SetReg(PIOS_LSM330_CTRL_REG2_A, 0b00000000) != 0);

	// Mode and Full-scale selection
	//  | BDU | BLE | FS1 | FS0 | HR | 0 | 0 | SIM |
	//   HR = 1
	while (PIOS_LSM330_Acc_SetReg(PIOS_LSM330_CTRL_REG4_A, 0b00001000 | cfg->range) != 0);

	// Output Data Rate, Power Mode and Axies configuratios
	//   | ODR3 | ODR2 | ODR1 | ODR0 | LPen | Zen | Yen | Xen |
	//   ODR = 1344 Hz, LPen = 0, Zen = 1, Yen = 1, Xen = 1
	while (PIOS_LSM330_Acc_SetReg(PIOS_LSM330_CTRL_REG1_A, 0b10010111) != 0);

	// Interrupt configuration
	//   | I1_CLICK | I1_AOI1 | 0 | I1_DRDY1 | I1_DRDY2 | I1_WTM | I1_OVERRUN  | -- |
	//   Default
	while (PIOS_LSM330_Acc_SetReg(PIOS_LSM330_CTRL_REG3_A, 0b00000000) != 0);

	lsm330_acc_configured = true;
}

/**
 * @brief Claim the SPI bus for the accel communications and select this chip
 * @return 0 if successful, -1 for invalid device, -2 if unable to claim bus
 */
static int32_t PIOS_LSM330_Acc_ClaimBus()
{
	if(PIOS_LSM330_Acc_Validate(acc_dev) != 0)
		return -1;

	if(PIOS_SPI_ClaimBus(acc_dev->spi_id) != 0)
		return -2;

	PIOS_SPI_RC_PinSet(acc_dev->spi_id,acc_dev->slave_num,0);
	return 0;
}

/**
 * @brief Release the SPI bus for the accel communications and end the transaction
 * @return 0 if successful, -1 for invalid device
 */
int32_t PIOS_LSM330_Acc_ReleaseBus()
{
	if(PIOS_LSM330_Acc_Validate(acc_dev) != 0)
		return -1;

	PIOS_SPI_RC_PinSet(acc_dev->spi_id,acc_dev->slave_num,1);

	return PIOS_SPI_ReleaseBus(acc_dev->spi_id);
}

/**
 * @brief Read a register from LSM330
 * @returns The register value or -1 if failure to get bus
 * @param reg[in] Register address to be read
 */
static int32_t PIOS_LSM330_Acc_GetReg(uint8_t reg)
{
	uint8_t data;

	if(PIOS_LSM330_Acc_ClaimBus() != 0)
		return -1;

	PIOS_SPI_TransferByte(acc_dev->spi_id,(0x80 | reg) ); // request byte
	data = PIOS_SPI_TransferByte(acc_dev->spi_id,0 );     // receive response

	PIOS_LSM330_Acc_ReleaseBus();
	return data;
}

/**
 * @brief Writes one byte to the LSM330
 * \param[in] reg Register address
 * \param[in] data Byte to write
 * \return 0 if operation was successful
 * \return -1 if unable to claim SPI bus
 * \return -2 if unable to claim i2c device
 */
static int32_t PIOS_LSM330_Acc_SetReg(uint8_t reg, uint8_t data)
{
	if(PIOS_LSM330_Acc_ClaimBus() != 0)
		return -1;

	PIOS_SPI_TransferByte(acc_dev->spi_id, 0x7f & reg);
	PIOS_SPI_TransferByte(acc_dev->spi_id, data);

	PIOS_LSM330_Acc_ReleaseBus();
	return 0;
}

/**
 * @brief Return number of entries in the fifo
 */
int32_t PIOS_LSM330_Acc_FifoElements()
{
	if(PIOS_LSM330_Acc_Validate(acc_dev) != 0)
		return -1;

	int32_t samples = PIOS_LSM330_Acc_GetReg(PIOS_LSM330_FIFO_SRC_REG_A);
	samples &= 0b00011111;

	return samples;
}

/**
 * @brief Read current X, Z, Y values (in that order)
 * \param[out] int16_t array of size 3 to store X, Z, and Y magnetometer readings
 * \returns The number of samples remaining in the fifo
 */
int32_t PIOS_LSM330_ReadAccel(struct pios_lsm330_acc_data * data)
{
	uint8_t buf[7] = {PIOS_LSM330_OUT_X_L_A | 0x80 | 0x40, 0, 0, 0, 0, 0, 0};
	uint8_t rec[7];
	
	if(PIOS_LSM330_Acc_ClaimBus() != 0)
		return -1;

	if(PIOS_SPI_TransferBlock(acc_dev->spi_id, &buf[0], &rec[0], sizeof(buf), NULL) < 0) {
		PIOS_LSM330_Acc_ReleaseBus();
		data->acc_x = 0;
		data->acc_y = 0;
		data->acc_z = 0;
		return -2;
	}

	PIOS_LSM330_Acc_ReleaseBus();
	
	memcpy((uint8_t *) &(data->acc_x), &rec[1], 6);

	int32_t samples = PIOS_LSM330_Acc_GetReg(PIOS_LSM330_FIFO_SRC_REG_A);
	samples &= 0b00011111;

	return samples;
}

/**
 * @brief Read the identification bytes from the LSM330 sensor
 * \return ID read from LSM330 or -1 if failure
*/
int32_t PIOS_LSM330_Acc_ReadID()
{
	int32_t lsm330_id = PIOS_LSM330_Acc_GetReg(PIOS_LSM330_WHOAMI_A);
	if(lsm330_id < 0)
		return -1;
	return lsm330_id;
}

float PIOS_LSM330_GetAccelScale() 
{
	if(PIOS_LSM330_Acc_Validate(acc_dev) != 0)
		return -1;

	switch (acc_dev->cfg->range) {
		case PIOS_LSM330_ACCEL_2G:
			return (GRAV / 16384.0f);
		case PIOS_LSM330_ACCEL_4G:
			return (GRAV / 8192.0f);
		case PIOS_LSM330_ACCEL_8G:
			return (GRAV / 4096.0f);
		case PIOS_LSM330_ACCEL_16G:
			return (GRAV / 2048.0f);
	}
	return 0;
}

/**
 * @brief Run self-test operation.
 * \return 0 if test succeeded
 * \return non-zero value if test succeeded
 */
uint8_t PIOS_LSM330_Acc_Test(void)
{
	int32_t lsm330_acc_id = PIOS_LSM330_Acc_ReadID();
	if(lsm330_acc_id < 0)
		return -1;

	uint8_t id = lsm330_acc_id;
	if(id == 0xD4)
		return 0;

	return -2;
}

/**
 ******************************************************************************
 *
 *   G Y R O S C O P E   S E C T I O N
 *
 ******************************************************************************
 */

/* Global Variables */
enum pios_lsm330_gyro_dev_magic {
	PIOS_LSM330_GYRO_DEV_MAGIC = 0x9d39bced,
};

#define PIOS_LSM330_GYRO_MAX_DOWNSAMPLE 3
struct lsm330_gyro_dev {
	uint32_t spi_id;
	uint32_t slave_num;
	xQueueHandle queue;
	const struct pios_lsm330_gyro_cfg * cfg;
	enum pios_lsm330_gyro_filter bandwidth;
	enum pios_lsm330_gyro_range range;
	enum pios_lsm330_gyro_dev_magic magic;
};

//! Global structure for this device device
static struct lsm330_gyro_dev * gyro_dev;

//! Private functions
static struct lsm330_gyro_dev * PIOS_LSM330_gyro_alloc(void);
static int32_t PIOS_LSM330_Gyro_Validate(struct lsm330_gyro_dev * dev);
static void PIOS_LSM330_Gyro_Config(struct pios_lsm330_gyro_cfg const * cfg);
static int32_t PIOS_LSM330_Gyro_SetReg(uint8_t address, uint8_t buffer);
static int32_t PIOS_LSM330_Gyro_GetReg(uint8_t address);
static int32_t PIOS_LSM330_Gyro_GetRegIsr(uint8_t address, bool *woken);
static int32_t PIOS_LSM330_Gyro_ClaimBus();
static int32_t PIOS_LSM330_Gyro_ClaimBusIsr();
static int32_t PIOS_LSM330_Gyro_ReleaseBus();
static int32_t PIOS_LSM330_Gyro_ReleaseBusIsr(bool *woken);

volatile bool lsm330_gyro_configured = false;

/* Local Variables */
#define DEG_TO_RAD (M_PI / 180.0)

/**
 * @brief Allocate a new device
 */
static struct lsm330_gyro_dev * PIOS_LSM330_gyro_alloc(void)
{
	struct lsm330_gyro_dev * lsm330_gyro_dev;

	lsm330_gyro_dev = (struct lsm330_gyro_dev *)pvPortMalloc(sizeof(*lsm330_gyro_dev));
	if (!lsm330_gyro_dev) return (NULL);

	lsm330_gyro_dev->magic = PIOS_LSM330_GYRO_DEV_MAGIC;

	lsm330_gyro_dev->queue = xQueueCreate(PIOS_LSM330_GYRO_MAX_DOWNSAMPLE, sizeof(struct pios_lsm330_gyro_data));
	if(lsm330_gyro_dev->queue == NULL) {
		vPortFree(lsm330_gyro_dev);
		return NULL;
	}

	return(lsm330_gyro_dev);
}

/**
 * @brief Validate the handle to the spi device
 * @returns 0 for valid device or -1 otherwise
 */
static int32_t PIOS_LSM330_Gyro_Validate(struct lsm330_gyro_dev * dev)
{
	if (dev == NULL) 
		return -1;
	if (dev->magic != PIOS_LSM330_GYRO_DEV_MAGIC)
		return -2;
	if (dev->spi_id == 0)
		return -3;
	return 0;
}

/**
 * @brief Initialize the LSM330 3-axis gyro sensor.
 * @return none
 */
#include <pios_board_info.h>
int32_t PIOS_LSM330_Gyro_Init(uint32_t spi_id, uint32_t slave_num, const struct pios_lsm330_gyro_cfg * cfg)
{
	gyro_dev = PIOS_LSM330_gyro_alloc();
	if(gyro_dev == NULL)
		return -1;

	gyro_dev->spi_id = spi_id;
	gyro_dev->slave_num = slave_num;
	gyro_dev->cfg = cfg;

	/* Configure the LSM330 Sensor */
	PIOS_LSM330_Gyro_Config(cfg);

	/* Set up EXTI */
	PIOS_EXTI_Init(cfg->exti_cfg);

	// An initial read is needed to get it running
	struct pios_lsm330_gyro_data data;
	PIOS_LSM330_ReadGyros(&data);

	return 0;
}

/**
 * @brief Initialize the LSM330 3-axis gyro sensor
 * \return none
 * \param[in] PIOS_LSM330_ConfigTypeDef struct to be used to configure sensor.
*
*/
static void PIOS_LSM330_Gyro_Config(struct pios_lsm330_gyro_cfg const * cfg)
{
	// Reboot memory content
	//   | BOOT | FIFO_EN | -- | HPen | INT1_SEL1 | INT1_SEL0 | Out_SEL1 | OUT_SEL0 |
	//   Boot = 1
	while (PIOS_LSM330_Gyro_SetReg(PIOS_LSM330_CTRL_REG5_G, 0b10000000) != 0);
	PIOS_DELAY_WaitmS(PIOS_LSM330_DELAY_MS);

	// Disable interrupts
	while (PIOS_LSM330_Gyro_SetReg(PIOS_LSM330_CTRL_REG3_G, 0b00000000) != 0);
	// Clear FIFO
	while (PIOS_LSM330_Gyro_SetReg(PIOS_LSM330_FIFO_CTRL_REG_G, 0b01000000) != 0);
	PIOS_DELAY_WaitmS(PIOS_LSM330_DELAY_MS);

	// High-pass filter configuration
	//   | 0 | 0 | HPM1 | HPM0 | HPCF3 | HPCF2 | HPCF1 | HPCF0 |
	// Default
	while (PIOS_LSM330_Gyro_SetReg(PIOS_LSM330_CTRL_REG2_G, 0b00000000) != 0);

	// Boot, FIFO, High-pass enable and Output selection
	//   | BOOT | FIFO_EN | -- | HPen | INT1_SEL1 | INT1_SEL0 | Out_SEL1 | OUT_SEL0 |
	//   FIFO enable, Out_SEL = b10 Data is low-pass filtered by LPF2
	while (PIOS_LSM330_Gyro_SetReg(PIOS_LSM330_CTRL_REG5_G, 0b01000010) != 0);

	// Full-scale selection
	//  | BDU | BLE | FS1 | FS0 | -- | 0 | 0 | SIM |
	while(PIOS_LSM330_Gyro_SetReg(PIOS_LSM330_CTRL_REG4_G, 0b00000000 | cfg->range) != 0);

	// Output Data Rate, Bandwidth, Power-down and Axies configuratios
	//   | DR1 | DR0 | BW1 | BW0 | PD | Zen | Yen | Xen |
	//   ODR = 800Hz, BW = 110Hz, PD = 1, Zen = 1, Yen = 1, Xen = 1
	while (PIOS_LSM330_Gyro_SetReg(PIOS_LSM330_CTRL_REG1_G, 0b11111111) != 0);

	// FIFO configuration
	//   | FM2 | FM1 | FM0 | WTM4 | WTM3 | WTM2 | WTM1 | WTM0 |
	//   FM = b010 (STREAM), WTM = 1
	while (PIOS_LSM330_Gyro_SetReg(PIOS_LSM330_FIFO_CTRL_REG_G, 0b01000001) != 0);

	// Interrupt configuration
	//   | I1_Int1 | I1_Boot | H_Lactive | PP_OD | I2_DRDY | I2_WTM | I2_ORun | I2_Empty |
	//   I2_WTM = 1
	while (PIOS_LSM330_Gyro_SetReg(PIOS_LSM330_CTRL_REG3_G, 0b00000100) != 0);

	lsm330_gyro_configured = true;
}

/**
 * @brief Claim the SPI bus for the accel communications and select this chip
 * @return 0 if successful, -1 for invalid device, -2 if unable to claim bus
 */
static int32_t PIOS_LSM330_Gyro_ClaimBus()
{
	if(PIOS_LSM330_Gyro_Validate(gyro_dev) != 0)
		return -1;

	if(PIOS_SPI_ClaimBus(gyro_dev->spi_id) != 0)
		return -2;

	PIOS_SPI_RC_PinSet(gyro_dev->spi_id,gyro_dev->slave_num,0);
	return 0;
}

/**
 * @brief Claim the SPI bus for the accel communications and select this chip
 * @return 0 if successful, -1 for invalid device, -2 if unable to claim bus
 */
static int32_t PIOS_LSM330_Gyro_ClaimBusIsr()
{
	if(PIOS_LSM330_Gyro_Validate(gyro_dev) != 0)
		return -1;

	if(PIOS_SPI_ClaimBusISR(gyro_dev->spi_id) != 0)
		return -2;

	PIOS_SPI_RC_PinSet(gyro_dev->spi_id,gyro_dev->slave_num,0);
	return 0;
}

/**
 * @brief Release the SPI bus for the accel communications and end the transaction
 * @return 0 if successful, -1 for invalid device
 */
int32_t PIOS_LSM330_Gyro_ReleaseBus()
{
	if(PIOS_LSM330_Gyro_Validate(gyro_dev) != 0)
		return -1;

	PIOS_SPI_RC_PinSet(gyro_dev->spi_id,gyro_dev->slave_num,1);

	return PIOS_SPI_ReleaseBus(gyro_dev->spi_id);
}

/**
 * @brief Release the SPI bus for the accel communications and end the transaction
 * \param[in] pointer which receives if a task has been woken
 * @return 0 if successful, -1 for invalid device
 */
int32_t PIOS_LSM330_Gyro_ReleaseBusIsr(bool *woken)
{
	if(PIOS_LSM330_Gyro_Validate(gyro_dev) != 0)
		return -1;

	PIOS_SPI_RC_PinSet(gyro_dev->spi_id,gyro_dev->slave_num,1);

	return PIOS_SPI_ReleaseBusISR(gyro_dev->spi_id, woken);
}

/**
 * @brief Read a register from LSM330
 * @returns The register value or -1 if failure to get bus
 * @param reg[in] Register address to be read
 */
static int32_t PIOS_LSM330_Gyro_GetReg(uint8_t reg)
{
	uint8_t data;

	if(PIOS_LSM330_Gyro_ClaimBus() != 0)
		return -1;

	PIOS_SPI_TransferByte(gyro_dev->spi_id,(0x80 | reg) ); // request byte
	data = PIOS_SPI_TransferByte(gyro_dev->spi_id,0 );     // receive response

	PIOS_LSM330_Gyro_ReleaseBus();
	return data;
}

/**
 * @brief Read a register from LSM330 from ISR context
 * @returns The register value or -1 if failure to get bus
 * @param reg[in] Register address to be read
 * \param[in] task woken
 */
static int32_t PIOS_LSM330_Gyro_GetRegIsr(uint8_t reg, bool *woken)
{
	uint8_t data;

	if(PIOS_LSM330_Gyro_ClaimBusIsr(woken) != 0)
		return -1;

	PIOS_SPI_TransferByte(gyro_dev->spi_id,(0x80 | reg) ); // request byte
	data = PIOS_SPI_TransferByte(gyro_dev->spi_id,0 );     // receive response

	PIOS_LSM330_Gyro_ReleaseBusIsr(woken);
	return data;
}

/**
 * @brief Writes one byte to the LSM330
 * \param[in] reg Register address
 * \param[in] data Byte to write
 * \return 0 if operation was successful
 * \return -1 if unable to claim SPI bus
 * \return -2 if unable to claim i2c device
 */
static int32_t PIOS_LSM330_Gyro_SetReg(uint8_t reg, uint8_t data)
{
	if(PIOS_LSM330_Gyro_ClaimBus() != 0)
		return -1;

	PIOS_SPI_TransferByte(gyro_dev->spi_id, 0x7f & reg);
	PIOS_SPI_TransferByte(gyro_dev->spi_id, data);

	PIOS_LSM330_Gyro_ReleaseBus();

	return 0;
}

/**
 * @brief Read current X, Z, Y values (in that order)
 * \param[out] int16_t array of size 3 to store X, Z, and Y magnetometer readings
 * \returns The number of samples remaining in the fifo
 */
uint32_t lsm330_irq = 0;
int32_t PIOS_LSM330_ReadGyros(struct pios_lsm330_gyro_data * data)
{
	uint8_t buf[7] = {PIOS_LSM330_OUT_X_L_G | 0x80 | 0x40, 0, 0, 0, 0, 0, 0};
	uint8_t rec[7];

	uint8_t samples = PIOS_LSM330_Gyro_GetReg(PIOS_LSM330_FIFO_SRC_REG_G);
	samples &= 0b00011111; // remove FIFO status bits;

	while (samples) {
		if(PIOS_LSM330_Gyro_ClaimBus() != 0)
			return -1;

		if(PIOS_SPI_TransferBlock(gyro_dev->spi_id, &buf[0], &rec[0], sizeof(buf), NULL) < 0) {
			PIOS_LSM330_Gyro_ReleaseBus();
			data->gyro_x = 0;
			data->gyro_y = 0;
			data->gyro_z = 0;
			data->temperature = 0;
			return -2;
		}

		PIOS_LSM330_Gyro_ReleaseBus();
		--samples;
	}

	memcpy((uint8_t *) &(data->gyro_x), &rec[1], 6);
	data->temperature = PIOS_LSM330_Gyro_GetReg(PIOS_LSM330_OUT_TEMP_G);

	return 0;
}

/**
 * @brief Read the identification bytes from the LSM330 sensor
 * \return ID read from LSM330 or -1 if failure
*/
int32_t PIOS_LSM330_Gyro_ReadID()
{
	int32_t lsm330_id = PIOS_LSM330_Gyro_GetReg(PIOS_LSM330_WHOAMI_G);
	if(lsm330_id < 0)
		return -1;
	return lsm330_id;
}

/**
 * \brief Reads the queue handle
 * \return Handle to the queue or null if invalid device
 */
xQueueHandle PIOS_LSM330_Gyro_GetQueue()
{
	if(PIOS_LSM330_Gyro_Validate(gyro_dev) != 0)
		return (xQueueHandle) NULL;

	return gyro_dev->queue;
}

/*
 *
 */
float PIOS_LSM330_GetGyroScale() 
{
	if(PIOS_LSM330_Gyro_Validate(gyro_dev) != 0)
		return -1;

	switch (gyro_dev->cfg->range) {
		case PIOS_LSM330_SCALE_250_DEG:
			return 0.00875f;
		case PIOS_LSM330_SCALE_500_DEG:
			return 0.01750f;
		case PIOS_LSM330_SCALE_2000_DEG:
			return 0.070f;
	}
	return 0;
}

/**
 * @brief Run self-test operation.
 * \return 0 if test succeeded
 * \return non-zero value if test succeeded
 */
uint8_t PIOS_LSM330_Gyro_Test(void)
{
	int32_t lsm330_gyro_id = PIOS_LSM330_Gyro_ReadID();
	if(lsm330_gyro_id < 0)
		return -1;

	uint8_t id = lsm330_gyro_id;
	if(id == 0xD4)
		return 0;

	return -2;
}

/**
* @brief IRQ Handler.  Read all the data from onboard buffer
*/
bool PIOS_LSM330_Gyro_IRQHandler(void)
{
	lsm330_irq++;

	struct pios_lsm330_gyro_data data;
	uint8_t buf[7] = {PIOS_LSM330_OUT_X_L_G | 0x80 | 0x40, 0, 0, 0, 0, 0, 0};
	uint8_t rec[7];
	bool woken = false;
	portBASE_TYPE xHigherPriorityTaskWoken;

	uint8_t samples = PIOS_LSM330_Gyro_GetRegIsr(PIOS_LSM330_FIFO_SRC_REG_G, &woken);
	samples &= 0b00011111; // remove FIFO status bits;

	/* This code duplicates ReadGyros above but uses ClaimBusIsr */
	while(samples) {
		if(PIOS_LSM330_Gyro_ClaimBusIsr() != 0)
			return woken;

		if(PIOS_SPI_TransferBlock(gyro_dev->spi_id, &buf[0], &rec[0], sizeof(buf), NULL) < 0) {
			PIOS_LSM330_Gyro_ReleaseBusIsr(&woken);
			return woken;
		}

		PIOS_LSM330_Gyro_ReleaseBusIsr(&woken);

		memcpy((uint8_t *) &(data.gyro_x), &rec[1], 6);
		data.temperature = PIOS_LSM330_Gyro_GetRegIsr(PIOS_LSM330_OUT_TEMP_G, &woken);

		xQueueSendToBackFromISR(gyro_dev->queue, (void *) &data, &xHigherPriorityTaskWoken);
		--samples;
	}
	return woken || (xHigherPriorityTaskWoken == pdTRUE);
}

#endif /* PIOS_INCLUDE_LSM330 */

/**
 * @}
 * @}
 */