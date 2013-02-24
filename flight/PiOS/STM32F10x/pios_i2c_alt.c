/**
 ******************************************************************************
 * @addtogroup PIOS PIOS Core hardware abstraction layer
 * @{
 * @addtogroup   PIOS_I2C I2C Functions
 * @brief STM32 Hardware dependent I2C functionality
 * @{
 *
 * @file       pios_i2c.c
 * @author     The OpenPilot Team, http://www.openpilot.org Copyright (C) 2010.
 * @brief      I2C Enable/Disable routines
 * @see        The GNU Public License (GPL) Version 3
 *
 *****************************************************************************/
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

/*
 ******************************************************************************
 * Quick and dirty workaround of PIOS I2C HAL for STM32F10X
 *
 * The majority of the code is taken from STM32InertialMonitor
 * https://github.com/Laurenceb/STM32InertialMonitor
 *
 ******************************************************************************
 */

/* Project Includes */
#include "pios.h"

#if defined(PIOS_INCLUDE_I2C)

#if defined(PIOS_INCLUDE_FREERTOS)
#define USE_FREERTOS
#endif

#include <pios_i2c_priv.h>

#ifdef USE_FREERTOS
#define I2C_DEFAULT_TIMEOUT 50
#else
#define I2C_DEFAULT_TIMEOUT 30000
#endif

static volatile uint16_t i2cErrorCount = 0;

static volatile bool error = false;
static volatile bool busy;

static volatile uint8_t addr;
static volatile uint8_t reg;
static volatile uint8_t bytes;
static volatile uint8_t dir;
static volatile uint8_t* buf_p;

static void i2c_adapter_fsm_init(struct pios_i2c_adapter *i2c_adapter);
static void i2c_adapter_reset_bus(struct pios_i2c_adapter *i2c_adapter);

/*
 *
 *
 */
static void i2c_adapter_fsm_init(struct pios_i2c_adapter *i2c_adapter)
{
	// Init pins
	GPIO_Init(i2c_adapter->cfg->scl.gpio, &(i2c_adapter->cfg->scl.init));
	GPIO_Init(i2c_adapter->cfg->sda.gpio, &(i2c_adapter->cfg->sda.init));

	i2c_adapter_reset_bus(i2c_adapter);
	i2c_adapter->curr_state = I2C_STATE_STOPPED;

	// Reset the I2C block
	I2C_DeInit(i2c_adapter->cfg->regs);
	// Disable EVT and ERR interrupts - they are enabled by the first request
	I2C_ITConfig(i2c_adapter->cfg->regs, I2C_IT_EVT | I2C_IT_ERR, DISABLE);
	// Initialize the I2C block
	I2C_Init(i2c_adapter->cfg->regs, &(i2c_adapter->cfg->init));
	// Enable I2C peripheral
	I2C_Cmd(i2c_adapter->cfg->regs, ENABLE);
}

/*
 *
 *
 */
static void i2c_adapter_reset_bus(struct pios_i2c_adapter *i2c_adapter)
{
	GPIO_InitTypeDef scl_gpio_init;
	GPIO_InitTypeDef sda_gpio_init;
	uint8_t i;

	scl_gpio_init = i2c_adapter->cfg->scl.init;
	scl_gpio_init.GPIO_Mode = GPIO_Mode_Out_OD;
	GPIO_Init(i2c_adapter->cfg->scl.gpio, &scl_gpio_init);

	sda_gpio_init = i2c_adapter->cfg->sda.init;
	sda_gpio_init.GPIO_Mode = GPIO_Mode_Out_OD;
	GPIO_Init(i2c_adapter->cfg->sda.gpio, &sda_gpio_init);

	GPIO_SetBits(i2c_adapter->cfg->scl.gpio, i2c_adapter->cfg->scl.init.GPIO_Pin);
	GPIO_SetBits(i2c_adapter->cfg->sda.gpio, i2c_adapter->cfg->sda.init.GPIO_Pin);

	for (i = 0; i < 8; i++) {
		// Wait for any clock stretching to finish
		while (!GPIO_ReadInputDataBit(i2c_adapter->cfg->scl.gpio, i2c_adapter->cfg->scl.init.GPIO_Pin))
			PIOS_DELAY_WaituS(10);

		// Pull low
		GPIO_ResetBits(i2c_adapter->cfg->scl.gpio, i2c_adapter->cfg->scl.init.GPIO_Pin); //Set bus low
		PIOS_DELAY_WaituS(10);
		// Release high again
		GPIO_SetBits(i2c_adapter->cfg->scl.gpio, i2c_adapter->cfg->scl.init.GPIO_Pin); //Set bus high
		PIOS_DELAY_WaituS(10);
	}

	// Generate a start then stop condition
	GPIO_ResetBits(i2c_adapter->cfg->sda.gpio, i2c_adapter->cfg->sda.init.GPIO_Pin); // Set bus data low
	PIOS_DELAY_WaituS(10);
	GPIO_ResetBits(i2c_adapter->cfg->scl.gpio, i2c_adapter->cfg->scl.init.GPIO_Pin); // Set bus scl low
	PIOS_DELAY_WaituS(10);
	GPIO_SetBits(i2c_adapter->cfg->scl.gpio, i2c_adapter->cfg->scl.init.GPIO_Pin); // Set bus scl high
	PIOS_DELAY_WaituS(10);
	GPIO_SetBits(i2c_adapter->cfg->sda.gpio, i2c_adapter->cfg->sda.init.GPIO_Pin); // Set bus sda high

	// Init pins
	GPIO_Init(i2c_adapter->cfg->scl.gpio, &(i2c_adapter->cfg->scl.init));
	GPIO_Init(i2c_adapter->cfg->sda.gpio, &(i2c_adapter->cfg->sda.init));
}

/*
 *
 *
 */
bool i2cWrite(struct pios_i2c_adapter *i2c_adapter, uint8_t addr_, uint8_t reg_, uint8_t len_, uint8_t *data)
{
#ifdef USE_FREERTOS
	portTickType timeout = i2c_adapter->cfg->transfer_timeout_ms / portTICK_RATE_MS;
#else
	uint32_t timeout = I2C_DEFAULT_TIMEOUT;
#endif

	addr  = addr_ << 1;
	reg   = reg_;
	dir   = I2C_Direction_Transmitter;
	buf_p = data;
	bytes = len_;
	busy  = 1;
	error = false;

	if (!(i2c_adapter->cfg->regs->CR2 & I2C_IT_EVT)) {        //if we are restarting the driver
		if (!(i2c_adapter->cfg->regs->CR1 & 0x0100)) {        // ensure sending a start
			while (i2c_adapter->cfg->regs->CR1 & 0x0200) { ; }               //wait for any stop to finish sending
			I2C_GenerateSTART(i2c_adapter->cfg->regs, ENABLE);        //send the start for the new job
		}
		I2C_ITConfig(i2c_adapter->cfg->regs, I2C_IT_EVT | I2C_IT_ERR, ENABLE);        //allow the interrupts to fire off again
	}

#ifdef USE_FREERTOS
	if (xSemaphoreTake(i2c_adapter->sem_ready, timeout) == pdFALSE) {
		i2cErrorCount++;
		// reinit peripheral + clock out garbage
		i2c_adapter_fsm_init(i2c_adapter);
	}
#else
	while (busy && --timeout > 0);
	if (timeout == 0) {
		i2cErrorCount++;
		// reinit peripheral + clock out garbage
		i2c_adapter_fsm_init(i2c_adapter);
	}
#endif /* USE_FREERTOS */

	return !error;
}

/*
 *
 *
 */
bool i2cRead(struct pios_i2c_adapter *i2c_adapter, uint8_t addr_, uint8_t reg_, uint8_t len, uint8_t* buf)
{
#ifdef USE_FREERTOS
	portTickType timeout = i2c_adapter->cfg->transfer_timeout_ms / portTICK_RATE_MS;
#else
	uint32_t timeout = I2C_DEFAULT_TIMEOUT;
#endif

	addr  = addr_ << 1;
	reg   = reg_;
	dir   = I2C_Direction_Receiver;
	buf_p = buf;
	bytes = len;
	busy  = 1;
	error = false;

	if (!(i2c_adapter->cfg->regs->CR2 & I2C_IT_EVT)) {			//if we are restarting the driver
		if (!(i2c_adapter->cfg->regs->CR1 & 0x0100)) {			// ensure sending a start
			while (i2c_adapter->cfg->regs->CR1 & 0x0200) { ; }	//wait for any stop to finish sending
			I2C_GenerateSTART(i2c_adapter->cfg->regs, ENABLE);	//send the start for the new job
		}
		I2C_ITConfig(i2c_adapter->cfg->regs, I2C_IT_EVT | I2C_IT_ERR, ENABLE);	//allow the interrupts to fire off again
	}

#ifdef USE_FREERTOS
	if (xSemaphoreTake(i2c_adapter->sem_ready, timeout) == pdFALSE) {
		i2cErrorCount++;
		// reinit peripheral + clock out garbage
		i2c_adapter_fsm_init(i2c_adapter);
	}
#else
	while (busy && --timeout > 0);
	if (timeout == 0) {
		i2cErrorCount++;
		// reinit peripheral + clock out garbage
		i2c_adapter_fsm_init(i2c_adapter);
	}
#endif /* USE_FREERTOS */

	return !error;
}

/**
 * Logs the last N state transitions and N IRQ events due to
 * an error condition
 * \param[out] data address where to copy the pios_i2c_fault_history structure to
 * \param[out] counts three uint16 that receive the bad event, fsm, and error irq
 * counts
 */
void PIOS_I2C_GetDiagnostics(struct pios_i2c_fault_history * data, uint8_t * counts)
{
	struct pios_i2c_fault_history i2c_adapter_fault_history;
	i2c_adapter_fault_history.type = PIOS_I2C_ERROR_EVENT;

	memcpy(data, &i2c_adapter_fault_history, sizeof(i2c_adapter_fault_history));
	counts[0] = counts[1] = counts[2] = 0;
}

/*
 *
 *
 */
static bool PIOS_I2C_validate(struct pios_i2c_adapter * i2c_adapter)
{
	return (i2c_adapter->magic == PIOS_I2C_DEV_MAGIC);
}

/*
 *
 *
 */
static struct pios_i2c_adapter pios_i2c_adapters[PIOS_I2C_MAX_DEVS];
static uint8_t pios_i2c_num_adapters;
static struct pios_i2c_adapter * PIOS_I2C_alloc(void)
{
	struct pios_i2c_adapter * i2c_adapter;

	if (pios_i2c_num_adapters >= PIOS_I2C_MAX_DEVS) {
		return (NULL);
	}

	i2c_adapter = &pios_i2c_adapters[pios_i2c_num_adapters++];
	i2c_adapter->magic = PIOS_I2C_DEV_MAGIC;

	return (i2c_adapter);
}

/**
* Initializes IIC driver
* \param[in] mode currently only mode 0 supported
* \return < 0 if initialisation failed
*/
int32_t PIOS_I2C_Init(uint32_t * i2c_id, const struct pios_i2c_adapter_cfg * cfg)
{
	PIOS_DEBUG_Assert(i2c_id);
	PIOS_DEBUG_Assert(cfg);

	struct pios_i2c_adapter * i2c_adapter;

	i2c_adapter = (struct pios_i2c_adapter *) PIOS_I2C_alloc();
	if (!i2c_adapter) goto out_fail;

	/* Bind the configuration to the device instance */
	i2c_adapter->cfg = cfg;

#ifdef USE_FREERTOS
	/*
	 * Must be done prior to calling i2c_adapter_fsm_init()
	 * since the sem_ready mutex is used in the initial state.
	 */
	vSemaphoreCreateBinary(i2c_adapter->sem_ready);
	xSemaphoreTake(i2c_adapter->sem_ready, I2C_DEFAULT_TIMEOUT / portTICK_RATE_MS);
	i2c_adapter->sem_busy = xSemaphoreCreateMutex();
#else
	i2c_adapter->sem_ready = NULL;
	i2c_adapter->sem_busy = NULL;
#endif // USE_FREERTOS

	/* Enable the associated peripheral clock */
	switch ((uint32_t) i2c_adapter->cfg->regs) {
	case (uint32_t) I2C1:
		/* Enable I2C peripheral clock (APB1 == slow speed) */
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1, ENABLE);
		break;
	case (uint32_t) I2C2:
		/* Enable I2C peripheral clock (APB1 == slow speed) */
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C2, ENABLE);
		break;
	}

	if (i2c_adapter->cfg->remap) {
		GPIO_PinRemapConfig(i2c_adapter->cfg->remap, ENABLE);
	}

	/* Initialize the state machine */
	i2c_adapter_fsm_init(i2c_adapter);

	*i2c_id = (uint32_t)i2c_adapter;

	/* Configure and enable I2C interrupts */
	NVIC_Init(&(i2c_adapter->cfg->event.init));
	NVIC_Init(&(i2c_adapter->cfg->error.init));

	/* No error */
	return 0;

out_fail:
	return(-1);
}

/**
 * @brief Perform a series of I2C transactions
 * @returns 0 if success or error code
 * @retval -1 for failed transaction
 * @retval -2 for failure to get semaphore
 */
int32_t PIOS_I2C_Transfer(uint32_t i2c_id, const struct pios_i2c_txn txn_list[], uint32_t num_txns)
{
	struct pios_i2c_adapter * i2c_adapter = (struct pios_i2c_adapter *)i2c_id;

	bool valid = PIOS_I2C_validate(i2c_adapter);
	PIOS_Assert(valid)

	PIOS_DEBUG_Assert(txn_list);
	PIOS_DEBUG_Assert(num_txns);

	bool semaphore_success = true;

#ifdef USE_FREERTOS
	/* Lock the bus */
	portTickType timeout = i2c_adapter->cfg->transfer_timeout_ms / portTICK_RATE_MS;
	semaphore_success = (xSemaphoreTake(i2c_adapter->sem_busy, timeout) == pdTRUE);
	if(!semaphore_success)
		return -2;
#endif /* USE_FREERTOS */

	// Reading
	if (num_txns == 2) {
		i2c_adapter->bus_error = !i2cRead(
			i2c_adapter,        // adapter
			txn_list[0].addr,   // addr
			txn_list[0].buf[0], // reg
			txn_list[1].len,    // len
			txn_list[1].buf     // buf
		);
	}
	// Writing
	else if (num_txns == 1) {
		i2c_adapter->bus_error = !i2cWrite(
			i2c_adapter,          // adapter
			txn_list[0].addr,     // addr
			txn_list[0].buf[0],   // reg
			txn_list[0].len - 1,  // len
			&(txn_list[0].buf[1]) // buf
		);
	}
	// Error
	else {
		// do nothing
	}

#ifdef USE_FREERTOS
	/* Unlock the bus */
	xSemaphoreGive(i2c_adapter->sem_busy);
#endif /* USE_FREERTOS */

	return !semaphore_success ? -2 :
		i2c_adapter->bus_error ? -1 :
		0;
}

/*
 *
 *
 */
void PIOS_I2C_EV_IRQ_Handler(uint32_t i2c_id)
{
	struct pios_i2c_adapter * i2c_adapter = (struct pios_i2c_adapter *)i2c_id;

	bool valid = PIOS_I2C_validate(i2c_adapter);
	PIOS_Assert(valid);

	// flag to indicate if subaddess sent
	static uint8_t subaddress_sent;
	// flag to indicate final bus condition
	static uint8_t final_stop;
	// index is signed -1 == send the subaddress
	static int8_t index;

#ifdef USE_FREERTOS
	signed portBASE_TYPE pxHigherPriorityTaskWoken = pdFALSE;
#endif

	// I2C Status Registers
	volatile uint32_t SR1Reg;
	volatile uint32_t SR2Reg;

	SR1Reg = I2C_ReadRegister(i2c_adapter->cfg->regs, I2C_Register_SR1);

	//we just sent a start - EV5 in ref manual
	if (SR1Reg & I2C_FLAG_SB) {
		i2c_adapter->cfg->regs->CR1 &= ~I2C_CR1_FLAG_POS;      //reset the POS bit so ACK/NACK applied to the current byte
		I2C_AcknowledgeConfig(i2c_adapter->cfg->regs, ENABLE); //make sure ACK is on
		index = 0;              //reset the index
		if ((dir == I2C_Direction_Receiver) && subaddress_sent) {     //we have sent the subaddr
			if (bytes == 2) {
				i2c_adapter->cfg->regs->CR1 |= I2C_CR1_FLAG_POS; //set the POS bit so NACK applied to the final byte in the two byte read
			}
			I2C_Send7bitAddress(i2c_adapter->cfg->regs, addr, I2C_Direction_Receiver); //send the address and set hardware mode
		} else { //direction is Tx, or we havent sent the sub and rep start
			I2C_Send7bitAddress(i2c_adapter->cfg->regs, addr, I2C_Direction_Transmitter); //send the address and set hardware mode
			index = -1;    //send a subaddress
		}
	//we just sent the address - EV6, EV6_1 or EV6_3 in ref manual
	} else if (SR1Reg & I2C_FLAG_ADDR) {
		if ((bytes == 1) && (dir == I2C_Direction_Receiver) && subaddress_sent) {              //we are receiving 1 byte - EV6_3
			I2C_AcknowledgeConfig(i2c_adapter->cfg->regs, DISABLE);    //turn off ACK
			//Read SR2 to clear ADDR
			SR2Reg = I2C_ReadRegister(i2c_adapter->cfg->regs, I2C_Register_SR2);
			if (SR2Reg) { ; }
			I2C_GenerateSTOP(i2c_adapter->cfg->regs, ENABLE);          //program the stop
			I2C_ITConfig(i2c_adapter->cfg->regs, I2C_IT_BUF, ENABLE);  //allow us to have an EV7
			final_stop = 1;
		} else {
			//Read SR2 to clear ADDR
			SR2Reg = I2C_ReadRegister(i2c_adapter->cfg->regs, I2C_Register_SR2);
			if (SR2Reg) { ; }
			if ((bytes == 2) && (dir == I2C_Direction_Receiver) && subaddress_sent) {       //rx 2 bytes - EV6_1
				I2C_AcknowledgeConfig(i2c_adapter->cfg->regs, DISABLE);    //turn off ACK
				I2C_ITConfig(i2c_adapter->cfg->regs, I2C_IT_BUF, DISABLE); //disable TXE to allow the buffer to fill
			} else if ((bytes == 3) && (dir == I2C_Direction_Receiver) && subaddress_sent) {       //rx 3 bytes
				I2C_ITConfig(i2c_adapter->cfg->regs, I2C_IT_BUF, DISABLE); //make sure RXNE disabled so we get a BTF in two bytes time
			} else { //receiving greater than three bytes, sending subaddress, or transmitting
				I2C_ITConfig(i2c_adapter->cfg->regs, I2C_IT_BUF, ENABLE);
			}
		}
	//Byte transfer finished - EV7_2, EV7_3 or EV8_2
	} else if (SR1Reg & I2C_FLAG_BTF) {
		final_stop = 1;
		//EV7_2, EV7_3
		if ((dir == I2C_Direction_Receiver) && subaddress_sent) {
			if (bytes > 2) { //EV7_2
				I2C_AcknowledgeConfig(i2c_adapter->cfg->regs, DISABLE);   //turn off ACK
				buf_p[index++] = I2C_ReceiveData(i2c_adapter->cfg->regs); //read data N-2
				I2C_GenerateSTOP(i2c_adapter->cfg->regs, ENABLE);         //program the Stop
				buf_p[index++] = I2C_ReceiveData(i2c_adapter->cfg->regs); //read data N-1
				I2C_ITConfig(i2c_adapter->cfg->regs, I2C_IT_BUF, ENABLE); //enable TXE to allow the final EV7
			} else {         //EV7_3
				I2C_GenerateSTOP(i2c_adapter->cfg->regs, ENABLE);         //program the Stop
				buf_p[index++] = I2C_ReceiveData(i2c_adapter->cfg->regs); //read data N-1
				buf_p[index++] = I2C_ReceiveData(i2c_adapter->cfg->regs); //read data N
				index++;     //to show job completed
			}
		//EV8_2, which may be due to a subaddress sent or a write completion
		} else if ((dir == I2C_Direction_Transmitter) || subaddress_sent) {
			I2C_GenerateSTOP(i2c_adapter->cfg->regs, ENABLE);  //program the Stop
			index++;        //to show that the job is complete
		} else {            //We need to send a subaddress
			I2C_GenerateSTART(i2c_adapter->cfg->regs, ENABLE); //program the repeated Start
			subaddress_sent = 1; //this is set back to zero upon completion of the current task
		}
		//we must wait for the start to clear, otherwise we get constant BTF
		while (i2c_adapter->cfg->regs->CR1 & I2C_CR1_FLAG_START) { ; }
	} else if (SR1Reg & I2C_FLAG_RXNE) { //Byte received - EV7
		buf_p[index++] = I2C_ReceiveData(i2c_adapter->cfg->regs);
		if (bytes == (index + 3))
			I2C_ITConfig(i2c_adapter->cfg->regs, I2C_IT_BUF, DISABLE); //disable TXE to allow the buffer to flush so we can get an EV7_2
		if (bytes == index) //We have completed a final EV7
			index++;        //to show job is complete
	} else if (SR1Reg & I2C_FLAG_TXE) { //Byte transmitted -EV8/EV8_1
		if (index == -1) {  //we dont have a subaddress to send
			index++;
			I2C_SendData(i2c_adapter->cfg->regs, reg); //send the subaddress
			if ((dir == I2C_Direction_Receiver) || !bytes) //if receiving or sending 0 bytes, flush now
				I2C_ITConfig(i2c_adapter->cfg->regs, I2C_IT_BUF, DISABLE); //disable TXE to allow the buffer to flush
		} else {
			I2C_SendData(i2c_adapter->cfg->regs, buf_p[index++]);
			if (bytes == index) //we have sent all the data
				I2C_ITConfig(i2c_adapter->cfg->regs, I2C_IT_BUF, DISABLE); //disable TXE to allow the buffer to flush
		}
	}
	//we have completed the current job
	if (index == bytes + 1) {
		// Completion Tasks go here
		// End of completion tasks
		subaddress_sent = 0;	// reset this here
		// If there is a final stop and no more jobs, bus is inactive, disable interrupts to prevent BTF
		if (final_stop) {
			// Disable EVT and ERR interrupts while bus inactive
			I2C_ITConfig(i2c_adapter->cfg->regs, I2C_IT_EVT | I2C_IT_ERR, DISABLE);
			while(i2c_adapter->cfg->regs->CR1 & (I2C_CR1_FLAG_START | I2C_CR1_FLAG_STOP)) { ; }
		}
		busy = 0;
#if defined(USE_FREERTOS)
		if (xSemaphoreGiveFromISR(i2c_adapter->sem_ready, &pxHigherPriorityTaskWoken) != pdTRUE) {
#if defined(I2C_HALT_ON_ERRORS)
			PIOS_DEBUG_Assert(0);
#endif /* I2C_HALT_ON_ERRORS */
		}
		portEND_SWITCHING_ISR(pxHigherPriorityTaskWoken);	/* FIXME: is this the right place for this? */
#endif /* USE_FREERTOS */
	}
}

/*
 *
 *
 */
void PIOS_I2C_ER_IRQ_Handler(uint32_t i2c_id)
{
	struct pios_i2c_adapter * i2c_adapter = (struct pios_i2c_adapter *)i2c_id;

	bool valid = PIOS_I2C_validate(i2c_adapter);
	PIOS_Assert(valid);

	volatile uint32_t SR1Reg, SR2Reg;

#ifdef USE_FREERTOS
	signed portBASE_TYPE pxHigherPriorityTaskWoken = pdFALSE;
#endif

	// Read the I2C status register
	SR1Reg = I2C_ReadRegister(i2c_adapter->cfg->regs, I2C_Register_SR1);

	// an error
	if (SR1Reg & (I2C_FLAG_BERR | I2C_FLAG_ARLO | I2C_FLAG_AF | I2C_FLAG_OVR)) {
		error = true;
	}

	// If AF, BERR or ARLO, abandon the current job and commence new if there are jobs
	if (SR1Reg & (I2C_FLAG_BERR | I2C_FLAG_ARLO | I2C_FLAG_AF)) {
		// read second status register to clear ADDR if it is set (note that BTF will not be set after a NACK)
		SR2Reg = I2C_ReadRegister(i2c_adapter->cfg->regs, I2C_Register_SR2);
		if (SR2Reg) { ; }
		// disable the RXNE/TXE interrupt - prevent the ISR tailchaining onto the ER (hopefully)
		I2C_ITConfig(i2c_adapter->cfg->regs, I2C_IT_BUF, DISABLE);
		// if we dont have an ARLO error, ensure sending of a stop
		if (!(SR1Reg & I2C_FLAG_ARLO) && !(i2c_adapter->cfg->regs->CR1 & I2C_CR1_FLAG_STOP)) {
			// We are currently trying to send a start, this is very bad as start, stop will hang the peripheral
			if (i2c_adapter->cfg->regs->CR1 & I2C_CR1_FLAG_START) {
				// wait for any start to finish sending
				while (i2c_adapter->cfg->regs->CR1 & I2C_CR1_FLAG_START);
				// send stop to finalise bus transaction
				I2C_GenerateSTOP(i2c_adapter->cfg->regs, ENABLE);
				// wait for stop to finish sending
				while (i2c_adapter->cfg->regs->CR1 & I2C_CR1_FLAG_STOP);
				// reset and configure the hardware
				i2c_adapter_fsm_init(i2c_adapter);
			} else {
				// stop to free up the bus
				I2C_GenerateSTOP(i2c_adapter->cfg->regs, ENABLE);
				// Disable EVT and ERR interrupts while bus inactive
				I2C_ITConfig(i2c_adapter->cfg->regs, I2C_IT_EVT | I2C_IT_ERR, DISABLE);
			}
		}
	}

	//reset all the error bits to clear the interrupt
	I2C_ClearFlag(i2c_adapter->cfg->regs, I2C_FLAG_BERR | I2C_FLAG_ARLO | I2C_FLAG_AF | I2C_FLAG_OVR);
	busy = 0;
#if defined(USE_FREERTOS)
	if (xSemaphoreGiveFromISR(i2c_adapter->sem_ready, &pxHigherPriorityTaskWoken) != pdTRUE) {
#if defined(I2C_HALT_ON_ERRORS)
		PIOS_DEBUG_Assert(0);
#endif /* I2C_HALT_ON_ERRORS */
	}
	portEND_SWITCHING_ISR(pxHigherPriorityTaskWoken);	/* FIXME: is this the right place for this? */
#endif /* USE_FREERTOS */
}

#endif

/**
  * @}
  * @}
  */
