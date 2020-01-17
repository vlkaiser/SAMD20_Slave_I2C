/***************************************************************************************************************************
* Project							: SAMD20 I2C Slave
* Program Name						:
* Author							: vkaiser
* Date Created						: Jan 16 2020
*
* Purpose							: Implement I2C between SAMD20/SAMD21 MCUs; One as SL, one as MS per AT11628
*
*
* MCU								: ATSAMD20J18
* Language							: C
* Hardware Modifications			: N/A
* Debugger							: EDBG (On-board)
*
* Repo / Revision History			: https://github.com/vlkaiser/
*
* - Special Setup -
*  Header files for all drivers that have been imported from Atmel Software Framework (ASF).
*  Use in conjunction with			: SAMD20 Xplained Pro
*  Wiring Details					: R305, R306 replaced with 1.82Kohm
*
* Revision History					:
* 	Date				Author			Notes
* 						vkaiser			- Initial commit
*
***************************************************************************************************************************/
#include <asf.h>

#define STANDARD_MODE_FAST_MODE	0x0		// I2C Speed Mode Standard
#define FAST_MODE_PLUS	0x01			// I2C SPEED bit field
#define HIGHSPEED_MODE	0x02			// I2C SPEED bit field
#define SLAVE_ADDR	0x12				// SLAVE device Address

#define BUF_SIZE	3					// TX/RX Buffer Size


/* GLOBALS */
uint8_t i;
volatile bool tx_done = false, rx_done = false;
uint8_t tx_buf[BUF_SIZE] = {1, 2, 3};
uint8_t rx_buf[BUF_SIZE];


/* Prototypes */
void i2c_clock_init(void);
void i2c_pin_init(void);
void i2c_slave_init(void);

static void pin_set_peripheral_function(uint32_t pinmux);



/******************************************************************************************************
 * @fn					- i2c_clock_init
 * @brief				- Configure peripheral bus clock (APB) and generic clock for i2c SERCOM module
 * @param[in]			- void
 * @return				- void
 *
 * @note				- 
 ******************************************************************************************************/
void i2c_clock_init()
{
	struct system_gclk_chan_config gclk_chan_conf;		//struct to configure generic clock for SERCOM
	uint32_t gclk_index = SERCOM2_GCLK_ID_CORE;

	system_apb_clock_set_mask(SYSTEM_CLOCK_APB_APBC, PM_APBCMASK_SERCOM2);	//Turn on module in Power Manager - peripheral bus C
	system_gclk_chan_get_config_defaults((&gclk_chan_conf));				//Turn on generic clock for i2c: Default is generator0
	system_gclk_chan_set_config(gclk_index, &gclk_chan_conf);				//Write defaults to SERCOM2
	system_gclk_chan_enable(gclk_index);									//Enable
}

/******************************************************************************************************
 * @fn					- pin_set_peripheral_function
 * @brief				- Initialize i2c pins to SERCOM-Alternate peripheral function (D)
 * @param[in]			- pinmux (MCU driver files for pin definitions)
 * @return				- void
 *
 * @note				- Assign I/O lines PA08 and PA09 to the SERCOM peripheral function.
 *						- Will switch the GPIO functionality of an I/O pin to peripheral
 *							 functionality and assigns the given peripheral function to the pin.
 ******************************************************************************************************/
static void pin_set_peripheral_function(uint32_t pinmux)
{
	uint8_t port = (uint8_t)((pinmux >> 16)/32);
	PORT->Group[port].PINCFG[((pinmux >> 16) - (port*32))].bit.PMUXEN = 1;
	PORT->Group[port].PMUX[((pinmux >> 16) - (port*32))/2].reg &= ~(0xF << (4 * ((pinmux >> 16) & 0x01u)));
	PORT->Group[port].PMUX[((pinmux >> 16) - (port*32))/2].reg |= (uint8_t)((pinmux &0x0000FFFF) << (4 * ((pinmux >> 16) & 0x01u)));

}

/******************************************************************************************************
 * @fn					- i2c_pin_init
 * @brief				- Initialize i2c pins to SERCOM-Alternate peripheral function (D)
 * @param[in]			- void
 * @return				- void
 *
 * @note				- PA08 = SDA, PA09 = SCL
 ******************************************************************************************************/
void i2c_pin_init()
{
	pin_set_peripheral_function(PINMUX_PA08D_SERCOM2_PAD0);	
	pin_set_peripheral_function(PINMUX_PA09D_SERCOM2_PAD1);
}

/******************************************************************************************************
 * @fn					- i2c_slave_init
 * @brief				- initialize the I2C slave functions
 * @param[in]			- void
 * @return				- void
 *
 * @note				- Configures control registers, baud registers, sets respective interrupt enable bits.
 *						
 ******************************************************************************************************/
void i2c_slave_init()
{
	/* Configurations while I2C is DISABLED: */

	/* Configure SERCOM_I2CM hardware register CTRLA:
	*	- SDAHOLD bit field as 0x02, SDA hold time is configured for 300-600ns
	*	- RUNSTDBY bit as 0x01, Generic clock is enabled in all sleep modes (any interrupt can wake up the device)
	*	- MODE bitfield to 0x4, SERCOM2 is configured as I2C Slave */

		SERCOM2->I2CS.CTRLA.reg = SERCOM_I2CS_CTRLA_SDAHOLD(0x2)	|
		SERCOM_I2CS_CTRLA_RUNSTDBY									|
		SERCOM_I2CS_CTRLA_MODE_I2C_SLAVE;

	/* Enable Smart Mode - Will ACK when DATA.DATA is read*/
	SERCOM2->I2CS.CTRLB.reg = SERCOM_I2CS_CTRLB_SMEN;

	/* Write the slave address into ADDR register */
	SERCOM2->I2CS.ADDR.reg = SLAVE_ADDR << 1 ;

	/* Address match interrupt; Data ready interrupt; stop received interrupts are enabled */
	SERCOM2->I2CS.INTENSET.reg = SERCOM_I2CS_INTENSET_PREC | SERCOM_I2CS_INTENSET_AMATCH | SERCOM_I2CS_INTENSET_DRDY;

	/* SERCOM2 peripheral enabled by setting the ENABLE bit as 1*/
	SERCOM2->I2CS.CTRLA.reg |= SERCOM_I2CS_CTRLA_ENABLE;

	/* SERCOM enable synchronization busy */
	while(SERCOM2->I2CS.STATUS.bit.SYNCBUSY  & SERCOM_I2CM_STATUS_SYNCBUSY);	//Todo: TEST

	/* SERCOM2 handler enabled */
	system_interrupt_enable(SERCOM2_IRQn);

}

/******************************************************************************************************
 * @fn					- SERCOM2_Handler
 * @brief				- After transmitting address to slave and receiving ACK/NACK
 * @param[in]			- void
 * @return				- void
 *
 * @note				- Interrupt handler during while(tx_done == true) in master_transaction
 *						- Overrides weak definition
 ******************************************************************************************************/
void SERCOM2_Handler(void)
{
	/* Check for Address match interrupt */
	if(SERCOM2->I2CS.INTFLAG.bit.AMATCH)
	{
		/* clearing the Address match interrupt */
		SERCOM2->I2CS.INTFLAG.bit.AMATCH = 1;
	}
	/* Data Ready interrupt check */
	//ToDo: Errata DS80000747B writing CTRLB in DRDY or AMATCH interrupts
	if(SERCOM2->I2CS.INTFLAG.bit.DRDY)
	{
		/* Checking for direction,
		DIR - 0 for slave read,
		DIR - 1 for slave write */
		if (SERCOM2->I2CS.STATUS.bit.DIR)
		{
			/* Slave write */
			if (i == (BUF_SIZE-1))
			{
				SERCOM2->I2CS.DATA.reg = rx_buf[i++];
				/* wait for stop condition */
				SERCOM2->I2CS.CTRLB.bit.CMD = 0x2;
				i = 0;
			}
			else
			{
				SERCOM2->I2CS.DATA.reg = rx_buf[i++];
				/* Execute a byte read operation followed by ACK/NACK reception by master*/
				SERCOM2->I2CS.CTRLB.bit.CMD = 0x3;
			}
		}
		else
		{
			/* Slave read */
			if (i == (BUF_SIZE-1))
			{
				SERCOM2->I2CS.CTRLB.bit.ACKACT = 0;
				/* Execute acknowledge action succeeded by waiting for any start (S/Sr) condition */
				SERCOM2->I2CS.CTRLB.bit.CMD = 0x2;
			}
			else
			{
				rx_buf[i++] = SERCOM2->I2CS.DATA.reg;
				SERCOM2->I2CS.CTRLB.bit.ACKACT = 0;
				/* Execute acknowledge action succeeded by reception of next byte to master*/
				SERCOM2->I2CS.CTRLB.bit.CMD = 0x3;
			}
		}
	}
	if (SERCOM2->I2CS.INTFLAG.bit.PREC)
	{
		SERCOM2->I2CS.INTFLAG.bit.PREC = 1;
		if (!SERCOM2->I2CS.STATUS.bit.DIR)
		{
			rx_buf[i++] = SERCOM2->I2CS.DATA.reg;
		}
		i = 0;
	}
}



/******************************************************************************************************
 * @fn					- MAIN
 * @brief				- 
 * @param[in]			- void
 * @return				- void
 *
 * @note				- 
 ******************************************************************************************************/
int main (void)
{
	system_init();
	i2c_clock_init();
	i2c_pin_init();
	i2c_slave_init();

	

	/* This skeleton code simply sets the LED to the state of the button. */
	while (1) {
		/* Is button pressed? */
		if (port_pin_get_input_level(BUTTON_0_PIN) == BUTTON_0_ACTIVE) {
			/* Yes, so turn LED on. */
			port_pin_set_output_level(LED_0_PIN, LED_0_ACTIVE);
		} else {
			/* No, so turn LED off. */
			port_pin_set_output_level(LED_0_PIN, !LED_0_ACTIVE);
		}
	}
}
