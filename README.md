# SAMD20_Slave_I2C
Uses the SAMD20 Xplained Pro as a slave I2C device in part 2 of the AT11628 app note, to the SAMD21 Xplained pro Master I2C device.

__Refer to the SAMD21_Master_I2C First before building the SAMD20_Slave_I2C as many features carry over.__

## Documents:
[AT11628 App Note](https://www.avrfreaks.net/sites/default/files/forum_attachments/Atmel-42631-SAM-D21-SERCOM-I2C-Configura.pdf)

[DS60001504B Datasheet](http://ww1.microchip.com/downloads/en/DeviceDoc/60001504B.pdf)

## Create New Project
Starting with the Demo Board, ideally port to Userboard
1. New GCC C ASF Board Project
2. Choose SAMD20J18 -> Xplained Pro ATSAMD20J18 and verify LED Toggle with SW0 is working.

## Overview
* Master will transmit a data buffer of a few bytes to the slave.
* Slave will re-transmit the same data buffer to the master.
* Using SERCOM2 (PA08 = SDA, PA09 = SCL).  These I2C lines are available on all 3 EXT headers on the Xplained Pro board, can use any.

## Hardware
* SAMD20 Xplained pro has on-board 4.7k pullup resistors on I2C lines (R305, R306) (Left of PB02 on EXT1) which have been replaced with 2k pullup resistors to improve signal timing. 
<a href="https://www.codecogs.com/eqnedit.php?latex=Rp_{max}&space;=&space;[t_{r}&space;*&space;(0.8473&space;*&space;C_{bus})]" target="_blank"><img src="https://latex.codecogs.com/gif.latex?Rp_{max}&space;=&space;[t_{r}&space;*&space;(0.8473&space;*&space;C_{bus})]" title="Rp_{max} = [t_{r} * (0.8473 * C_{bus})]" /></a>

For the SAMD20J18A:  worst case (Fast Mode) tr = 100ns, Cb = 400pF, 

## Clocks
Master and slave application uses OSC8M as the clock source for Generator 0 (Undivided = 8MHz)
fGCLK_SERCOM2_CORE (SERCOM2 input clock frequency) max = 48MHz
* Clock Init is the same as the MASTER code.

## I2C Pin and Register Init (I2C_Master_Init)
* pin_init and peripheral_function are the same as MASTER 

## I2C Slave Init
One difference between the SAMD20 and the SAMD21 is that the SAMD21 supports Fast Mode I2C.  SAMD20 supports _Standard_ and _Fast Mode_ frequencies (100khz and 400khz).  SO we will have to make the SAMD21 match the SAMD20.
__The I2C standard Fm+ (Fast-mode plus) requires a nominal high to low SCL ratio of 1:2, and BAUD should be set accordingly. At a minimum, BAUD.BAUD and/or BAUD.BAUDLOW must be nonzero.__

# Differences from App Note to be addressed:
1. i2c_slave_init: No SERCOM_I2CS_CTRLA_SPEED.  Leave out.
2. BAUD and BAUDLOW ???
3. SYNCBUSY is undefined: 
4. Some of the SERCOM flags are allegedly read only.
