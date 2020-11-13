/*
 * QMA6981.c
 *
 *  Created on: Jul 9, 2019
 *      Author: adam
 *  Hacked up for Arduino on: Sep 29, 2020
 *      Author: ccombs
 */

// Datasheet at
// https://datasheet.lcsc.com/szlcsc/QST-QMA6981_C310611.pdf
// Some values taken from
// https://github.com/yangzhiqiang723/rainbow-RB59M325ALB/blob/02ea6fc2a7f9744273b850cff751ffd2fcf1820b/src/QMA6981.c
// https://github.com/yangzhiqiang723/rainbow-RB59M325ALB/blob/02ea6fc2a7f9744273b850cff751ffd2fcf1820b/inc/QMA6981.h


#include "QMA6981.h"
#include <Arduino.h>
#include <Wire.h>


/*============================================================================
 * Variables
 *==========================================================================*/

accel_t lastKnownAccel = {0};

/*============================================================================
 * Functions
 *==========================================================================*/
/**
 * @brief Constructor
 * 
 * 
*/

void QMA6981() 
{
    interrupt1.value = 0;
    interrupt2.value = 0;
}
/**
 * @brief Write a single byte of data to the given QMA6981 register
 *
 * @param addr The address to write to
 * @param data The single byte to write
 * @return true if the data was read, false if there was an i2c error
 */
uint8_t QMA6981::writeRegister(QMA6981_reg_addr addr, uint8_t data)
{
    Wire.beginTransmission(QMA6981_ADDR);
    uint8_t writeCmd[2] = {addr, data};
    Wire.write(writeCmd, sizeof(writeCmd));
    return Wire.endTransmission();
}

/**
 * @brief TODO Read a number of bytes from the given QMA6981 register
 *
 * @param addr The address to read from
 * @param len  The number of bytes to read
 * @param data A pointer to read the bytes into
 * @return true if the data was read, false if there was an i2c error
 */
uint8_t QMA6981::readRegister(QMA6981_reg_addr addr, uint8_t len, uint8_t* data)
{
    Wire.beginTransmission(QMA6981_ADDR);
    uint8_t reg[1] = {addr};
    Wire.write(reg, sizeof(reg));
    Wire.endTransmission();
    Wire.requestFrom(QMA6981_ADDR, len);

    uint8_t i = 0;
    while(Wire.available())    // slave may send less than requested
    {
        data[i] = Wire.read();
        i++;
    }
    return 0;
}

/**
 * @brief Initialize the QMA6981 and start it going
 *
 * @return true if initialization succeeded, false if it failed
 */
bool QMA6981::begin(uint8_t address)
{
    _address = address;

    uint8_t chip_id;
    if(0 != readRegister(QMA6981_CHIP_ID, 1, chip_id)) {
        // read failure;
        return false;
    }
    if ((chip_id < 0xB0) || (chip_id > 0xBF)) {
        // did not return proper chip ID
        return false;
    }

    QMA6981_POWER_VAL active =
    {
        .bitmask.MODE_BIT = true,
        .bitmask.res = true,
        .bitmask.SLEEP_DUR = SLEEP_DUR_FULL_SPEED,
        .bitmask.PRESET = Tpreset_12us
    };
    if(0 != writeRegister(QMA6981_POWER_MODE, active.val))
    {
        return false;
    }

    if(0 != writeRegister(QMA6981_SOFT_RESET, QMA6981_SOFT_RESET_ALL_REGISTERS))
    {
        return false;
    }
    if(0 != writeRegister(QMA6981_POWER_MODE, active.val))
    {
        return false;
    }
    delayMicroseconds(5);

    QMA6981_BW_VAL bandwidth =
    {
        .bitmask.ODRH = false,
        .bitmask.BW = QMA6981_BW_31_2
    };
    if(0 != writeRegister(QMA6981_BW, bandwidth.val))
    {
        return false;
    }

    if(0 != writeRegister(QMA6981_FULL_SCALE, QMA6981_RANGE_2G))
    {
        return false;
    }

    if(0 != writeRegister(QMA6981_POWER_MODE, active.val))
    {
        return false;
    }

    return true;
}

/**
 * @brief Poll the QMA6981 for the current acceleration value
 *
 * @param currentAccel A pointer where the acceleration data will be stored
 */
void QMA6981::poll(accel_t* currentAccel)
{
    // Read 7 bytes of data(0x00)
    uint8_t raw_data[6];
    if(0 != readRegister(QMA6981_DATA, 6, raw_data))
    {
        // os_printf("read xyz error!!!\n");
        // Try reinitializing, then return last known value
        QMA6981_setup();
    }
    else
    {
        // Convert the data to 12-bits, save it as the last known value
        lastKnownAccel.x = convertTwosComplement10bit(((raw_data[0] >> 6 ) | (raw_data[1]) << 2) & 0x03FF);
        lastKnownAccel.y = convertTwosComplement10bit(((raw_data[2] >> 6 ) | (raw_data[3]) << 2) & 0x03FF);
        lastKnownAccel.z = convertTwosComplement10bit(((raw_data[4] >> 6 ) | (raw_data[5]) << 2) & 0x03FF);
    }

    // Copy out the acceleration value
    currentAccel->x = lastKnownAccel.x;
    currentAccel->y = lastKnownAccel.y;
    currentAccel->z = lastKnownAccel.z;
    return currentAccel;
}

void QMA6981::poll()
{
    accel_t currentAccel;
    return poll(currentAccel);
}

bool QMA6981::enableIinterrupts(uint8_t interrupt_pin, QMA6981_INT_EN_VAL interrupt_value)
{
    if (interrupt_pin == 1) {
        interrupt1.value = interrupt_value.value;

        if(0 != writeRegister(QMA6981_INT_MAP + 1, interrupt_value.value >> 8))
        {
            return false;
        }
        if(0 != writeRegister(QMA6981_INT_MAP, combined & 255))
        {
            return false;
        }
    } else if (interrupt_pin == 2)
    {
        interrupt2.value = interrupt_value.value;

        if(0 != writeRegister(QMA6981_INT_MAP + 3, interrupt_value.value >> 8))
        {
            return false;
        }
        if(0 != writeRegister(QMA6981_INT_MAP + 2, combined & 255))
        {
            return false;
        }
    } 
    

    uint16_t combined = interrupt1.value | interrupt2.value; 

    if(0 != writeRegister(QMA6981_INT_EN + 1, combined >> 8))
    {
        return false;
    }
    if(0 != writeRegister(QMA6981_INT_EN, combined & 255))
    {
        return false;
    } 
    return true;
}

bool QMA6981::setInterruptType(QMA6981_INT_PIN_CONF_VAL configuration)
{
    if (0 != writeRegister(QMA6981_INT_PIN_CONF, configuration.value))
    {
        return false;
    }
    return true;
}



/**
 * @brief Helper function to convert a 10 bit 2's complement number to 16 bit
 *
 * @param in
 * @return int16_t convertTwosComplement10bit
 */
int16_t QMA6981::convertTwosComplement10bit(uint16_t in)
{
    if(in & 0x200)
    {
        return (in | 0xFC00); // extend the sign bit all the way out
    }
    else
    {
        return (in & 0x01FF); // make sure the sign bits are cleared
    }
}


