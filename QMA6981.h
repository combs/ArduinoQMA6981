/*
 * QMA6981.h
 *
 *  Created on: Jul 9, 2019
 *      Author: adam
 */

#ifndef QMA6981_H_
#define QMA6981_H_

#if ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif
#include <Wire.h>



/* Soft reset register value */

#define QMA6981_SOFT_RESET_ALL_REGISTERS 0xB6

#define QMA6981_ADDR 0x12


typedef struct
{
    int16_t x;
    int16_t y;
    int16_t z;
} accel_t;



/*============================================================================
 * Register addresses and definitions
 *==========================================================================*/

/* For the bandwidth register */

typedef enum
{
    QMA6981_BW_3_9  = 0b000,
    QMA6981_BW_7_8  = 0b001,
    QMA6981_BW_15_6 = 0b010,
    QMA6981_BW_31_2 = 0b011,
    QMA6981_BW_62_5 = 0b100,
    QMA6981_BW_125  = 0b101,
    QMA6981_BW_250  = 0b110,
    QMA6981_BW_500  = 0b111,
} QMA6981_BANDWIDTH;

typedef union
{
    uint8_t val;
    struct
    {
        QMA6981_BANDWIDTH BW : 5;
        bool ODRH            : 1;
    } bitmask;
} QMA6981_BW_VAL;

/* For the range register */

enum
{
    QMA6981_RANGE_2G  = 0x01,
    QMA6981_RANGE_4G  = 0x02,
    QMA6981_RANGE_8G  = 0x04,
};

/* For interrupt configuration register */

typedef union
{
    uint16_t val;
    struct
    {
        bool HIGH_EN_X         : 1;
        bool HIGH_EN_Y         : 1;
        bool HIGH_EN_Z         : 1;
        bool LOW_EN            : 1;
        bool DATA_EN           : 1;
        bool FFULL_EN          : 1;
        bool FWM_EN            : 1;
        bool res               : 2;
        bool STEP_UNSIMILAR_EN : 1;
        bool STEP_QUIT_EN      : 1;
        bool STEP_EN           : 1;
        bool D_TAP_EN          : 1;
        bool S_TAP_EN          : 1;
        bool ORIENT_EN         : 1;
        bool FOB_EN            : 1;
    } bitmask;
} QMA6981_INT_EN_VAL;

typedef union
{
    uint16_t val;
    struct
    {
        bool INT1_LVL          : 1;
        bool INT1_OD           : 1;
        bool INT2_LVL          : 1;
        bool INT2_OD           : 1;
        bool res               : 4;
    } bitmask;
} QMA6981_INT_PIN_CONF_VAL;

/* For the power level value register */

typedef enum
{
    Tpreset_12us   = 0b00,
    Tpreset_96us   = 0b01,
    Tpreset_768us  = 0b10,
    Tpreset_2048us = 0b11
} QMA6981_POWER_VAL_PRESET;

typedef enum
{
    SLEEP_DUR_FULL_SPEED = 0b0000,
    SLEEP_DUR_0_5ms      = 0b0001,
    SLEEP_DUR_1ms        = 0b0110,
    SLEEP_DUR_2ms        = 0b0111,
    SLEEP_DUR_4ms        = 0b1000,
    SLEEP_DUR_6ms        = 0b1001,
    SLEEP_DUR_10ms       = 0b1010,
    SLEEP_DUR_25ms       = 0b1011,
    SLEEP_DUR_50ms       = 0b1100,
    SLEEP_DUR_100ms      = 0b1101,
    SLEEP_DUR_500ms      = 0b1110,
    SLEEP_DUR_1000ms     = 0b1111,
} QMA6981_POWER_VAL_SLEEP_DUR;

typedef union
{
    uint8_t val;
    struct
    {
        QMA6981_POWER_VAL_SLEEP_DUR SLEEP_DUR : 4;
        QMA6981_POWER_VAL_PRESET PRESET       : 2;
        bool res                              : 1;
        bool MODE_BIT                         : 1;
    } bitmask;
} QMA6981_POWER_VAL;

/* Register addresses */

typedef enum
{
    QMA6981_CHIP_ID      = 0x00,
    QMA6981_DATA         = 0x01,
    QMA6981_STEP_CNT     = 0x07,
    QMA6981_INT_STATUS   = 0x0A,
    QMA6981_FIFO_STATUS  = 0x0E,
    QMA6981_FULL_SCALE   = 0x0F,
    QMA6981_BW           = 0x10,
    QMA6981_POWER_MODE   = 0x11,
    QMA6981_STEP_CONF    = 0x13,
    QMA6981_INT_EN1      = 0x16,
    QMA6981_INT_EN2      = 0x17,
    QMA6981_INT_SRC      = 0x18,
    QMA6981_INT_MAP0     = 0x19,
    QMA6981_INT_MAP1     = 0x1a,
    QMA6981_INT_MAP2     = 0x1b,
    QMA6981_INT_MAP3     = 0x1c,
    QMA6981_INT_PIN_CONF = 0x20,
    QMA6981_INT_LATCH    = 0x21,
    QMA6981_LowG_HighG   = 0x22,
    QMA6981_OS_CUST      = 0x27,
    QMA6981_TAP_1        = 0x2A,
    QMA6981_TAP_2        = 0x2B,
    QMA6981__4D_6D       = 0x2C,
    QMA6981_FIFO_WM      = 0x31,
    QMA6981_SelfTest     = 0x32,
    QMA6981_NVM_CFG      = 0x33,
    QMA6981_SOFT_RESET   = 0x36,
    QMA6981_IMAGE        = 0x37,
    QMA6981_FIFO_CONF    = 0x3E,
} QMA6981_reg_addr;

/*============================================================================
 * Function prototypes
 *==========================================================================*/

class QMA6981 {
    public:
        QMA6981();
        bool begin(uint8_t address = QMA6981_ADDR);
        uint8_t writeRegister(QMA6981_reg_addr addr, uint8_t data);
        uint8_t readRegister(QMA6981_reg_addr addr, uint8_t len, uint8_t* data);
        int16_t convertTwosComplement10bit(uint16_t in);
        accel_t* poll(accel_t* currentAccel);
        accel_t* poll();
        bool enableInterrupts(uint8_t interrupt_pin, QMA6981_INT_EN_VAL interrupt_value);
        bool setInterruptType(QMA6981_INT_PIN_CONF_VAL configuration);

    private:
        uint8_t _address = QMA6981_ADDR;
        QMA6981_INT_EN_VAL interrupt1;
        QMA6981_INT_EN_VAL interrupt2;

};

#endif /* QMA6981_H_ */
