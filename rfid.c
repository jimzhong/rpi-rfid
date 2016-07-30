#include <wiringPi.h>
#include <stdlib.h>
#include <string.h>

#include "rfid.h"
#include "bitspi.h"

void PCD_hard_reset()
{
    digitalWrite(PIN_RST, LOW);
    //hold RST down for 50 msecs
    delay(50);
    digitalWrite(PIN_RST, HIGH);
}

void PCD_soft_reset()
{
    PCD_WriteRegister(CommandReg, PCD_SoftReset);
    delay(50);
    while (PCD_ReadRegister(CommandReg) & (1<<4));
	// PCD still restarting - unlikely after waiting 50ms, but better safe than sorry.
}

uint8_t PCD_ReadRegister(uint8_t reg)
{
    uint8_t value;
    SPI_begin_transaction();
	SPI_transfer(0x80 | (reg & 0x7E));			// MSB == 1 is for reading. LSB is not used in address.
	value = SPI_transfer(0);					// Read the value back. Send 0 to stop reading.
	SPI_end_transaction();
	return value;
}


void PCD_WriteRegister(uint8_t reg, uint8_t value)
{
    SPI_begin_transaction();
	SPI_transfer(reg & 0x7E);			// MSB == 1 is for reading. LSB is not used in address.
    SPI_transfer(value);					// Read the value back. Send 0 to stop reading.
	SPI_end_transaction();
}

void PCD_SetRegisterBitMask(uint8_t reg, uint8_t mask)
{
	uint8_t tmp;
	tmp = PCD_ReadRegister(reg);
	PCD_WriteRegister(reg, tmp | mask);			// set bit mask
}

void PCD_ClearRegisterBitMask(uint8_t reg, uint8_t mask)
{
    uint8_t tmp;
    tmp = PCD_ReadRegister(reg);
    PCD_WriteRegister(reg, tmp & (~mask));		// clear bit mask
}

void PCD_AntennaOn()
{
    uint8_t value;
    value = PCD_ReadRegister(TxControlReg);
    if ((value & 0x03) != 0x03)
        PCD_WriteRegister(TxControlReg, value | 0x03);
}

void PCD_AntennaOff()
{
	PCD_ClearRegisterBitMask(TxControlReg, 0x03);
}

void PCD_init()
{
    pinMode(PIN_SS, OUTPUT);
    pinMode(PIN_RST, OUTPUT);
    //do not select the chip for now
    digitalWrite(PIN_SS, HIGH);
    PCD_hard_reset();

    //configure a timeout of 25ms
    PCD_WriteRegister(TModeReg, 0x80);
    PCD_WriteRegister(TPrescalerReg, 0xA9);
    PCD_WriteRegister(TReloadRegH, 0x03);
    PCD_WriteRegister(TReloadRegL, 0xE8);

    //force 100% ASK
    PCD_WriteRegister(TxASKReg, 0x40);
    //Set the preset value for the CRC coprocessor for the CalcCRC command to 0x6363
    PCD_WriteRegister(ModeReg, 0x3D);

    PCD_AntennaOn();
}

uint8_t PCD_version()
{
    return PCD_ReadRegister(VersionReg);
}
