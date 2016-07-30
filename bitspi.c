#include <wiringPi.h>
#include "bitspi.h"

void SPI_init(void)
{
    pinMode(PIN_SS, OUTPUT);
    pinMode(PIN_MISO, INPUT);
    pinMode(PIN_MOSI, OUTPUT);
    pinMode(PIN_CLK, OUTPUT);

    digitalWrite(PIN_CLK, LOW);
    digitalWrite(PIN_SS, HIGH);
    digitalWrite(PIN_MOSI, LOW);
}


void SPI_deinit(void)
{
    pinMode(PIN_SS, INPUT);
    pinMode(PIN_MISO, INPUT);
    pinMode(PIN_MOSI, INPUT);
    pinMode(PIN_CLK, INPUT);
}

void SPI_begin_transaction()
{
    delayMicroseconds(4);
    digitalWrite(PIN_SS, LOW);
    delayMicroseconds(4);
}

void SPI_end_transaction()
{
    delayMicroseconds(4);
    digitalWrite(PIN_SS, HIGH);
    delayMicroseconds(4);
}

/*
Transfer a byte to SPI with CPOH=CPOA=0
MSB first, return data on MISO
*/
uint8_t SPI_transfer(uint8_t value_out)
{
    uint8_t mask;
    uint8_t value_in;
    for (mask = 0x80; mask; mask >>= 1)
    {
        digitalWrite(PIN_MOSI, (value_out & mask) ? HIGH : LOW);
        digitalWrite(PIN_CLK, HIGH);
        delayMicroseconds(2);

        if (digitalRead(PIN_MISO))
            value_in |= mask;

        digitalWrite(PIN_CLK, LOW);
        delayMicroseconds(2);
    }
    return value_in;
}
