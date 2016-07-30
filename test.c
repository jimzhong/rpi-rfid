#include <stdio.h>
#include <stdlib.h>

#include "rfid.h"
#include <wiringPi.h>

void print_hex(uint8_t *buf, uint8_t len)
{
    while (len--)
        printf("%2x ", *(buf++));
    printf("\n");
}

void print_uid(Uid uid)
{
    printf("%lx\n", (*((uint32_t *)uid.uidByte)));
}

int main()
{
    uint8_t version;
    Uid card;
    if (wiringPiSetupGpio() == -1)
    {
        printf("GPIO init failed.\n");
        exit(-1);
    }
    PCD_Init();
    version = PCD_Version();
    printf("Version: %x\n", version);

    while (1)
    {
        if (PICC_IsNewCardPresent() && (PICC_Select(&card, 0) == STATUS_OK))
        {
            printf("newcard: %lx\n", (*((unsigned long *)uid.uidByte)));
        }
        delay(100);
    }

    PCD_Deinit();
    return 0;
}
