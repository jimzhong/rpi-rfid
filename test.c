#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "rfid.h"
#include <wiringPi.h>

void print_hex(uint8_t *buf, uint8_t len)
{
    while (len--)
        printf("%2x ", *(buf++));
    printf("\n");
}

int main()
{
    uint8_t version;
    Uid current;
    MIFARE_Key key = {};

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
        if (PICC_IsNewCardPresent() && (PICC_Select(&current, 0) == STATUS_OK))
        {
            printf("new card: %lx.\n", *(unsigned long *)current.uidByte);
            break;
        }
        else
        {
            printf("no new card.\n");
        }
        delay(200);
    }

    PICC_DumpMifareClassic(&current, PICC_GetType(current.sak), &key);

    PCD_Deinit();
    return 0;
}
