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
    Uid previous, current;
    if (wiringPiSetupGpio() == -1)
    {
        printf("GPIO init failed.\n");
        exit(-1);
    }
    PCD_Init();
    version = PCD_Version();
    printf("Version: %x\n", version);

    memset(&previous, 0, sizeof(previous));

    while (1)
    {
        if (PICC_IsNewCardPresent() && (PICC_Select(&current, 0) == STATUS_OK))
        {
            if (memcmp(previous.uidByte, current.uidByte, current.size) != 0)
            {
                printf("new card: %lx.\n", *(unsigned long *)current.uidByte);
                previous = current;
            }
        }
        else
        {
            printf("no new card.\n");
        }
        delay(100);
    }

    PCD_Deinit();
    return 0;
}
