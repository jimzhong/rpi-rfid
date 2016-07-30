#include <stdio.h>
#include <stdlib.h>

#include "rfid.h"
#include <wiringPi.h>

int main()
{
    uint8_t version;
    if (wiringPiSetupGpio() == -1)
    {
        printf("GPIO init failed.\n");
        exit(-1);
    }
    PCD_Init();
    version = PCD_Version();
    printf("Version: %x\n", version);
    PCD_Deinit();
    return 0;
}
