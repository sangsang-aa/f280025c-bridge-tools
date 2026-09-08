#include "modbus_rtu.h"
#include <stdio.h>

int main(void)
{
    ModbusSlave s;
    ModbusSlave_init(&s, 1);
    int i;
    for (i = 0; i < 100; i++) ModbusSlave_pushWaveSample(&s, (float)i);
    printf("batch=%u status=%u\n", (unsigned)s.waveBatch, (unsigned)s.waveStatus);
    printf("sample0 hi=%04X lo=%04X (0.0f)\n",
           ModbusSlave_getWaveReg(&s, 0), ModbusSlave_getWaveReg(&s, 1));
    printf("sample1 hi=%04X lo=%04X (1.0f)\n",
           ModbusSlave_getWaveReg(&s, 2), ModbusSlave_getWaveReg(&s, 3));
    printf("sample99 hi=%04X (99.0f)\n", ModbusSlave_getWaveReg(&s, 198));
    for (i = 0; i < 100; i++) ModbusSlave_pushWaveSample(&s, (float)(100 + i));
    printf("batch=%u status=%u\n", (unsigned)s.waveBatch, (unsigned)s.waveStatus);
    printf("next sample0 hi=%04X lo=%04X (100.0f)\n",
           ModbusSlave_getWaveReg(&s, 0), ModbusSlave_getWaveReg(&s, 1));
    return 0;
}
