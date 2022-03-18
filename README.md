# STM32F103C8T6 CAN

Circuit developed for a ADC -> CAN Converter.\
Controller used is STM32F103C8T6. \
8 controller channels configured as ADC...Range of signal is between 0-3.3 VDC. \
CAN protocol implemented is CAN open. \
\
\
CAN data sent in 8-byte Data Frame\
DATA_FRAME \

| BYTE7 | BYTE6 | BYTE5 | BYTE4 | BYTE3 | BYTE2 | BYTE1 | BYTE0 | \
| ADC_8 | ADC_7 | ADC_6 | ADC_5 | ADC_4 | ADC_3 | ADC_2 | ADC_1 | \
\
\
ADC VALUE 0-3.3V corresponds to 0x00-0xFF. \
Code has deveoped using STM utilities. \
