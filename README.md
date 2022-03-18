# STM32F103C8T6 CAN

Circuit developed for a ADC -> CAN Converter.\
Controller used is STM32F103C8T6. \
8 controller channels configured as ADC...Range of signal is between 0-3.3 VDC. \
CAN protocol implemented is CAN open. \
\
\
CAN data sent in 8-byte Data Frame\
DATA_FRAME \
 _______________________________________________________________________\
|        |        |        |        |        |        |        |        | \
| BYTE_7 | BYTE_6 | BYTE_5 | BYTE_4 | BYTE_3 | BYTE_2 | BYTE_1 | BYTE_0 | \
|--------|--------|--------|--------|--------|--------|--------|--------| \
|  ADC_8 |  ADC_7 |  ADC_6 |  ADC_5 |  ADC_4 |  ADC_3 |  ADC_2 |  ADC_1 | \
|________|________|________|________|________|________|________|________| \
\
\
ADC VALUE 0-3.3V corresponds to 0x00-0xFF. \
Code has deveoped using STM utilities. \
