/*
 * main.h
 *
 *  Created on: Nov 22, 2011
 *      Author: Jerry
 */

#ifndef MAIN_H_
#define MAIN_H_

#define USART_RX_DATA_SIZE   2048

typedef struct
{
  uint32_t bitrate;
  uint8_t format;
  uint8_t paritytype;
  uint8_t datatype;
}LINE_CODING;

void SERIAL_Init(void);

void USB_To_USART_Send_Data(uint8_t* data_buffer, uint8_t Nb_bytes);
void USART_To_USB_Send_Data(void);
//void Handle_USBAsynchXfer (void);




#endif /* MAIN_H_ */
