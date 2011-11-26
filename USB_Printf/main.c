/******************** (C) COPYRIGHT 2011 STMicroelectronics ********************
* File Name          : main.c
* Author             : MCD Application Team
* Version            : V3.3.0
* Date               : 21-March-2011
* Description        : Virtual Com Port Demo main file
********************************************************************************
* THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
* WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE TIME.
* AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
* INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
* CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
* INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
*******************************************************************************/

/* Includes ------------------------------------------------------------------*/
//Library config for this project!!!!!!!!!!!
#include "stm32f4_discovery.h"

#include "stm32f4xx_conf.h"
 
#include "usbd_cdc_core.h"
#include "usbd_usr.h"
#include "usbd_desc.h"

#include "stm32f4xx_usart.h"
#include "main.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
#ifdef USB_OTG_HS_INTERNAL_DMA_ENABLED
  #if defined ( __ICCARM__ ) /*!< IAR Compiler */
    #pragma data_alignment = 4
  #endif
#endif /* USB_OTG_HS_INTERNAL_DMA_ENABLED */


/* USB CDC Variables */
__ALIGN_BEGIN USB_OTG_CORE_HANDLE  USB_OTG_dev __ALIGN_END;
extern uint8_t  APP_Rx_Buffer [];
extern uint32_t APP_Rx_ptr_in;

/* Private function prototypes -----------------------------------------------*/
void SERIAL_Init(void);
int OutByte(int ch);
int uartPutch(int ch);
int uartGetch(void);

/*******************************************************************************
* Function Name  : main.
* Description    : Main routine.
* Input          : None.
* Output         : None.
* Return         : None.
*******************************************************************************/
int main(void)
{
	  SERIAL_Init();

	  USBD_Init(&USB_OTG_dev,
		  USB_OTG_FS_CORE_ID,
	      &USR_desc,
	      &USBD_CDC_cb,
	      &USR_cb);

	  while (1)  {
		  uartGetch();
		  //printf over serial port
		  TestPrintf();
		  //printf over usb CDC
		  uTestPrintf();
	  }
}
/*******************************************************************************
* Function Name  :  USART_Config_Default.
* Description    :  configure the EVAL_COM1 with default values.
* Input          :  None.
* Return         :  None.
*******************************************************************************/
void SERIAL_Init(void)
{
	USART_InitTypeDef USART_InitStructure;
	/* USARTx configured as follow:
	      - BaudRate = 115200 baud
	      - Word Length = 8 Bits
	      - One Stop Bit
	      - No parity
	      - Hardware flow control disabled (RTS and CTS signals)
	      - Receive and transmit enabled
	 */
	USART_InitStructure.USART_BaudRate = 115200;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	STM_EVAL_COMInit(COM1, &USART_InitStructure);
}

/**
  * @brief  Outputs a char to the USART.
  * @param  char
  * @retval char
  */
int UsbOutByte(int ch) {
	APP_Rx_Buffer[APP_Rx_ptr_in]=ch;
	APP_Rx_ptr_in++;
	if(APP_Rx_ptr_in == APP_RX_DATA_SIZE) APP_Rx_ptr_in = 0;
}

/**
  * @brief  Outputs a char to the USART.
  * @param  char
  * @retval char
  */
int uartPutch(int ch) {
	USART_SendData(EVAL_COM1, (uint8_t) ch);
	/* Loop until the end of transmission */
	while (USART_GetFlagStatus(EVAL_COM1, USART_FLAG_TC) == RESET) {}
}

/**
  * @brief  Waits for then gets a char from the USART.
  * @param  none
  * @retval char
  */
int uartGetch() {
	int ch;
	while (USART_GetFlagStatus(EVAL_COM1, USART_FLAG_RXNE) == RESET) {}
	ch=USART_ReceiveData(EVAL_COM1);
	//uartPutch(ch);
	return ch;
}

#ifdef USE_FULL_ASSERT
/*******************************************************************************
* Function Name  : assert_failed
* Description    : Reports the name of the source file and the source line number
*                  where the assert_param error has occurred.
* Input          : - file: pointer to the source file name
*                  - line: assert_param error line source number
* Output         : None
* Return         : None
*******************************************************************************/
void assert_failed(uint8_t* file, uint32_t line)
{
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {}
}
#endif

/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
