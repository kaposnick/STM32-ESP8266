#include "stm32l1xx.h"
#include "string.h"
#include "stdlib.h"

#define baudrate 115200

void uart_init();
void gpio_init();
void delay(int us);
void uart_send(char* msg);
void esp_send(char* msg);

typedef struct {
  uint8_t linkId;
  int dataBufferLength;
  char dataBuffer[5];
} tcp_session;



// BASIC AT COMMAND SET
char reset[] = "AT+RST\r\n";
char working[] = "AT\r\n";
char disable_echo[] = "ATE0\r\n";
char firmware_version[] = "AT+GMR\r\n";

//WIFI FUNCTIONS OVERVIEW
char set_mode[] = "AT+CWMODE_CUR=2\r\n";
char ap_connect[] = "AT+CWJAP_CUR=\"\",\"\"\r\n";
char ap_disconnect[] = "AT+CWQAP\r\n";
char ap_configuration[] = "AT+CWSAP_CUR=\"ESP8266\",\"12345678\",5,4\r\n";
char ap_ip_set[] = "AT+CIPAP_CUR=\"192.168.4.1\",\"192.168.4.1\",\"255.255.255.0\"\r\n";
char ap_retrieve_connected_ips[] = "AT+CWLIF\r\n";

//TCP/IP RELATED AT COMMANDS
char mux_enable[] = "AT+CIPMUX=1\r\n";
char start_server[] = "AT+CIPSERVER=1,8080\r\n";

char receivedMessage[256];

void resetBuffer(){
  for( int i=0; i<sizeof(receivedMessage); i++ ){
    receivedMessage[i] = 0x00;
  }
}

int string_ends_with( char *str, const char* suffix){
  int str_len = strlen(str);
  int suffix_len = strlen(suffix);
  return ( str_len >= suffix_len ) && (0 == strcmp(str + (str_len - suffix_len) , suffix));
}

char* rcvChar;
uint8_t current_state;
char dataReceived[32];
int tcp_index = 0;
uint8_t server_enabled = 0x00;
tcp_session *session = NULL;
uint8_t bufLength = 0x00;

void USART1_IRQHandler(void) {
  if ( USART_GetFlagStatus(USART1,USART_FLAG_RXNE) == SET){
    if (!server_enabled || 1){
      *rcvChar++ = USART_ReceiveData(USART1);
      *rcvChar = 0x00;
      if (string_ends_with(receivedMessage, "CLOSED\r\n")){
        rcvChar = receivedMessage;
        int start_index = 0;
        while (*rcvChar != ':'){
          start_index++;
          rcvChar++;
        }
        start_index++;
        rcvChar = &receivedMessage[start_index];
        int msg_length = 0;
        while (*rcvChar != '\r'){
          rcvChar++;
          msg_length++;
        }
        char *p;
        p = (char*) (calloc(msg_length+1, sizeof(char)));
        for (int i=0; i<msg_length; i++){
          *(p+i) = receivedMessage[start_index+i];
        }
        *(p+msg_length) = '\0';
        if (strcmp(p,"open")==0){
          GPIO_WriteBit(GPIOC, GPIO_Pin_8, Bit_SET);
        } else if (strcmp(p,"close")==0){
          GPIO_WriteBit(GPIOC, GPIO_Pin_8, Bit_RESET);
        }
        free(p);
        resetBuffer();
        rcvChar = receivedMessage;
      }
    }
  }
}

int main(void) {

  current_state = 0x00;
  
  uart_init();
  gpio_init();

  GPIO_ToggleBits(GPIOC,GPIO_Pin_9);
  rcvChar = receivedMessage;
        
  esp_send(reset);
  esp_send(disable_echo);
  esp_send(working);
  esp_send(set_mode);              
  esp_send(ap_configuration);        
  esp_send(ap_ip_set);        
  esp_send(mux_enable);        
  esp_send(start_server);
  GPIO_ToggleBits(GPIOC,GPIO_Pin_9);
  server_enabled = 0x01;
  while (1);
}

void esp_send(char* msg){
  uart_send(msg);
  delay(700000);
  __NOP();
  resetBuffer();
  rcvChar = receivedMessage;
}

void uart_send(char* msg){
  while (*msg){
    while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) != SET) ;
    USART_SendData(USART1, *msg++);
  }
}

void uart_init() {
	USART_InitTypeDef USART1_init_struct;
	GPIO_InitTypeDef gpioa_init_struct;

	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);

	GPIO_PinAFConfig(GPIOA, GPIO_PinSource9, GPIO_AF_USART1);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource10, GPIO_AF_USART1);

	gpioa_init_struct.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_10;
	gpioa_init_struct.GPIO_Speed = GPIO_Speed_2MHz;
	gpioa_init_struct.GPIO_Mode = GPIO_Mode_AF;
	gpioa_init_struct.GPIO_OType = GPIO_OType_PP;
	gpioa_init_struct.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOA, &gpioa_init_struct);

	USART1_init_struct.USART_BaudRate = 115200 * 2;
	USART1_init_struct.USART_WordLength = USART_WordLength_8b;
	USART1_init_struct.USART_StopBits = USART_StopBits_1;
	USART1_init_struct.USART_Parity = USART_Parity_No;
	USART1_init_struct.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART1_init_struct.USART_HardwareFlowControl =
	USART_HardwareFlowControl_None;
	USART_Init(USART1, &USART1_init_struct);
	USART_Cmd(USART1, ENABLE);
	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);

	//NVIC_InitTypeDef NVIC_InitStructure;

	/* Enable the USARTx Interrupt */
	//NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
	//NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	//NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	//NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	//NVIC_Init(&NVIC_InitStructure);
	NVIC_EnableIRQ(USART1_IRQn);
}

void gpio_init() {
	GPIO_InitTypeDef GPIO_InitStructure;

	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOC, ENABLE);

	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9;
	GPIO_Init(GPIOC, &GPIO_InitStructure);
}

#ifdef  USE_FULL_ASSERT

/**
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t* file, uint32_t line) {
	/* User can add his own implementation to report the file name and line number,
	 ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

	/* Infinite loop */
	while (1) {
	}
}
#endif

void delay(int a) {
	volatile int i, j;
	for (i = 0; i < a; i++) {
		j++;
	}
	return;
}
