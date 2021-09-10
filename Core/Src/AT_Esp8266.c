#include "AT_Esp8266.h"

uint8_t transMode = SINGLE_CON;
uint8_t TransparentProtocol = UDP_TRANS;

void ESP_SendCommand(char *cmd)
{
	int len = strlen(cmd);
	char str[100] = {0};
	sprintf(str,"%s\r\n",cmd);
	HAL_UART_Transmit(&MyUart,(uint8_t*)str, len + 2, 100);
}

void ESP_SetIP(uint8_t mode, char *ip)
{
	char cmd[100] = {0};
	if(mode == STA)
	{
		sprintf(cmd,"AT+CIPSTA=\"%s\"",ip);
	}
	if(mode == AP)
	{
		sprintf(cmd,"AT+CIPAP=\"%s\"",ip);
	}
	ESP_SendCommand(cmd);
	HAL_Delay(50);
}

void ESP_Init(UART_HandleTypeDef *huart, uint8_t mode)
{
	memcpy(&MyUart,huart,sizeof(*huart));
	char cmd[100] = {0};
	sprintf(cmd,"AT+CWMODE=%d",mode);
	ESP_SendCommand(cmd);
	HAL_Delay(50);
	switch(mode)
	{
		case STA: ESP_SetIP(STA,IP);
							break;
		case AP:  ESP_SetIP(AP, IP);
							break;
		case STA_AP: ESP_SetIP(STA,IP);
								 break;
		default: break;
	}
}

void ESP_WifiConnect(char *ssid, char *pass)
{
	char cmd[100] = {0};
	sprintf(cmd,"AT+CWJAP=\"%s\",\"%s\"",ssid,pass);
	while(1)
	{
	  ESP_SendCommand(cmd);
		for(int i = 0; i<100; i++)
		HAL_Delay(10);
		if(strstr(request,"WIFI CONNECTED")) break;
	}
}

void ESP_SoftAPCreate(char *ssid, char *pass)
{
	char cmd[100] = {0};
	sprintf(cmd,"AT+CWSAP=\"%s\",\"%s\",1,2",ssid,pass);
  ESP_SendCommand(cmd);
	HAL_Delay(50);
}

HAL_StatusTypeDef ESP_CheckWifiConnect(void)
{
	ESP_SendCommand("AT+CWJAP?");
	for(int i = 0; i<100; i++)
	HAL_Delay(10);
	if(strstr(request,"+CWJAP:")) return HAL_OK;
	else if(strstr(request,"No AP"))
	{
		ESP_SendCommand("AT+CWJAP?");
		for(int i = 0; i<400; i++)
		HAL_Delay(10);
		if(strstr(request,"No AP")) return HAL_ERROR;
	}
	return HAL_BUSY;
}

void ESP_TCP_CreateServer(void)
{
	ESP_SendCommand("AT+CIPMUX=1");
	transMode = MULTI_CON;
	HAL_Delay(50);
	char str[100] = {0};
	sprintf(str,"AT+CIPSERVER=1,%d",PORT);
	memset(request,0,BUFFER_SIZE);
	ESP_SendCommand(str);
	HAL_Delay(50);
	while(!strstr(request,"OK"))
	{
		ESP_SendCommand(str);
		HAL_Delay(50);
	}
}

void ESP_SendData(char *data)
{
	int len = strlen(data);
	char str[100] = {0};
	sprintf(str,"AT+CIPSEND=%d,%d",clientID,len);
	ESP_SendCommand(str);
	HAL_Delay(3);
	ESP_SendCommand(data);
}

void ESP_CloseServer(void)
{
	ESP_SendCommand("AT+CIPSERVER=0");
}

void ESP_CloseConnect(void)
{
	if(transMode == SINGLE_CON) ESP_SendCommand("AT+CIPCLOSE");
	else
	{
		char str[100] = {0};
		sprintf(str,"AT+CIPCLOSE=%d",clientID);
		ESP_SendCommand(str);
	}
}

void ESP_UDP_CreateTransparentMode(char *ip, uint16_t server_port)
{
	memset(request,0,BUFFER_SIZE);
	ESP_SendCommand("AT+CIPMUX=0");
	transMode = SINGLE_CON;
	HAL_Delay(50);
	ESP_SendCommand("AT+CIPMODE=1");
	HAL_Delay(50);
	char str[100] = {0};
	sprintf(str,"AT+CIPSTART=\"UDP\",\"%s\",%d,%d,0",ip,server_port,PORT);
	ESP_SendCommand(str);
	HAL_Delay(50);
	while(!strstr(request,"CONNECT"))
	{
		ESP_SendCommand(str);
		HAL_Delay(50);
	}
	ESP_SendCommand("AT+CIPSEND");
	HAL_Delay(50);
	while(!strstr(request,">"))
	{
		ESP_SendCommand("AT+CIPSEND");
		HAL_Delay(50);
	}
	TransparentProtocol = UDP_TRANS;
}

void ESP_TransparentSend(char *data)
{
	ESP_SendCommand(data);
}

void ESP_CloseTransparent(void)
{
	memset(request,0,BUFFER_SIZE);
	char str[3] = "+++";
	for(int i = 0; i<3; i++)
	{
		HAL_UART_Transmit(&MyUart,(uint8_t*)str, 3, 100);
		HAL_Delay(50);
		ESP_CloseConnect();
		HAL_Delay(50);
	}
}

void ESP_TCP_CreateTransparentMode(char *ip, uint16_t server_port)
{
	memset(request,0,BUFFER_SIZE);
	ESP_SendCommand("AT+CIPMUX=0");
	transMode = SINGLE_CON;
	HAL_Delay(50);
	ESP_SendCommand("AT+CIPMODE=1");
	HAL_Delay(50);
	char str[100] = {0};
	sprintf(str,"AT+CIPSTART=\"TCP\",\"%s\",%d",ip,server_port);
	ESP_SendCommand(str);
	HAL_Delay(50);
	while(!strstr(request,"CONNECT"))
	{
		ESP_SendCommand(str);
		HAL_Delay(50);
	}
	ESP_SendCommand("AT+CIPSEND");
	HAL_Delay(50);
	while(!strstr(request,">"))
	{
		ESP_SendCommand("AT+CIPSEND");
		HAL_Delay(50);
	}
	TransparentProtocol = TCP_TRANS;
}


