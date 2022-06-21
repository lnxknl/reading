#define LCD_COMMAND 0
#define LCD_DATA 1

#define LCD_SETYADDR 0x40
#define LCD_SETXADDR 0x80
#define LCD_DISPLAY_BLANK 0x08
#define LCD_DISPLAY_NORMAL 0x0C
#define LCD_DISPLAY_ALL_ON 0x09
#define LCD_DISPLAY_INVERTED 0x0D

#define LCD_WIDTH 84
#define LCD_HEIGHT 48
#define LCD_SIZE LCD_WIDTH * LCD_HEIGHT / 8


#void LCD_send(uint8_t val){
  uint8_t i;

  for(i = 0; i < 8; i++){
    HAL_GPIO_WritePin(lcd_gpio.DINPORT, lcd_gpio.DINPIN, !!(val & (1 << (7 - i))));
    HAL_GPIO_WritePin(lcd_gpio.CLKPORT, lcd_gpio.CLKPIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(lcd_gpio.CLKPORT, lcd_gpio.CLKPIN, GPIO_PIN_RESET);
  }
}


void LCD_putChar(char c){
  for(int i = 0; i < 6; i++){
    if(lcd.inverttext != true)
      LCD_write(ASCII[c - 0x20][i], LCD_DATA);
    else
      LCD_write(~(ASCII[c - 0x20][i]), LCD_DATA);
  }
}


void LCD_print(char *str, uint8_t x, uint8_t y){
  LCD_goXY(x, y);
  while(*str){
    LCD_putChar(*str++);
  }
}


void LCD_clrScr(){
  for(int i = 0; i < 504; i++){
    LCD_write(0x00, LCD_DATA);
    lcd.buffer[i] = 0;
  }
}

void LCD_refreshScr(){
  LCD_goXY(LCD_SETXADDR, LCD_SETYADDR);
  for(int i = 0; i < 6; i++){
    for(int j = 0; j < LCD_WIDTH; j++){
      LCD_write(lcd.buffer[(i * LCD_WIDTH) + j], LCD_DATA);
    }
  }
}

//=========

int strlen(const char *str) {
	const char *s;

	s = str;
	while (*s)
		s++;
	return s - str;
}


char *strrev(char *str) {
	char *p1, *p2;

	if (!str || !*str)
		return str;

	for (p1 = str, p2 = str + strlen(str) - 1; p2 > p1; ++p1, --p2) {
		*p1 ^= *p2;
		*p2 ^= *p1;
		*p1 ^= *p2;
	}

	return str;
}


//====

void init_I2C1(void)
{
    // Включаем тактирование нужных модулей
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1, ENABLE);

    // А вот и настройка I2C
    i2c.I2C_ClockSpeed = 100000;
    i2c.I2C_Mode = I2C_Mode_I2C;
    i2c.I2C_DutyCycle = I2C_DutyCycle_2;
    // Адрес я тут взял первый пришедший в голову
    i2c.I2C_OwnAddress1 = 0x15;
    i2c.I2C_Ack = I2C_Ack_Enable;
    i2c.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
    I2C_Init(I2C1, &i2c);

    // I2C использует две ноги микроконтроллера, их тоже нужно настроить
    i2c_gpio.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
    i2c_gpio.GPIO_Mode = GPIO_Mode_AF_OD;
    i2c_gpio.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &i2c_gpio);

    // Ну и включаем, собственно, модуль I2C1
    I2C_Cmd(I2C1, ENABLE);
}



		double timer_float = 20/(((float)Timer_Counter)/20000);	//Frames per sec

		sprintf(counter_buff, "FPS:  %.2f", timer_float);
		ILI9341_Draw_Text(counter_buff, 10, 50, BLACK, 2, WHITE);
		double MB_PS = timer_float*240*320*2/1000000;
		sprintf(counter_buff, "MB/S: %.2f", MB_PS);

void MX_SPI5_Init(void)
{

  hspi5.Instance = SPI5;
  hspi5.Init.Mode = SPI_MODE_MASTER;
  hspi5.Init.Direction = SPI_DIRECTION_2LINES;
  hspi5.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi5.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi5.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi5.Init.NSS = SPI_NSS_SOFT;
  hspi5.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi5.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi5.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi5.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi5.Init.CRCPolynomial = 7;
  hspi5.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi5.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  if (HAL_SPI_Init(&hspi5) != HAL_OK)
  {
    Error_Handler();
  }

}

