/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c2;

UART_HandleTypeDef huart4;

/* USER CODE BEGIN PV */
/*Информация о регистрах взята из "BME280 Combined humidity and pressure sensor"*/
#define BME280_ADDRESS 0x76<<1 /*I2C Адрес BME280, p.32, 6.2*/
#define BME280_REG_ID 0xD0 /*ID регистр BME280, p.26, 5.2*/
#define BME280_ID 0x60 /*Информация, читаемая от BME280 в ID регистре, 27, 5.4.1*/
#define BME280_REG_SOFTRESET 0xE0 /*Регистр для перезагрузки BME280, p.27, 5.4.2*/
#define BME280_SOFTRESET_VALUE 0xB6 /*Значение, записываемое в регистр для перезагрузки BME280, p.27, 5.4.2*/
#define BME280_REGISTER_STATUS 0XF3 /*Регистр статуса BME280, p.28, 5.4.4*/
#define BME280_STATUS_MEASURING 0X08 /*Значение из регистра статуса при запуске измерения BME280, p.28, 5.4.4*/
#define BME280_STATUS_IM_UPDATE 0X01 /*Значение из регистра статуса при окончании измерения BME280, p.28, 5.4.4*/
#define BME280_REGISTER_DIG_T1 0x88/*Регистр, откуда читаем калибровочное значение 1, p.24, 4.2.2*/
#define BME280_REGISTER_DIG_T2 0x8A/*Регистр, откуда читаем калибровочное значение 2, p.24, 4.2.2*/
#define BME280_REGISTER_DIG_T3 0x8C/*Регистр, откуда читаем калибровочное значение 3, p.24, 4.2.2*/

#define BME280_REGISTER_DIG_H1 0xA1/*Регистр, откуда читаем калибровочное значение H 1, p.24, 4.2.2*/
#define BME280_REGISTER_DIG_H2 0xE1/*Регистр, откуда читаем калибровочное значение H 2, p.24, 4.2.2*/
#define BME280_REGISTER_DIG_H3 0xE3/*Регистр, откуда читаем калибровочное значение H 3, p.24, 4.2.2*/
#define BME280_REGISTER_DIG_H4 0xE4/*Регистр, откуда читаем калибровочное значение H 4, p.24, 4.2.2*/
#define BME280_REGISTER_DIG_H5 0xE5/*Регистр, откуда читаем калибровочное значение H 5, p.24, 4.2.2*/
#define BME280_REGISTER_DIG_H6 0xE7/*Регистр, откуда читаем калибровочное значение H 6, p.24, 4.2.2*/
//------------------------------------------------//
#define BME280_REG_CONFIG 0xF5 /*Регистр конфигурации BME280, задаём время ожидания, значение постоянной времени
фильтра BME280, p.29, 5.4.6*/
#define BME280_STBY_MSK 0xE0/*Вспомогательная "маска" для параметрирования времени ожидания*/
/*Время ожидания t_standby, мс, p.30*/
#define BME280_STBY_0_5 0x00
#define BME280_STBY_62_5 0x20
#define BME280_STBY_125 0x40
#define BME280_STBY_250 0x60
#define BME280_STBY_500 0x80
#define BME280_STBY_1000 0xA0
#define BME280_STBY_10 0xC0
#define BME280_STBY_20 0xE0
//------------------------------------------------//
#define BME280_FILTER_MSK 0x1C/*Вспомогательная "маска" для параметрирования постоянной времени фильтра*/
/*Постоянная времени фильтра, p.30, 5.4.6., p.30*/
#define BME280_FILTER_OFF 0x00
#define BME280_FILTER_2 0x04
#define BME280_FILTER_4 0x08
#define BME280_FILTER_8 0x0C
#define BME280_FILTER_16 0x10
//------------------------------------------------//
#define BME280_REG_CTRL_HUM 0xF2 // /*Регистр параметров сбора данных влажности, p.27, 5.4.3*/

#define BME280_OSRS_H_MSK 0x07/*Вспомогательная "маска" для параметрирования преддискретизации (Table 19)*/
/*Параметр преддискретизации влажности, p.28*/
#define BME280_OSRS_H_SKIP 0x00
#define BME280_OSRS_H_x1 0x01
#define BME280_OSRS_H_x2 0x02
#define BME280_OSRS_H_x4 0x03
#define BME280_OSRS_H_x8 0x04
#define BME280_OSRS_H_x16 0x05
//------------------------------------------------//
#define BME280_MODE_MSK 0x03/*Вспомогательная "маска" для параметрирования режима работы BME280*/
/*Режим работы датчика, p.29*/
#define BME280_MODE_SLEEP 0x00
#define BME280_MODE_FORCED 0x01
#define BME280_MODE_NORMAL 0x03
//------------------------------------------------//
#define BME280_REGISTER_HUMIDDATA 0xFD/*Регистр, откуда читаются данные влажности BME280, p.31, 5.4.9*/
//------------------------------------------------
#define be24toword(a) ((((a)>>16)&0x000000ff)|((a)&0x0000ff00)|(((a)<<16)&0x00ff0000))/*Функция перестановки байтов*/

#define be16toword(a) ((((a)>>8)&0xff)|(((a)<<8)&0xff00))/*Функция перестановки байтов*/
//------------------------------------------------//
char str[100];/*Строка, в которой мы будем передавать нужную нам информацию по UART*/
typedef struct
{
  uint16_t dig_T1;
  int16_t dig_T2;
  int16_t dig_T3;
  uint8_t dig_H1;
  int16_t dig_H2;
  uint8_t dig_H3;
  int16_t dig_H4;
  int16_t dig_H5;
  int8_t dig_H6;
} BME280_CalibData;/*Переменные для калибровочных коэффициентов, p.24, 4.2.2*/
BME280_CalibData CalibData;

float hf = 0.0f;/*Переменная, куда записываем значение влажности*/
int32_t t_fine;/*Вспомогательная переменная для вычисления влажности*/
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C2_Init(void);
static void MX_USART4_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void Error()/*Функция, вызываемая в случае возникновения ошибки*/
{
	 HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);/*Выключение пользовательского светодиода*/
}
//------------------------------------------------//
void BME280I2Cx_WriteData(uint8_t Reg, uint8_t Value)/*Функция записи одного байта данных в заданный регистр*/
{
	HAL_StatusTypeDef status = HAL_OK;
	status = HAL_I2C_Mem_Write(&hi2c2, BME280_ADDRESS, (uint16_t)Reg, I2C_MEMADD_SIZE_8BIT, &Value, 1, 1000);/*Функция
записи данных value в заданный регистр Reg датчика BME280*/
	if(status != HAL_OK)/*Если произошла проблема при записи данных,*/
	{
		Error();/*Вызовем функцию ошибки*/
	}
}
//------------------------------------------------//
uint8_t BME280I2Cx_ReadData(uint8_t Reg)/*Функция чтения одного байта данных из заданного регистра*/
{
	HAL_StatusTypeDef status = HAL_OK;
	uint8_t value = 0;
	status = HAL_I2C_Mem_Read(&hi2c2, BME280_ADDRESS, Reg, I2C_MEMADD_SIZE_8BIT, &value, 1, 1000);/*Функция
чтения данных в переменную value из заданного регистра Reg датчика BME280*/
	if(status != HAL_OK)/*Если произошла проблема при чтении данных,*/
	{
		Error();/*Вызовем функцию ошибки*/
	}
	return value;/*Возвращаем прочитанное значение*/
}
//------------------------------------------------//
void BME280_ReadReg_U16(uint8_t Reg, uint16_t *Value)/*Функция чтения двух байт данных в беззнаковую переменную
value из заданного регистра*/
{
	HAL_StatusTypeDef status = HAL_OK;
	status = HAL_I2C_Mem_Read(&hi2c2, BME280_ADDRESS, Reg, I2C_MEMADD_SIZE_8BIT, (uint8_t*)Value, 2, 1000);/*Функция
чтения данных в переменную value из заданного регистра Reg датчика BME280*/
	if(status != HAL_OK)/*Если произошла проблема при чтении данных,*/
	{
		Error();/*Вызовем функцию ошибки*/
	}
}
//------------------------------------------------//
void BME280_ReadReg_S16(uint8_t Reg, int16_t *Value)/*Функция чтения двух байт данных в знаковую переменную
value из заданного регистра*/
{
	HAL_StatusTypeDef status = HAL_OK;
	status = HAL_I2C_Mem_Read(&hi2c2, BME280_ADDRESS, Reg, I2C_MEMADD_SIZE_8BIT, (uint8_t*)Value, 2, 1000);/*Функция
чтения данных в переменную value из заданного регистра Reg датчика BME280*/
	if(status != HAL_OK)/*Если произошла проблема при чтении данных,*/
	{
		Error();/*Вызовем функцию ошибки*/
	}
}
//------------------------------------------------//

//------------------------------------------------//
void BME280_ReadReg_BE_S16(uint8_t Reg, int16_t *Value)
{
	HAL_StatusTypeDef status = HAL_OK;
		status = HAL_I2C_Mem_Read(&hi2c2, BME280_ADDRESS, Reg, I2C_MEMADD_SIZE_8BIT, (uint8_t*)Value, 2, 1000);/*Функция чтения данных в переменную value из заданного регистра Reg датчика BME280*/
		if(status != HAL_OK)/*Если произошла проблема при чтении данных,*/
		{
			Error();/*Вызовем функцию ошибки*/
		}
  *(uint16_t *) Value = be16toword(*(uint16_t *) Value);
}
//------------------------------------------------//

void BME280_ReadReg_S24(uint8_t Reg, int32_t *Value)/*Функция чтения трёх байт данных в знаковую переменную value из заданного регистра*/
{
	HAL_StatusTypeDef status = HAL_OK;
	status = HAL_I2C_Mem_Read(&hi2c2, BME280_ADDRESS, Reg, I2C_MEMADD_SIZE_8BIT, (uint8_t*)Value, 3, 1000);/*Функция
чтения данных в переменную value из заданного регистра Reg датчика BME280*/
	if(status != HAL_OK)/*Если произошла проблема при чтении данных,*/
	{
		Error();/*Вызовем функцию ошибки*/
	}
	*(uint32_t *) Value = be24toword(*(uint32_t *) Value) & 0x00FFFFFF;/*Замена местами байт слова*/
}
//------------------------------------------------//
void BME280_Init()/*Функция инициализации BME280*/
{
	uint8_t value=0;
	uint32_t value32=0;
	HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);/*Зажигаем пользовательский светодиод*/
	value = BME280I2Cx_ReadData(BME280_REG_ID);/*Считаем ID датчика*/
	sprintf(str, "ID: 0x%02X\n", value);/*Запишем в строку считанные данные*/
	HAL_UART_Transmit(&huart4,(uint8_t*)str,strlen(str),1000);/*Передадим строку по UART*/
	if(value !=BME280_ID)/*Если считанные данные не равны ID датчика, то*/
	{
		Error();/*Вызовем функцию ошибки*/
	}
	BME280I2Cx_WriteData(BME280_REG_SOFTRESET, BME280_SOFTRESET_VALUE);/*Перезагрузим датчик*/
	while (BME280I2Cx_ReadData(BME280_REGISTER_STATUS)&0x09 & BME280_STATUS_IM_UPDATE);/*Проверяем статус датчика,
готов ли датчик к работе*/

	/*Считаем калибровочные коэффициенты влажности датчика и передадим каждый из коэффициентов по UART*/
	CalibData.dig_H1 = BME280I2Cx_ReadData(BME280_REGISTER_DIG_H1);
	sprintf(str, "DIG_H1: %d\n", CalibData.dig_H1);
	HAL_UART_Transmit(&huart4,(uint8_t*)str,strlen(str),0x1000);
	BME280_ReadReg_S16(BME280_REGISTER_DIG_H2,&CalibData.dig_H2);
	sprintf(str, "DIG_H2: %d\n", CalibData.dig_H2);
	HAL_UART_Transmit(&huart4,(uint8_t*)str,strlen(str),0x1000);
	CalibData.dig_H3 = BME280I2Cx_ReadData(BME280_REGISTER_DIG_H3);
	sprintf(str, "DIG_H3: %d\n", CalibData.dig_H3);
	HAL_UART_Transmit(&huart4,(uint8_t*)str,strlen(str),0x1000);
	CalibData.dig_H4 = (BME280I2Cx_ReadData(BME280_REGISTER_DIG_H4) << 4) | (BME280I2Cx_ReadData(BME280_REGISTER_DIG_H4+1) & 0xF);
	sprintf(str, "DIG_H4: %d\n", CalibData.dig_H4);
	HAL_UART_Transmit(&huart4,(uint8_t*)str,strlen(str),0x1000);
	CalibData.dig_H5 = (BME280I2Cx_ReadData(BME280_REGISTER_DIG_H5+1) << 4) | (BME280I2Cx_ReadData(BME280_REGISTER_DIG_H5) >> 4);
	sprintf(str, "DIG_H5: %d\n", CalibData.dig_H5);
	HAL_UART_Transmit(&huart4,(uint8_t*)str,strlen(str),0x1000);
	CalibData.dig_H6 = (int8_t)BME280I2Cx_ReadData(BME280_REGISTER_DIG_H6);
	sprintf(str, "DIG_H6: %d\n", CalibData.dig_H3);
	HAL_UART_Transmit(&huart4,(uint8_t*)str,strlen(str),0x1000);

	/*Зададим время ожидания датчика*/
	value = BME280I2Cx_ReadData(BME280_REG_CONFIG) & ~BME280_STBY_MSK;/*Считаем данные из регистра*/
	value |= BME280_STBY_1000 & BME280_STBY_MSK;/*Перезапишем именно те биты, которые отвечают за время ожидания*/
	BME280I2Cx_WriteData(BME280_REG_CONFIG,value);/*Запишем новые данные в регистр конфигурации BME280*/

	/*Зададим постоянную времени фильтра*/
	value = BME280I2Cx_ReadData(BME280_REG_CONFIG) & ~BME280_FILTER_MSK;/*Считаем данные из регистра*/
	value |= BME280_FILTER_4 & BME280_FILTER_MSK;/*Перезапишем биты, которые отвечают за постоянную времени фильтра*/
	BME280I2Cx_WriteData(BME280_REG_CONFIG,value);/*Запишем новые данные в регистр конфигурации BME280*/

	/*Настроим параметры сбора данных влажности*/
	value = BME280I2Cx_ReadData(BME280_REG_CTRL_HUM) & ~BME280_OSRS_H_MSK;/*Считаем данные из регистра*/
	value |= BME280_OSRS_H_x1 & BME280_OSRS_H_MSK;/*Перезапишем биты, которые отвечают за преддискретизацию влажности*/
	BME280I2Cx_WriteData(BME280_REG_CTRL_HUM,value);/*Запишем новые данные в регистр параметров сбора данных*/

	//value = BME280I2Cx_ReadData(BME280_REG_CTRL_MEAS);
	//BME280I2Cx_WriteData(BME280_REG_CTRL_MEAS,value);

	value32 = BME280I2Cx_ReadData(BME280_REG_CTRL_MEAS);/*Считаем данные из регистра параметров сбора данных*/

	value32 |= BME280I2Cx_ReadData(BME280_REG_CTRL_HUM) << 8;/*Считаем данные из регистра параметров сбора данных*/

	sprintf(str, "Measurements status: %04X\n", value32);/*Запишем в строку считанные данные*/
	HAL_UART_Transmit(&huart4,(uint8_t*)str,strlen(str),1000);/*Передадим строку по UART*/
	sprintf(str, "Temperature: %s\n Humidity: %s\n",
				(value32 & BME280_OSRS_T_MSK) ? "ON" : "OFF",
				((value32 >> 8) & BME280_OSRS_H_MSK) ? "ON" : "OFF");/*Запишем в строку считанные данные и, если всё в порядке,
				то в строку пропишем ON, иначе OFF*/
	HAL_UART_Transmit(&huart4,(uint8_t*)str,strlen(str),1000);/*Передадим строку по UART*/
	/*Настроим режим работы*/
	value = BME280I2Cx_ReadData(BME280_REG_CTRL_MEAS) & ~BME280_MODE_MSK;/*Считаем данные из регистра параметров сбора данных*/
	value |= BME280_MODE_NORMAL & BME280_MODE_MSK;/*Перезапишем биты, которые отвечают за настройку режима работы BME280*/
	BME280I2Cx_WriteData(BME280_REG_CTRL_MEAS,value);/*Запишем новые данные в регистр параметров сбора данных*/
}
//------------------------------------------------//
float BME280_ReadHumidity()/*Функция чтения влажности*/
{
  float hum_float = 0.0f;/*Переменная типа Float (с плавающей точкой), куда записывается значение влажности*/
  int16_t hum_raw;/*Вспомогательная знаковая переменная, в которое переводятся полученные из регистра датчика данные*/
  int32_t hum_raw_sign, v_x1_u32r;/*Вспомогательные переменные для вычисления влажности*/
  BME280_ReadTemperature(); //Вычисление температуры для получения значения t_fine
  BME280_ReadReg_BE_S16(BME280_REGISTER_HUMIDDATA,&hum_raw);/*Читаем данные из регистра данных влажности*/
  hum_raw_sign = ((int32_t)hum_raw)&0x0000FFFF;/*Присвоим считанное значение 32-битной знаковой переменной*/
  /*Далее используем программный код, предлагаемый в документации от производителя, p.25*/
  v_x1_u32r = (t_fine - ((int32_t)76800));
  v_x1_u32r = (((((hum_raw_sign << 14) - (((int32_t)CalibData.dig_H4) << 20) -
  (((int32_t)CalibData.dig_H5) * v_x1_u32r)) + ((int32_t)16384)) >> 15) *
  (((((((v_x1_u32r * ((int32_t)CalibData.dig_H6)) >> 10) *
  (((v_x1_u32r * ((int32_t)CalibData.dig_H3)) >> 11) + ((int32_t)32768))) >> 10) +
  ((int32_t)2097152)) * ((int32_t)CalibData.dig_H2) + 8192) >> 14));
  v_x1_u32r = (v_x1_u32r - (((((v_x1_u32r >> 15) * (v_x1_u32r >> 15)) >> 7) *
  ((int32_t)CalibData.dig_H1)) >> 4));
  v_x1_u32r = (v_x1_u32r < 0) ? 0 : v_x1_u32r;
  v_x1_u32r = (v_x1_u32r > 419430400) ? 419430400 : v_x1_u32r;
  hum_float = (v_x1_u32r>>12);
  hum_float /= 1024.0f;/*Поделим вычисленные данные на 1024, чтобы привести данные к удобному для чтения виду*/
  return hum_float;/*Возвращаем вычисленное в функции значение*/
}
//------------------------------------------------
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C2_Init();
  MX_USART4_UART_Init();
  /* USER CODE BEGIN 2 */
  BME280_Init();/*Запустим функцию инициализации датчика BME280*/
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  hf = BME280_ReadHumidity();/*Считаем значение влажности из функции вычисления влажности*/
 	  sprintf(str, "Humidity: %.2f %%\r\n", hf);/*Запишем в строку считанное из функции значение влажности*/
 	  HAL_UART_Transmit(&huart4,(uint8_t*)str,strlen(str),0x1000);/*Передадим строку по UART*/

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.Timing = 0x2000090E;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c2, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c2, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief USART4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART4_UART_Init(void)
{

  /* USER CODE BEGIN USART4_Init 0 */

  /* USER CODE END USART4_Init 0 */

  /* USER CODE BEGIN USART4_Init 1 */

  /* USER CODE END USART4_Init 1 */
  huart4.Instance = USART4;
  huart4.Init.BaudRate = 9600;
  huart4.Init.WordLength = UART_WORDLENGTH_9B;
  huart4.Init.StopBits = UART_STOPBITS_1;
  huart4.Init.Parity = UART_PARITY_EVEN;
  huart4.Init.Mode = UART_MODE_TX_RX;
  huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart4.Init.OverSampling = UART_OVERSAMPLING_16;
  huart4.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart4.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART4_Init 2 */

  /* USER CODE END USART4_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LED_Pin */
  GPIO_InitStruct.Pin = LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
