


enum {TEMP, HUMI};


#define SHT10_AHB2_CLK        RCC_APB2Periph_GPIOB
#define SHT10_DATA_PIN        GPIO_PIN_2
#define SHT10_SCK_PIN         GPIO_PIN_3
#define SHT10_DATA_PORT       GPIOB
#define SHT10_SCK_PORT        GPIOB

#define SHT10_DATA_H()        HAL_GPIO_WritePin(SHT10_DATA_PORT, SHT10_DATA_PIN,GPIO_PIN_SET)                         
#define SHT10_DATA_L()        HAL_GPIO_WritePin(SHT10_DATA_PORT, SHT10_DATA_PIN,GPIO_PIN_RESET)                       
#define SHT10_DATA_R()        HAL_GPIO_ReadPin(SHT10_DATA_PORT, SHT10_DATA_PIN)        

#define SHT10_SCK_H()        HAL_GPIO_WritePin(SHT10_SCK_PORT, SHT10_SCK_PIN,GPIO_PIN_SET)                                
#define SHT10_SCK_L()        HAL_GPIO_WritePin(SHT10_SCK_PORT, SHT10_SCK_PIN,GPIO_PIN_RESET)                        

#define noACK        0
#define ACK          1

//addr  command         r/w
#define STATUS_REG_W         0x06        //000         0011          0          
#define STATUS_REG_R         0x07        //000         0011          1          
#define MEASURE_TEMP         0x03        //000         0001          1          
#define MEASURE_HUMI         0x05        //000         0010          1          
#define SOFTRESET            0x1E        //000         1111          0          
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f1xx_hal.h"
#include "math.h"

#include "stdio.h"
#include "stdlib.h"

I2C_HandleTypeDef hi2c1;
UART_HandleTypeDef huart1;


void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART1_UART_Init(void);


void SHT10_Config(void);
void SHT10_ConReset(void);
uint8_t SHT10_SoftReset(void);
void myprint(const char* str);
uint8_t SHT10_Measure(uint16_t *p_value, uint8_t *p_checksum, uint8_t mode);
void SHT10_Calculate(uint16_t, uint16_t rh,float *p_temperature, float *p_humidity);
float SHT10_CalcuDewPoint(float t, float h);

char mass_h[14];
char mass_t[14];

uint16_t humi_val, temp_val;
uint8_t err = 0, checksum = 0;
float humi_val_real = 0.0; 
float temp_val_real = 0.0;
float dew_point = 0.0; 


main(void)
{

  
  HAL_Init();
  
  
  SystemClock_Config();
  
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_USART1_UART_Init();
  
  SHT10_Config();
  SHT10_Config();
  
  while(1)
  {
    err += SHT10_Measure(&temp_val, &checksum, TEMP);              
    err += SHT10_Measure(&humi_val, &checksum, HUMI);                 
    if(err != 0)
      SHT10_ConReset();
    else
    {
      SHT10_Calculate(temp_val, humi_val, &temp_val_real, &humi_val_real); 
      dew_point = SHT10_CalcuDewPoint(temp_val_real, humi_val_real);             
    }
    
    
    HAL_UART_Transmit(&huart1,"Sensor:1",8,100);
    HAL_UART_Transmit(&huart1,"\r\n",2,100);
    
    temp_val_real = round(temp_val_real*100)/100;
    humi_val_real = round(humi_val_real*100)/100;
    sprintf(mass_t,"Temp:%3.2f \r\n",temp_val_real);
    HAL_UART_Transmit(&huart1,mass_t,10,100);
    HAL_UART_Transmit(&huart1,"\r\n",2,100);
    
    sprintf (mass_h,"Humm:%3.2f \r\n",humi_val_real);
    HAL_UART_Transmit(&huart1,mass_h,10,100);
    HAL_UART_Transmit(&huart1,"\r\n",2,100);
    
    HAL_UART_Transmit(&huart1,"______\r\n",8,100);
    HAL_Delay(1000);
    
    
  }
  
  
}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{
  
  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  
  /**Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }
  
  /**Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
    |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }
  
  /**Configure the Systick interrupt time 
  */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);
  
  /**Configure the Systick 
  */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);
  
  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* I2C1 init function */
static void MX_I2C1_Init(void)
{
  
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }
  
}

/* USART1 init function */
static void MX_USART1_UART_Init(void)
{
  
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
 // huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }
  
}


static void MX_GPIO_Init(void)
{
  
  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  
}


void _Error_Handler(char * file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
  {
  }

}


void SHT10_Dly(void)
{
  uint16_t i;
  for(i = 500; i > 0; i--);
}


void SHT10_Config(void)
{
  GPIO_InitTypeDef GPIO_InitStruct;

  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  
  GPIO_InitStruct.Pin = SHT10_DATA_PIN | SHT10_SCK_PIN;
  GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  HAL_GPIO_Init(SHT10_DATA_PORT, &GPIO_InitStruct);
  SHT10_ConReset();        
}



void SHT10_DATAOut(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
      
  GPIO_InitStructure.Pin = SHT10_DATA_PIN;
  GPIO_InitStructure.Speed = GPIO_SPEED_HIGH;
  GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;         
  HAL_GPIO_Init(SHT10_DATA_PORT, &GPIO_InitStructure);
}



void SHT10_DATAIn(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  
  GPIO_InitStructure.Pin = SHT10_DATA_PIN;
  GPIO_InitStructure.Speed = GPIO_SPEED_HIGH;
  GPIO_InitStructure.Mode = GPIO_MODE_INPUT;
  HAL_GPIO_Init(SHT10_DATA_PORT, &GPIO_InitStructure);
}



uint8_t SHT10_WriteByte(uint8_t value)
{
  uint8_t i, err = 0;
  
  SHT10_DATAOut();                         
  
  for(i = 0x80; i > 0; i /= 2)  
  {
    if(i & value)
      SHT10_DATA_H();
    else
      SHT10_DATA_L();
    SHT10_Dly();
    SHT10_SCK_H();
    SHT10_Dly();
    SHT10_SCK_L();
    SHT10_Dly();
  }
  SHT10_DATAIn();                               
  SHT10_SCK_H();
  err = SHT10_DATA_R();                  
  SHT10_SCK_L();
  
  return err;
}


uint8_t SHT10_ReadByte(uint8_t Ack)
{
  uint8_t i, val = 0;
  
  SHT10_DATAIn();                              
  for(i = 0x80; i > 0; i /= 2)  
  {
    SHT10_Dly();
    SHT10_SCK_H();
    SHT10_Dly();
    if(SHT10_DATA_R())
      val = (val | i);
    SHT10_SCK_L();
  }
  SHT10_DATAOut();                        
  if(Ack)
    SHT10_DATA_L();                          
  else
    SHT10_DATA_H();                        
  SHT10_Dly();
  SHT10_SCK_H();
  SHT10_Dly();
  SHT10_SCK_L();
  SHT10_Dly();
  
  return val;                                         
}



void SHT10_TransStart(void)
{
  SHT10_DATAOut();                          
  
  SHT10_DATA_H();
  SHT10_SCK_L();
  SHT10_Dly();
  SHT10_SCK_H();
  SHT10_Dly();
  SHT10_DATA_L();
  SHT10_Dly();
  SHT10_SCK_L();
  SHT10_Dly();
  SHT10_SCK_H();
  SHT10_Dly();
  SHT10_DATA_H();
  SHT10_Dly();
  SHT10_SCK_L();
  
}



void SHT10_ConReset(void)
{
  uint8_t i;
  
  SHT10_DATAOut();
  
  SHT10_DATA_H();
  SHT10_SCK_L();
  
  for(i = 0; i < 9; i++)                 
  {
    SHT10_SCK_H();
    SHT10_Dly();
    SHT10_SCK_L();
    SHT10_Dly();
  }
  SHT10_TransStart();                         
}




uint8_t SHT10_SoftReset(void)
{
  uint8_t err = 0;
  
  SHT10_ConReset();                              
  err += SHT10_WriteByte(SOFTRESET);/
  
  return err;
}


uint8_t SHT10_ReadStatusReg(uint8_t *p_value, uint8_t *p_checksum)
{
  uint8_t err = 0;
  
  SHT10_TransStart();                                       
  err = SHT10_WriteByte(STATUS_REG_R);
  *p_value = SHT10_ReadByte(ACK);              
  *p_checksum = SHT10_ReadByte(noACK);
  
  return err;
}




uint8_t SHT10_WriteStatusReg(uint8_t *p_value)
{
  uint8_t err = 0;
  
  SHT10_TransStart();                                       
  err += SHT10_WriteByte(STATUS_REG_W);
  err += SHT10_WriteByte(*p_value);         
  
  return err;
}




uint8_t SHT10_Measure(uint16_t *p_value, uint8_t *p_checksum, uint8_t mode)
{
  uint8_t err = 0;
  uint32_t i;
  uint8_t value_H = 0;
  uint8_t value_L = 0;
  
  SHT10_TransStart();                                              
  switch(mode)                                                         
  {
  case TEMP:                                                             
    err += SHT10_WriteByte(MEASURE_TEMP);
    break;
  case HUMI:
    err += SHT10_WriteByte(MEASURE_HUMI);
    break;
  default:
    break;
  }
  SHT10_DATAIn();
  for(i = 0; i < 72000000; i++)                             
  {
    if(SHT10_DATA_R() == 0) break;            
  }
  if(SHT10_DATA_R() == 1)                                
    err += 1;
  value_H = SHT10_ReadByte(ACK);
  value_L = SHT10_ReadByte(ACK);
  *p_checksum = SHT10_ReadByte(noACK);           
  *p_value = (value_H << 8) | value_L;
  return err;
}



*************************************************************/
void SHT10_Calculate(uint16_t t, uint16_t rh, float *p_temperature, float *p_humidity)
{
  const float d1 = -39.7;
  const float d2 = +0.01;
  const float C1 = -2.0468;
  const float        C2 = +0.0367;
  const float C3 = -0.0000015955;        
  const float T1 = +0.01;
  const float T2 = +0.00008;
  
  float RH_Lin;                                                                                       
  float RH_Ture;                                                                                 
  float temp_C;
  
  temp_C = d1 + d2 * t;                                                              
  RH_Lin = C1 + C2 * rh + C3 * rh * rh;                           
  RH_Ture = (temp_C -25) * (T1 + T2 * rh) + RH_Lin;      
  
  if(RH_Ture > 100)                                                                    
    RH_Ture        = 100;
  if(RH_Ture < 0.1)
    RH_Ture = 0.1;                                                                    
  
  *p_humidity = RH_Ture;
  *p_temperature = temp_C;
  
}



float SHT10_CalcuDewPoint(float t, float h)
{
  float logEx, dew_point;
  
  logEx = 0.66077 + 7.5 * t / (237.3 + t) + (log10(h) - 2);
  dew_point = ((0.66077 - logEx) * 237.3) / (logEx - 8.16077);
  
  return dew_point; 
}







