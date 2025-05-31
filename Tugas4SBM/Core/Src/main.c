#include "main.h"
#include <string.h>
#include <stdlib.h>
#include <stdio.h>

ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
#define ADC_BUF_LEN 16
uint16_t adc_dma_buf[ADC_BUF_LEN];
float adc_dma_avg = 0;
char uart_rx_buf[10];
uint8_t uart_rx_index = 0;
uint8_t uart_rx_data;

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART1_UART_Init(void);

void uart_send_string(char *s);
void process_menu(char *cmd);
void start_adc_dma(void);
float average_adc_dma(void);
void update_pwm_duty(uint16_t val);

int main(void)
{

  HAL_Init();

  SystemClock_Config();


  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_TIM2_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
  HAL_UART_Receive_IT(&huart1, &uart_rx_data, 1);
  start_adc_dma();
  while (1)
  {

  }

}

void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

static void MX_ADC1_Init(void)
{

  ADC_ChannelConfTypeDef sConfig = {0};

  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_9;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }


}

static void MX_TIM2_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 71;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 625;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }

  HAL_TIM_MspPostInit(&htim2);

}


static void MX_USART1_UART_Init(void)
{


  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }

}


static void MX_DMA_Init(void)
{


  __HAL_RCC_DMA2_CLK_ENABLE();

  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);

}

static void MX_GPIO_Init(void)
{

  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if(huart->Instance == USART1)
  {
    if(uart_rx_data == '\r' || uart_rx_data == '\n')
    {
      uart_rx_buf[uart_rx_index] = 0;
      process_menu(uart_rx_buf);
      uart_rx_index = 0;
      memset(uart_rx_buf, 0, sizeof(uart_rx_buf));
    }
    else
    {
      if(uart_rx_index < sizeof(uart_rx_buf)-1)
      {
        uart_rx_buf[uart_rx_index++] = uart_rx_data;
      }
    }
    // Start reception again
    HAL_UART_Receive_IT(&huart1, &uart_rx_data, 1);
  }
}

void uart_send_string(char *s)
{
  HAL_UART_Transmit(&huart1, (uint8_t*)s, strlen(s), HAL_MAX_DELAY);
}

void process_menu(char *cmd)
{
  if(cmd[0] == '1')
  {
    int val = atoi(&cmd[1]);
    if(val >=0 && val <= 625)
    {
      update_pwm_duty(val);
      char msg[30];
      sprintf(msg, "PWM duty set to %d\r\n", val);
      uart_send_string(msg);
    }
    else
    {
      uart_send_string("Invalid PWM value (0-625)\r\n");
    }
  }
  else if(cmd[0] == '2')
  {
    HAL_ADC_Stop(&hadc1);
    hadc1.Init.ContinuousConvMode = DISABLE;
    HAL_ADC_Init(&hadc1);

    HAL_ADC_Start(&hadc1);
    if(HAL_ADC_PollForConversion(&hadc1, 100) == HAL_OK)
    {
      uint32_t val = HAL_ADC_GetValue(&hadc1);
      char msg[30];
      sprintf(msg, "ADC single val: %lu\r\n", val);
      uart_send_string(msg);
    }
    HAL_ADC_Stop(&hadc1);
    hadc1.Init.ContinuousConvMode = ENABLE;
    HAL_ADC_Init(&hadc1);
    start_adc_dma();
  }
  else if(cmd[0] == '3')
  {
    float avg = average_adc_dma();
    char msg[32];
    sprintf(msg, "ADC DMA avg: %.2f\r\n", avg);
    uart_send_string(msg);
  }
  else if(cmd[0] == '4')
  {

    while(1)
    {
      float avg = average_adc_dma();
      update_pwm_duty((uint16_t)avg);
      char msg[30];
      sprintf(msg, "Run frame ADC avg: %.2f\r\n", avg);
      uart_send_string(msg);
      HAL_Delay(500);

    }
  }
  else
  {
    uart_send_string("Unknown command\r\n");
  }
}

void update_pwm_duty(uint16_t val)
{
  if(val > 625) val = 625;
  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, val);
}

void start_adc_dma(void)
{
  HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_dma_buf, ADC_BUF_LEN);
}

float average_adc_dma(void)
{
  uint32_t sum = 0;
  for(int i=0; i<ADC_BUF_LEN; i++)
  {
    sum += adc_dma_buf[i];
  }
  return (float)sum / ADC_BUF_LEN;
}

void Error_Handler(void)
{

  __disable_irq();
  while (1)
  {
  }
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
