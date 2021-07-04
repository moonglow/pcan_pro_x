#include <stm32f4xx_hal.h>
#include "pcanpro_timestamp.h"
#include "pcanpro_can.h"
#include "pcanpro_led.h"
#include "pcanpro_protocol.h"
#include "usb_device.h"

void Error_Handler(void)
{
  /* reboot */
  HAL_Delay( 250 );
  HAL_NVIC_SystemReset();
  for(;;);
}

void SysTick_Handler(void)
{
  HAL_IncTick();
}

void HAL_MspInit(void)
{
  __HAL_RCC_SYSCFG_CLK_ENABLE();
  __HAL_RCC_PWR_CLK_ENABLE();
}

static void pcan_io_config(void)
{
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
}

void pcan_clock_config( void )
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /* Configure the main internal regulator output voltage */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /* Initializes the CPU, AHB and APB busses clocks */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 192;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if( HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK )
  {
    Error_Handler();
  }
  /* Initializes the CPU, AHB and APB busses clocks */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if( HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3 ) != HAL_OK )
  {
    Error_Handler();
  }
}

int main(void)
{
  HAL_Init();

  pcan_clock_config();
  pcan_io_config();
  pcan_timestamp_init();
  
  pcan_led_init();
  pcan_led_set_mode( LED_STAT, LED_MODE_BLINK_FAST, 0xFFFFFFFF );
  pcan_protocol_init();
  pcan_usb_device_init();
  
  for(;;)
  {
    pcan_usb_device_poll();
    pcan_can_poll();
    pcan_protocol_poll();
    pcan_led_poll();
  }
}
