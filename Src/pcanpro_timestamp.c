#include <assert.h>
#include <stm32f4xx_hal.h>
#include "pcanpro_timestamp.h"

#define TIM_BUS_FREQ (48000000)

void pcan_timestamp_init( void )
{
  switch( TIM_BUS_FREQ )
  {
    case 48000000:
      __HAL_RCC_TIM3_CLK_ENABLE();
      __HAL_RCC_TIM9_CLK_ENABLE();

      /* master TIM3 */
      TIM3->PSC = (48-1); /* => tick=1uS */
      /* set clock division to zero: */
      TIM3->CR1 &= (uint16_t)(~TIM_CR1_CKD);
      TIM3->CR1 |= TIM_CLOCKDIVISION_DIV1;
      TIM3->CR2 |= TIM_TRGO_UPDATE;

      /* slave - TIM9 counter the upper 16 bits */
      TIM9->PSC = 0x0000;
      /* set clock division to zero: */
      TIM9->CR1 &= (uint16_t)(~TIM_CR1_CKD);
      TIM9->CR1 |= TIM_CLOCKDIVISION_DIV1;
      TIM9->SMCR |= (TIM_TS_ITR1 | TIM_SLAVEMODE_EXTERNAL1);
      /* enable the two counters: */
      TIM9->CR1 |= TIM_CR1_CEN;
      TIM3->CR1 |= TIM_CR1_CEN;
    break;
    default:
      assert( 0 );
    break;
  }

  //HAL_GetTick must not return 0 to use LED events in early stage.
  //Let's increment the value here by one.
  HAL_IncTick();
}

uint32_t pcan_timestamp_millis( void )
{
  return  HAL_GetTick();
}

uint32_t pcan_timestamp_us( void )
{
  uint32_t tim_lo, tim_hi;
  do
  {
    tim_hi = TIM9->CNT;
    tim_lo = TIM3->CNT;
  }
  while( tim_hi != TIM9->CNT );

  return (tim_hi<<16u)|tim_lo;
}
