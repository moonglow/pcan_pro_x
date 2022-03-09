#pragma once
#include <stm32f4xx.h>


/* pin mode */
// MODE_INPUT    
// MODE_OUTPUT_PP
// MODE_OUTPUT_OD
// MODE_AF_PP    
// MODE_AF_OD    
// MODE_ANALOG
// MODE_IT_RISING        
// MODE_IT_FALLING       
// MODE_IT_RISING_FALLING
// MODE_EVT_RISING        
// MODE_EVT_FALLING       
// MODE_EVT_RISING_FALLING

/* pin speed */
// SPEED_FREQ_LOW      
// SPEED_FREQ_MEDIUM   
// SPEED_FREQ_HIGH     
// SPEED_FREQ_VERY_HIGH

/* pin pulls */
//NOPULL  
//PULLUP  
//PULLDOWN


/* pin config */
#define GPIO_NOAF (0u)
#define EX_PIN_INIT( _PORT, _PIN, _MODE, _PULL, _SPEED, _AF )\
HAL_GPIO_Init( GPIO##_PORT, (GPIO_InitTypeDef[])\
{{\
  .Pin = GPIO_PIN_##_PIN,\
  .Mode = GPIO_##_MODE,\
  .Pull = GPIO_##_PULL,\
  .Speed = GPIO_##_SPEED,\
  .Alternate = GPIO_##_AF\
}} )

#define PIN_INIT( CONFIG ) EX_PIN_INIT( CONFIG )

#define EX_PIN_DEINIT( _PORT, _PIN, ... ) HAL_GPIO_DeInit( GPIO##_PORT, GPIO_PIN_##_PIN )
#define PIN_DEINIT( CONFIG ) EX_PIN_DEINIT( CONFIG )

/* pin control */
#define EX_GPIO_PIN_STAT( PORT, PIN, ... )      ( (GPIO##PORT->IDR & (1u<<PIN)) != 0 )
#define EX_GPIO_PIN_STAT_INV( PORT, PIN, ... )  ( (GPIO##PORT->IDR & (1u<<PIN)) == 0 )

#define PIN_STAT( CONFIG ) EX_GPIO_PIN_STAT( CONFIG )
#define PIN_INV_STAT( CONFIG ) EX_GPIO_PIN_STAT_INV( CONFIG )
/* -------------------------- */
#define EX_GPIO_PIN_HI( PORT, PIN, ... )        GPIO##PORT->BSRR = (1u<<PIN)
#define EX_GPIO_PIN_LOW( PORT, PIN, ... )       GPIO##PORT->BSRR = (0x10000u<<PIN)

#define PIN_HI( CONFIG ) EX_GPIO_PIN_HI( CONFIG )
#define PIN_LOW( CONFIG ) EX_GPIO_PIN_LOW( CONFIG )
#define PIN_HiZ( CONFIG ) EX_GPIO_PIN_HI( CONFIG )
#define PIN_1( CONFIG ) EX_GPIO_PIN_HI( CONFIG )
#define PIN_0( CONFIG ) EX_GPIO_PIN_LOW( CONFIG )
#define PIN_ONE( CONFIG ) EX_GPIO_PIN_HI( CONFIG )
#define PIN_ZERO( CONFIG ) EX_GPIO_PIN_LOW( CONFIG )
/* -------------------------- */
#define EX_GPIO_PIN_TOGGLE( PORT, PIN, ... )\
do{\
  if( GPIO##PORT->ODR & (1u<<PIN) ){\
    EX_GPIO_PIN_LOW( PORT, PIN );\
  }\
  else{\
    EX_GPIO_PIN_HI( PORT, PIN );\
  }\
}while(0)

#define PIN_TOGGLE( CONFIG ) EX_GPIO_PIN_TOGGLE( CONFIG )

#define EX_PIN_PORT( PORT, ... ) PORT
#define PIN_PORT( CONFIG ) EX_PIN_PORT( CONFIG )

/* maximimum is 8 params */
#define NUM_ARGS_(_1, _2, _3, _4, _5, _6, _7, _8, TOTAL, ...) TOTAL
#define NUM_ARGS(...) NUM_ARGS_(__VA_ARGS__, 8, 7, 6, 5, 4, 3, 2, 1)

#define CONCATE_(X, Y) X##Y
#define CONCATE(MACRO, NUMBER) CONCATE_(MACRO, NUMBER)
#define VA_MACRO(MACRO, ...) CONCATE(MACRO, NUM_ARGS(__VA_ARGS__))(__VA_ARGS__)

#define PORT_ENABLE_CLOCK(...)\
do{\
  __IO uint32_t tmp;\
  VA_MACRO(PORT_ENABLE_CLOCK, __VA_ARGS__);\
  /* readback */\
  tmp = RCC->AHB1ENR;\
  (void)tmp;\
}while(0)

#define PORT_ENABLE_CLOCK1(_1)                                  RCC->AHB1ENR |= RCC_AHB1ENR_GPIO##_1##EN
#define PORT_ENABLE_CLOCK2(_1, _2)                              RCC->AHB1ENR |= RCC_AHB1ENR_GPIO##_1##EN |\
                                                                                RCC_AHB1ENR_GPIO##_2##EN
#define PORT_ENABLE_CLOCK3(_1, _2, _3)                          RCC->AHB1ENR |= RCC_AHB1ENR_GPIO##_1##EN |\
                                                                                RCC_AHB1ENR_GPIO##_2##EN |\
                                                                                RCC_AHB1ENR_GPIO##_3##EN
#define PORT_ENABLE_CLOCK4(_1, _2, _3, _4)                      RCC->AHB1ENR |= RCC_AHB1ENR_GPIO##_1##EN |\
                                                                                RCC_AHB1ENR_GPIO##_2##EN |\
                                                                                RCC_AHB1ENR_GPIO##_3##EN |\
                                                                                RCC_AHB1ENR_GPIO##_4##EN

#define PORT_ENABLE_CLOCK5(_1, _2, _3, _4, _5)                  RCC->AHB1ENR |= RCC_AHB1ENR_GPIO##_1##EN |\
                                                                                RCC_AHB1ENR_GPIO##_2##EN |\
                                                                                RCC_AHB1ENR_GPIO##_3##EN |\
                                                                                RCC_AHB1ENR_GPIO##_4##EN |\
                                                                                RCC_AHB1ENR_GPIO##_5##EN

#define PORT_ENABLE_CLOCK6(_1, _2, _3, _4, _5, _6)              RCC->AHB1ENR |= RCC_AHB1ENR_GPIO##_1##EN |\
                                                                                RCC_AHB1ENR_GPIO##_2##EN |\
                                                                                RCC_AHB1ENR_GPIO##_3##EN |\
                                                                                RCC_AHB1ENR_GPIO##_4##EN |\
                                                                                RCC_AHB1ENR_GPIO##_5##EN |\
                                                                                RCC_AHB1ENR_GPIO##_6##EN

#define PORT_ENABLE_CLOCK7(_1, _2, _3, _4, _5, _6, _7)          RCC->AHB1ENR |= RCC_AHB1ENR_GPIO##_1##EN |\
                                                                                RCC_AHB1ENR_GPIO##_2##EN |\
                                                                                RCC_AHB1ENR_GPIO##_3##EN |\
                                                                                RCC_AHB1ENR_GPIO##_4##EN |\
                                                                                RCC_AHB1ENR_GPIO##_5##EN |\
                                                                                RCC_AHB1ENR_GPIO##_6##EN |\
                                                                                RCC_AHB1ENR_GPIO##_7##EN

#define PORT_ENABLE_CLOCK8(_1, _2, _3, _4, _5, _6, _7, _8)      RCC->AHB1ENR |= RCC_AHB1ENR_GPIO##_1##EN |\
                                                                                RCC_AHB1ENR_GPIO##_2##EN |\
                                                                                RCC_AHB1ENR_GPIO##_3##EN |\
                                                                                RCC_AHB1ENR_GPIO##_4##EN |\
                                                                                RCC_AHB1ENR_GPIO##_5##EN |\
                                                                                RCC_AHB1ENR_GPIO##_6##EN |\
                                                                                RCC_AHB1ENR_GPIO##_7##EN |\
                                                                                RCC_AHB1ENR_GPIO##_8##EN


