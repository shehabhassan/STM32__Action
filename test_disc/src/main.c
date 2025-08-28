#define GPIO_project            1
#define Nvic_project            2
#define SYSTICK_Project         3

#define Run_project             SYSTICK_Project            


#if Run_project == GPIO_project
#include "stm32f4xx_hal.h"

// Define LED pins (PD12=Green LED, PD14=Red LED)
#define LED_GREEN_PIN      GPIO_PIN_12
#define LED_RED_PIN        GPIO_PIN_14
#define LED_PORT           GPIOD
#define LED_GPIO_CLK_ENABLE()  __HAL_RCC_GPIOD_CLK_ENABLE()
#define pushButton_CLK_ENABLE() __HAL_RCC_GPIOA_CLK_ENABLE()
#define pushButton_PIN     GPIO_PIN_0

void LED_Init();

int main(void) {
  HAL_Init();
  pushButton_CLK_ENABLE();
  LED_Init();
  HAL_GPIO_Init(pushButton_PIN,GPIO_MODE_INPUT);
int state_pin = GPIO_PIN_RESET;
  while (1) {
    // Toggle both LEDs simultaneously
    // HAL_GPIO_TogglePin(LED_PORT, LED_GREEN_PIN | LED_RED_PIN);
      HAL_GPIO_TogglePin(LED_PORT, LED_GREEN_PIN );
      if(HAL_GPIO_ReadPin(GPIOA,pushButton_PIN)==GPIO_PIN_SET)
      {
        state_pin = !state_pin;
        HAL_GPIO_WritePin(GPIOD,GPIO_PIN_14,state_pin); // Turn off Red LED
        HAL_GPIO_WritePin(GPIOD,GPIO_PIN_13,!state_pin); // Turn off Orange LED
    }
      
      
    HAL_Delay(500);  // 500ms delay (half-second)
  }
}

void LED_Init() {
  LED_GPIO_CLK_ENABLE();  // Enable GPIOD clock
  
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  GPIO_InitStruct.Pin = LED_GREEN_PIN | LED_RED_PIN;  // Configure both pins
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;         // Push-Pull output mode
  GPIO_InitStruct.Pull = GPIO_NOPULL;                 // No pull resistors needed
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;       // High speed (100MHz)
  
  HAL_GPIO_Init(LED_PORT, &GPIO_InitStruct);  // Initialize GPIO

  GPIO_InitTypeDef LED_blue = {0};
    LED_blue.Pin =GPIO_PIN_13;
    LED_blue.Mode =GPIO_MODE_OUTPUT_PP ;
    LED_blue.Pull = GPIO_NOPULL;
    LED_blue.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOD,&LED_blue); // init GPIO 
}

// Optional: Default SysTick handler (already handled by HAL)
void SysTick_Handler(void) {
  HAL_IncTick();
}

/*************************************************************** */

#elif Run_project == Nvic_project

/* main.c - Corrected GPIO Interrupt Example for STM32F407G Discovery Board */
/* main.c - Working GPIO Interrupt Example for 144MHz Clock */
/* main.c - Guaranteed Working Interrupt Example for STM32F407G-DISC1 */
/* main.c - Complete, Verified Interrupt Example for STM32F407G-DISC1 */
#include "stm32f4xx_hal.h"

// Pin definitions (Discovery board specific)
#define LED_GREEN_PIN     GPIO_PIN_12  // PD12 = Green LED (LD4)
#define LED_RED_PIN       GPIO_PIN_14  // PD14 = Red LED (LD3)
#define LED_PORT          GPIOD
#define USER_BUTTON_PIN   GPIO_PIN_0   // PA0 = User Button (B1)
#define USER_BUTTON_PORT  GPIOA

// Function prototypes
void SystemClock_Config(void);
static void Error_Handler(void);
void EXTI0_IRQHandler(void);
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);
void SysTick_Handler(void);

int main(void) {
  // 1. Initialize HAL library
  HAL_Init();
  
  // 2. Configure system clock (144MHz - works without OverDrive)
  SystemClock_Config();
  
  // 3. CRITICAL: Enable SYSCFG clock FIRST (MOST IMPORTANT STEP!)
  __HAL_RCC_SYSCFG_CLK_ENABLE();
  
  // 4. Enable GPIO clocks
  __HAL_RCC_GPIOA_CLK_ENABLE();  // For user button
  __HAL_RCC_GPIOD_CLK_ENABLE();  // For LEDs

  // 5. Configure LEDs (PD12=Green, PD14=Red)
  GPIO_InitTypeDef gpio = {0};
  gpio.Pin = LED_GREEN_PIN | LED_RED_PIN;
  gpio.Mode = GPIO_MODE_OUTPUT_PP;
  gpio.Pull = GPIO_NOPULL;
  gpio.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(LED_PORT, &gpio);
  
  // Turn off LEDs initially
  HAL_GPIO_WritePin(LED_PORT, LED_GREEN_PIN, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(LED_PORT, LED_RED_PIN, GPIO_PIN_RESET);

  // 6. Configure user button (PA0) for interrupts
  gpio.Pin = USER_BUTTON_PIN;
  gpio.Mode = GPIO_MODE_IT_FALLING;  // Interrupt on falling edge
  gpio.Pull = GPIO_PULLDOWN;           // Internal pull-up (button connects to GND)
  gpio.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(USER_BUTTON_PORT, &gpio);

  // 7. Configure NVIC for EXTI line 0 (PA0 = EXTI0)
  HAL_NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4); // 16 priority levels
  HAL_NVIC_SetPriority(EXTI0_IRQn, 1, 0);  // Priority level 1
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);          // Enable the interrupt

  printf("\n--- SysTick Configuration ---\n");
  printf("SysTick->LOAD:  %lu\n", SysTick->LOAD);  // Reload value
  printf("SysTick->VAL:   %lu\n", SysTick->VAL);   // Current value
  printf("SysTick->CTRL:  0x%08lX\n", SysTick->CTRL); // Control register
  printf("HAL_GetTick():  %lu ms\n", HAL_GetTick());
  printf("SystemCoreClock: %lu Hz\n", SystemCoreClock);

  // 8. Main loop - blink red LED slowly while waiting for button presses
  while (1) {
    HAL_GPIO_TogglePin(LED_PORT, LED_RED_PIN);
    HAL_Delay(500);  // Red LED blinks every 500ms
  }
}

/**
 * @brief System Clock Configuration (144MHz)
 */
void SystemClock_Config(void) {
  RCC_OscInitTypeDef osc_init = {0};
  RCC_ClkInitTypeDef clk_init = {0};
  HAL_StatusTypeDef status;

  __HAL_RCC_PWR_CLK_ENABLE();
  
  // Check if voltage scaling succeeded
  status = HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);
  if (status != HAL_OK) {
    Error_Handler();
  }

  osc_init.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  osc_init.HSEState = RCC_HSE_ON;
  osc_init.PLL.PLLState = RCC_PLL_ON;
  osc_init.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  osc_init.PLL.PLLM = 8;
  osc_init.PLL.PLLN = 288;  // 144MHz (288/2 = 144)
  osc_init.PLL.PLLP = RCC_PLLP_DIV2;
  osc_init.PLL.PLLQ = 7;
  
  if (HAL_RCC_OscConfig(&osc_init) != HAL_OK) {
    Error_Handler();
  }
  
  clk_init.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                       RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  clk_init.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  clk_init.AHBCLKDivider = RCC_SYSCLK_DIV1;
  clk_init.APB1CLKDivider = RCC_HCLK_DIV4;
  clk_init.APB2CLKDivider = RCC_HCLK_DIV2;
  
  if (HAL_RCC_ClockConfig(&clk_init, FLASH_LATENCY_4) != HAL_OK) {
    Error_Handler();
  }
}

/**
 * @brief EXTI line 0 interrupt handler
 */
void EXTI0_IRQHandler(void) {
  HAL_GPIO_EXTI_IRQHandler(USER_BUTTON_PIN);
}

 static uint8_t led_state = GPIO_PIN_SET;

/**
 * @brief GPIO EXTI callback function
 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
  if (GPIO_Pin == USER_BUTTON_PIN) {
   led_state = !led_state;
    // Toggle green LED when button is pressed
    HAL_GPIO_TogglePin(LED_PORT, LED_GREEN_PIN);
    
    // Quick visual feedback with red LED
    HAL_GPIO_WritePin(LED_PORT, LED_RED_PIN, led_state);
    // HAL_Delay(50);
    // HAL_GPIO_WritePin(LED_PORT, LED_RED_PIN, !);
  }
}

/**
 * @brief Error handler (blinks red LED rapidly)
 */
__attribute__((used)) static void Error_Handler(void) {
  while (1) {
    // Error pattern: 3 fast blinks
    for (int i = 0; i < 3; i++) {
      HAL_GPIO_WritePin(LED_PORT, LED_RED_PIN, GPIO_PIN_SET);
      HAL_Delay(100);
      HAL_GPIO_WritePin(LED_PORT, LED_RED_PIN, GPIO_PIN_RESET);
      HAL_Delay(100);
    }
    HAL_Delay(1000); // Pause between error codes
  }
}

/**
 * @brief SysTick handler (required for HAL_Delay)
 */
void SysTick_Handler(void) {
  HAL_IncTick();
}


#elif Run_project == SYSTICK_Project
/* main.c - Working SysTick Example with Debug Output */
#include "stm32f4xx_hal.h"
#include <stdio.h>

UART_HandleTypeDef huart2;

void SystemClock_Config(void);
static void MX_USART2_UART_Init(void);

// Redirect stdout to UART2
int _write(int file, char *ptr, int len) {
    HAL_UART_Transmit(&huart2, (uint8_t*)ptr, len, HAL_MAX_DELAY);
    return len;
}

int main(void) {
    HAL_Init();
    SystemClock_Config();
    MX_USART2_UART_Init();

    printf("STM32F407G Discovery Board Started!\r\n");

    float sensor_value = 123.45;
    uint32_t counter = 0;

    while (1) {
        printf("Counter: %lu, Value: %.2f\r\n", counter++, sensor_value);
        HAL_Delay(1000);  // Send data every second
    }
}

// USART2 Initialization
static void MX_USART2_UART_Init(void) {
    huart2.Instance = USART2;
    huart2.Init.BaudRate = 115200;
    huart2.Init.WordLength = UART_WORDLENGTH_8B;
    huart2.Init.StopBits = UART_STOPBITS_1;
    huart2.Init.Parity = UART_PARITY_NONE;
    huart2.Init.Mode = UART_MODE_TX_RX;
    huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart2.Init.OverSampling = UART_OVERSAMPLING_16;
    HAL_UART_Init(&huart2);
}

// System Clock Configuration (generated by STM32CubeMX)
void SystemClock_Config(void) {
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    // Configure the main internal regulator output voltage
    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    // Initializes the CPU, AHB and APB busses clocks
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState = RCC_HSE_ON;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLM = 8;
    RCC_OscInitStruct.PLL.PLLN = 336;
    RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
    RCC_OscInitStruct.PLL.PLLQ = 7;
    HAL_RCC_OscConfig(&RCC_OscInitStruct);

    // Initializes the CPU, AHB and APB busses clocks
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                                |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;
    HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5);
}

// Error handler
void Error_Handler(void) {
    while(1);
}


#endif