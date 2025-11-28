#include "main.h"
#include "app_lorawan.h"
#include "usart.h"
#include "gpio.h"
#include <stdint.h>
#include <stddef.h>
TIM_HandleTypeDef htim2;

/* USER CODE BEGIN PV */
void pin_release(GPIO_TypeDef *port, uint16_t pin);
void pin_drive_low(GPIO_TypeDef *port, uint16_t pin);
void pin_drive_high_or_release(GPIO_TypeDef *port, uint16_t pin);
void apply_event(const void *e); /* forward, cast to (const Ev*) in implementation */
void start_playback(void);
void stop_playback(void);

/* --- Configuration macros --- */
#define OPEN_DRAIN 1           /* 1 = open-drain safe mode; 0 = push-pull */
#define GAP_US 1500000ULL      /* configurable gap between cycles (microseconds) */

/* --- Pins (PB6/PB7) --- */
#define PIN_CH0_PORT GPIOB
#define PIN_CH0_PIN  GPIO_PIN_6   /* PB6 */
#define PIN_CH1_PORT GPIOB
#define PIN_CH1_PIN  GPIO_PIN_7   /* PB7 */

/* Event structure */
typedef struct { uint64_t t_us; uint8_t ch0; uint8_t ch1; } Ev;

/* --- Your event table (as provided) --- */
static const Ev events[] = {
  {      0ULL, 1, 1},
  {  35866ULL, 1, 1},
  {1185173ULL, 0, 1},
  {1621935ULL, 1, 1},
  {1633276ULL, 1, 0},
  {1984619ULL, 1, 1},
  {2001909ULL, 0, 1},
  {2020489ULL, 1, 1},
};
static const size_t N_EVENTS = sizeof(events)/sizeof(events[0]);
static const uint64_t CYCLE_US = 2020489ULL;

/* TIM state */
volatile int event_index = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_TIM2_Init(void);
int main(void)
{
  HAL_Init();
  SystemClock_Config();  
 GPIO_InitTypeDef GPIO_InitStruct = {0};
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pins : PB7 PB6 */
  GPIO_InitStruct.Pin = GPIO_PIN_7|GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
  MX_TIM2_Init();
  GPIO_InitTypeDef g = {0};
  g.Pin = PIN_CH0_PIN | PIN_CH1_PIN;
  g.Mode = GPIO_MODE_INPUT;
  g.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &g);
  HAL_TIM_Base_Start(&htim2);
  MX_LoRaWAN_Init();
  while (1)
  {
    MX_LoRaWAN_Process();
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

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();
  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSE|RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = RCC_MSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_11;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the SYSCLKSource, HCLK, PCLK1 and PCLK2 clocks dividers
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK3|RCC_CLOCKTYPE_HCLK
                              |RCC_CLOCKTYPE_SYSCLK|RCC_CLOCKTYPE_PCLK1
                              |RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.AHBCLK3Divider = RCC_SYSCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
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
  htim2.Init.Prescaler = 47;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 0xffffffff;
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
  if (HAL_TIM_OC_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_TIMING;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_OC_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  HAL_TIM_MspPostInit(&htim2);

}

void pin_release(GPIO_TypeDef *port, uint16_t pin) {
  GPIO_InitTypeDef g = {0};
  g.Pin = pin;
  g.Mode = GPIO_MODE_INPUT;
  g.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(port, &g);
}

void pin_drive_low(GPIO_TypeDef *port, uint16_t pin) {
#if OPEN_DRAIN
  GPIO_InitTypeDef g = {0};
  g.Pin = pin;
  g.Mode = GPIO_MODE_OUTPUT_OD;
  g.Pull = GPIO_NOPULL;
  g.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(port, &g);
  HAL_GPIO_WritePin(port, pin, GPIO_PIN_RESET);
#else
  GPIO_InitTypeDef g = {0};
  g.Pin = pin;
  g.Mode = GPIO_MODE_OUTPUT_PP;
  g.Pull = GPIO_NOPULL;
  g.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(port, &g);
  HAL_GPIO_WritePin(port, pin, GPIO_PIN_RESET);
#endif
}

void pin_drive_high_or_release(GPIO_TypeDef *port, uint16_t pin) {
#if OPEN_DRAIN
  /* In open-drain mode: release to Hi-Z and let pull-up create HIGH */
  pin_release(port, pin);
#else
  GPIO_InitTypeDef g = {0};
  g.Pin = pin;
  g.Mode = GPIO_MODE_OUTPUT_PP;
  g.Pull = GPIO_NOPULL;
  g.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(port, &g);
  HAL_GPIO_WritePin(port, pin, GPIO_PIN_SET);
#endif
}

void apply_event(const void *ptr) {
  const Ev *e = (const Ev *)ptr;
  if (e->ch0 == 0) pin_drive_low(PIN_CH0_PORT, PIN_CH0_PIN);
  else pin_drive_high_or_release(PIN_CH0_PORT, PIN_CH0_PIN);

  if (e->ch1 == 0) pin_drive_low(PIN_CH1_PORT, PIN_CH1_PIN);
  else pin_drive_high_or_release(PIN_CH1_PORT, PIN_CH1_PIN);
}

static inline uint32_t us_to_ticks(uint64_t us) { return (uint32_t)us; }

/* TIM2 OC callback (called from HAL ISR) */
void HAL_TIM_OC_DelayElapsedCallback(TIM_HandleTypeDef *htim) {
  if (htim->Instance != TIM2) return;
  apply_event(&events[event_index]);

  /* advance to next event */
  event_index++;
  if (event_index >= (int)N_EVENTS) {
    /* end of cycle: release pins and schedule next cycle after GAP_US */
    pin_release(PIN_CH0_PORT, PIN_CH0_PIN);
    pin_release(PIN_CH1_PORT, PIN_CH1_PIN);

    __HAL_TIM_SET_COUNTER(&htim2, 0);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, us_to_ticks(CYCLE_US + GAP_US));
    event_index = 0;
  } else {
    uint32_t cval = us_to_ticks(events[event_index].t_us);
    if (cval == 0) cval = 1; /* ensure compare != 0 so ISR fires reliably */
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, cval);
  }
}

void start_playback(void) {
  event_index = 0;

  /* release pins */
  pin_release(PIN_CH0_PORT, PIN_CH0_PIN);
  pin_release(PIN_CH1_PORT, PIN_CH1_PIN);

  __HAL_TIM_SET_COUNTER(&htim2, 0);

  /* if first event time is 0, apply it immediately and advance index */
  if (events[0].t_us == 0) {
    apply_event(&events[0]);
    event_index = 1;
  }

  uint32_t next = (event_index < (int)N_EVENTS) ? us_to_ticks(events[event_index].t_us) : us_to_ticks(CYCLE_US + GAP_US);
  if (next == 0) next = 1;
  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, next);

  HAL_TIM_Base_Start(&htim2);
  HAL_TIM_OC_Start_IT(&htim2, TIM_CHANNEL_1);
}

void stop_playback(void) {
  HAL_TIM_OC_Stop_IT(&htim2, TIM_CHANNEL_1);
  pin_release(PIN_CH0_PORT, PIN_CH0_PIN);
  pin_release(PIN_CH1_PORT, PIN_CH1_PIN);
}
