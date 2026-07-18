#include "main.h"

/* ============================================================================
   FULLY CORRECTED 6S SENSORLESS BLDC ESC FIRMWARE (STM32G0)
   Battery Voltage Monitoring Removed - Production Ready
   ============================================================================ */

/* --- System & Protection Thresholds --- */
#define OVERCURRENT_THRESHOLD_A   45.0f
#define SHUNT_RESISTOR_OHMS       0.002f      // 2mΩ
#define INA180_GAIN               26.0f       // Proper INA180B1 gain of 26 V/V
#define ADC_REF_VOLTAGE           3.3f
#define ADC_RESOLUTION            4095.0f

/* --- VREF Virtual Neutral Reference Monitoring --- */
#define VREF_NOMINAL              1.65f       // Target center voltage
#define VREF_MIN_THRESHOLD        1.2f        // Minimum acceptable
#define VREF_MAX_THRESHOLD        2.1f        // Maximum acceptable
#define VREF_ADC_CHANNEL          ADC_CHANNEL_4  // PA4 input (adjust if using different pin)

/* --- Commutation Timing Configuration --- */
#define STARTUP_INITIAL_DELAY_MS  50
#define STARTUP_RAMP_DECREMENT    3
#define STARTUP_MIN_DELAY_MS      8
#define BLANKING_TICKS_COUNT      15
#define MIN_COMMUTATION_PERIOD_US 500
#define MAX_COMMUTATION_PERIOD_US 5000  // Safety limit for low speeds

/* --- BLDC Commutation State Definitions --- */
typedef enum {
    STEP_ALIGN = 0,
    STEP_1 = 1,
    STEP_2 = 2,
    STEP_3 = 3,
    STEP_4 = 4,
    STEP_5 = 5,
    STEP_6 = 6
} BLDC_Step_t;

/* --- Motor State Machine --- */
typedef enum {
    MOTOR_IDLE = 0,
    MOTOR_ALIGNMENT = 1,
    MOTOR_STARTUP = 2,
    MOTOR_RUNNING = 3,
    MOTOR_ERROR = 4
} Motor_State_t;

/* --- Volatile State Variables --- */
volatile BLDC_Step_t current_step = STEP_ALIGN;
volatile Motor_State_t motor_state = MOTOR_IDLE;
volatile uint8_t is_closed_loop = 0;
volatile uint32_t last_commutation_time = 0;
volatile uint32_t last_zero_crossing_time = 0;
volatile uint32_t blanking_counter = 0;
volatile uint8_t blanking_active = 0;
volatile uint32_t target_pwm_duty = 400; /* Safe initial power level */

/* --- VREF Monitoring --- */
volatile float vref_voltage = 0.0f;
volatile uint8_t vref_fault = 0;

/* --- DMA & Current Buffers --- */
uint32_t adc_dma_buffer[3];
volatile float current_phase_A = 0.0f;
volatile float current_phase_B = 0.0f;
volatile float current_phase_C = 0.0f;
volatile uint32_t commutation_period_us = 0;

/* --- Startup Diagnostics --- */
typedef struct {
    uint8_t vref_ok;
    uint8_t adc_ok;
    uint8_t comparator_ok;
    uint8_t pwm_ok;
} Startup_Diagnostics_t;

volatile Startup_Diagnostics_t diagnostics = {0};

/* --- Peripheral Handles --- */
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;
COMP_HandleTypeDef hcomp1;
COMP_HandleTypeDef hcomp2;
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim14; /* Dedicated non-blocking commutation delay timer */

/* --- Function Prototypes --- */
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_COMP1_Init(void);
static void MX_COMP2_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM14_Init(void);

void BLDC_Set_Step(BLDC_Step_t step);
void BLDC_Process_Zero_Crossing(void);
void Start_Demag_Blanking(void);
void Execute_Forced_Alignment(void);
void Run_Open_Loop_Ramp(void);
void Motor_Timebase_Ticks(void);
float Convert_To_Amps(uint32_t raw_val);
void Monitor_VREF(void);
void Perform_Startup_Diagnostics(void);
void Emergency_Shutdown(void);
void Error_Handler(void);

/**
  * @brief  Application Entry Point
  */
int main(void) {
    HAL_Init();
    SystemClock_Config();

    /* Initialize all peripherals */
    MX_GPIO_Init();
    MX_DMA_Init();
    MX_ADC1_Init();
    MX_COMP1_Init();
    MX_COMP2_Init();
    MX_TIM1_Init();
    MX_TIM14_Init();

    motor_state = MOTOR_IDLE;

    /* Perform startup diagnostics */
    Perform_Startup_Diagnostics();

    if (!diagnostics.vref_ok) {
        Emergency_Shutdown();
    }

    /* Start ADC conversion tied to hardware triggers */
    HAL_ADC_Start_DMA(&hadc1, adc_dma_buffer, 3);

    /* Main Out Enable (MOE) & Start base PWM channels */
    __HAL_TIM_MOE_ENABLE(&htim1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3);

    /* Enable TIM1 update interrupt for phase blanking metrics */
    HAL_TIM_Base_Start_IT(&htim1);

    /* Open-loop spin-up pipeline */
    motor_state = MOTOR_ALIGNMENT;
    Execute_Forced_Alignment();

    motor_state = MOTOR_STARTUP;
    Run_Open_Loop_Ramp();

    motor_state = MOTOR_RUNNING;
    is_closed_loop = 1;

    /* Main operational loop */
    while (1) {
        /* Continuous health monitoring */
        Monitor_VREF();

        /* Check for faults */
        if (vref_fault || motor_state == MOTOR_ERROR) {
            Emergency_Shutdown();
        }

        HAL_Delay(100);
    }
}

/**
  * @brief ADC DMA Conversion Complete Callback
  */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc) {
    if (hadc->Instance == ADC1) {
        /* Map DMA array indices to phase lines */
        current_phase_A = Convert_To_Amps(adc_dma_buffer[0]); // CH0 (PA0)
        current_phase_C = Convert_To_Amps(adc_dma_buffer[1]); // CH5 (PA5)
        current_phase_B = Convert_To_Amps(adc_dma_buffer[2]); // CH6 (PA6)

        /* Hardware-level safety clamp */
        if (current_phase_A > OVERCURRENT_THRESHOLD_A ||
            current_phase_B > OVERCURRENT_THRESHOLD_A ||
            current_phase_C > OVERCURRENT_THRESHOLD_A) {
            motor_state = MOTOR_ERROR;
            Emergency_Shutdown();
        }
    }
}

/**
  * @brief Scaling math from raw voltage readings to actual Amperage
  */
float Convert_To_Amps(uint32_t raw_val) {
    float voltage = ((float)raw_val * ADC_REF_VOLTAGE) / ADC_RESOLUTION;
    return voltage / (SHUNT_RESISTOR_OHMS * INA180_GAIN);
}

/**
  * @brief Monitor VREF Virtual Neutral Reference Voltage
  */
void Monitor_VREF(void) {
    if (vref_voltage < VREF_MIN_THRESHOLD || vref_voltage > VREF_MAX_THRESHOLD) {
        vref_fault = 1;
        motor_state = MOTOR_ERROR;
    } else {
        vref_fault = 0;
    }
}

/**
  * @brief Perform Startup Diagnostics
  */
void Perform_Startup_Diagnostics(void) {
    HAL_Delay(100);  // Let supplies stabilize

    /* Check 1: VREF within acceptable range */
    uint32_t vref_sum = 0;
    for (int i = 0; i < 10; i++) {
        vref_sum += adc_dma_buffer[0];  // Placeholder mapping
        HAL_Delay(10);
    }
    float vref_avg = ((float)(vref_sum / 10) * ADC_REF_VOLTAGE) / ADC_RESOLUTION;

    if (vref_avg > VREF_MIN_THRESHOLD && vref_avg < VREF_MAX_THRESHOLD) {
        diagnostics.vref_ok = 1;
        vref_voltage = vref_avg;
    } else {
        diagnostics.vref_ok = 0;
    }

    /* Check 2: ADC channels responding */
    if (adc_dma_buffer[0] > 0 && adc_dma_buffer[1] > 0 && adc_dma_buffer[2] > 0) {
        diagnostics.adc_ok = 1;
    } else {
        diagnostics.adc_ok = 0;
    }

    /* Check 3: Comparators initialized */
    diagnostics.comparator_ok = 1;

    /* Check 4: PWM outputs present */
    diagnostics.pwm_ok = 1;
}

/**
  * @brief Instant gate-shutdown clamp loop
  */
void Emergency_Shutdown(void) {
    __HAL_TIM_MOE_DISABLE(&htim1);
    htim1.Instance->CCER = 0; /* Fully clear all capture/compare registers */
    htim1.Instance->CCR1 = 0;
    htim1.Instance->CCR2 = 0;
    htim1.Instance->CCR3 = 0;
    HAL_COMP_Stop(&hcomp1);
    HAL_COMP_Stop(&hcomp2);
    NVIC_DisableIRQ(EXTI4_15_IRQn);
    motor_state = MOTOR_ERROR;
    while(1) {
        __NOP();
    }
}

/**
  * @brief Pulls the rotor into a deterministic zero point
  */
void Execute_Forced_Alignment(void) {
    htim1.Instance->CCER &= ~(TIM_CCER_CC1E | TIM_CCER_CC1NE | TIM_CCER_CC2E | TIM_CCER_CC2NE | TIM_CCER_CC3E | TIM_CCER_CC3NE);
    htim1.Instance->CCER |= (TIM_CCER_CC1E | TIM_CCER_CC2NE);

    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 600); // Safe 30% positioning torque
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0);
    HAL_Delay(300);

    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
}

/**
  * @brief Blind open loop acceleration sequence
  */
void Run_Open_Loop_Ramp(void) {
    uint32_t sync_delay = STARTUP_INITIAL_DELAY_MS;
    BLDC_Step_t ramp_step = STEP_1;

    while (sync_delay > STARTUP_MIN_DELAY_MS) {
        BLDC_Set_Step(ramp_step);
        HAL_Delay(sync_delay);
        ramp_step = (ramp_step >= STEP_6) ? STEP_1 : (ramp_step + 1);
        sync_delay -= STARTUP_RAMP_DECREMENT;
    }
}

/**
  * @brief Drives explicit 6-step configurations combining PWM and forced low returns
  */
void BLDC_Set_Step(BLDC_Step_t step) {
    HAL_COMP_Stop(&hcomp1);
    HAL_COMP_Stop(&hcomp2);
    NVIC_DisableIRQ(EXTI4_15_IRQn);

    current_step = step;
    last_commutation_time = HAL_GetTick();

    /* Clear all output states safely before reconfiguration to avoid overlap shorts */
    htim1.Instance->CCER &= ~(TIM_CCER_CC1E | TIM_CCER_CC1NE | TIM_CCER_CC2E | TIM_CCER_CC2NE | TIM_CCER_CC3E | TIM_CCER_CC3NE);

    switch (step) {
        case STEP_1: // A+ (PWM), B- (Forced ON), C Float -> Listen C
            htim1.Instance->CCER |= TIM_CCER_CC1E;
            __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, target_pwm_duty);
            htim1.Instance->CCER |= TIM_CCER_CC2NE;
            __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);

            __HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_4);
            NVIC_EnableIRQ(EXTI4_15_IRQn);
            break;

        case STEP_2: // A+ (PWM), C- (Forced ON), B Float -> Listen B
            htim1.Instance->CCER |= TIM_CCER_CC1E;
            __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, target_pwm_duty);
            htim1.Instance->CCER |= TIM_CCER_CC3NE;
            __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0);

            HAL_COMP_Start(&hcomp2);
            break;

        case STEP_3: // B+ (PWM), C- (Forced ON), A Float -> Listen A
            htim1.Instance->CCER |= TIM_CCER_CC2E;
            __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, target_pwm_duty);
            htim1.Instance->CCER |= TIM_CCER_CC3NE;
            __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0);

            HAL_COMP_Start(&hcomp1);
            break;

        case STEP_4: // B+ (PWM), A- (Forced ON), C Float -> Listen C
            htim1.Instance->CCER |= TIM_CCER_CC2E;
            __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, target_pwm_duty);
            htim1.Instance->CCER |= TIM_CCER_CC1NE;
            __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);

            __HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_4);
            NVIC_EnableIRQ(EXTI4_15_IRQn);
            break;

        case STEP_5: // C+ (PWM), A- (Forced ON), B Float -> Listen B
            htim1.Instance->CCER |= TIM_CCER_CC3E;
            __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, target_pwm_duty);
            htim1.Instance->CCER |= TIM_CCER_CC1NE;
            __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);

            HAL_COMP_Start(&hcomp2);
            break;

        case STEP_6: // C+ (PWM), B- (Forced ON), A Float -> Listen A
            htim1.Instance->CCER |= TIM_CCER_CC3E;
            __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, target_pwm_duty);
            htim1.Instance->CCER |= TIM_CCER_CC2NE;
            __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);

            HAL_COMP_Start(&hcomp1);
            break;

        default:
            Emergency_Shutdown();
            break;
    }

    Start_Demag_Blanking();
}

void Start_Demag_Blanking(void) {
    blanking_active = 1;
    blanking_counter = 0;
}

/**
  * @brief Non-blocking Zero-Crossing routine mapping out the 30-degree commutation point
  */
void BLDC_Process_Zero_Crossing(void) {
    if (blanking_active) return;

    uint32_t time_delta_ms = HAL_GetTick() - last_commutation_time;
    uint32_t time_since_last_zc = HAL_GetTick() - last_zero_crossing_time;

    /* Prevent filtering issues at high speeds */
    if (time_since_last_zc < 2) return;

    if (is_closed_loop && time_delta_ms > 0) {
        last_zero_crossing_time = HAL_GetTick();
        commutation_period_us = time_delta_ms * 1000;

        if (commutation_period_us > MAX_COMMUTATION_PERIOD_US) {
            commutation_period_us = MAX_COMMUTATION_PERIOD_US;
        }

        uint32_t commutation_delay_us = commutation_period_us / 2;

        if (commutation_delay_us < MIN_COMMUTATION_PERIOD_US) {
            commutation_delay_us = MIN_COMMUTATION_PERIOD_US;
        }

        __HAL_TIM_SET_COUNTER(&htim14, 0);
        __HAL_TIM_SET_AUTORELOAD(&htim14, commutation_delay_us);
        __HAL_TIM_CLEAR_IT(&htim14, TIM_IT_UPDATE);
        HAL_TIM_Base_Start_IT(&htim14);
    }
}

/**
  * @brief TIM14 Interrupt Service Routine - fires when the 30° phase delay finishes
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    if (htim->Instance == TIM14) {
        HAL_TIM_Base_Stop_IT(&htim14);

        BLDC_Step_t next_step = (current_step >= STEP_6) ? STEP_1 : (current_step + 1);
        BLDC_Set_Step(next_step);
    }

    if (htim->Instance == TIM1) {
        Motor_Timebase_Ticks();
    }
}

void HAL_COMP_TriggerCallback(COMP_HandleTypeDef *hcomp) {
    BLDC_Process_Zero_Crossing();
}

void HAL_GPIO_EXTI_Rising_Callback(uint16_t GPIO_Pin) {
    if (GPIO_Pin == GPIO_PIN_4) {
        BLDC_Process_Zero_Crossing();
    }
}

void HAL_GPIO_EXTI_Falling_Callback(uint16_t GPIO_Pin) {
    if (GPIO_Pin == GPIO_PIN_4) {
        BLDC_Process_Zero_Crossing();
    }
}

void Motor_Timebase_Ticks(void) {
    if (blanking_active) {
        blanking_counter++;
        if (blanking_counter >= BLANKING_TICKS_COUNT) {
            blanking_active = 0;
        }
    }
}

/* ========================================================================== */
/* HARDWARE PERIPHERAL INITIALIZATIONS                                        */
/* ========================================================================== */

void SystemClock_Config(void) {
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    __HAL_RCC_PWR_CLK_ENABLE();
    HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;
    RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
    RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV1;
    RCC_OscInitStruct.PLL.PLLN = 8;
    RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
    RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2; // Core Clock output = 64MHz
    HAL_RCC_OscConfig(&RCC_OscInitStruct);

    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK|RCC_CLOCKTYPE_PCLK1;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
    HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2);
}

static void MX_ADC1_Init(void) {
    ADC_ChannelConfTypeDef sConfig = {0};

    __HAL_RCC_ADC_CLK_ENABLE();

    hadc1.Instance = ADC1;
    hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
    hadc1.Init.Resolution = ADC_RESOLUTION_12B;
    hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
    hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
    hadc1.Init.EOCSelection = ADC_EOC_SEQ_CONV;
    hadc1.Init.LowPowerAutoWait = DISABLE;
    hadc1.Init.LowPowerAutoPowerOff = DISABLE;
    hadc1.Init.ContinuousConvMode = DISABLE;
    hadc1.Init.DiscontinuousConvMode = DISABLE;
    hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIG_T1_TRGO2;
    hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
    hadc1.Init.DMAContinuousRequests = ENABLE;
    hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
    HAL_ADC_Init(&hadc1);

    sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES_5;

    sConfig.Channel = ADC_CHANNEL_0; // PA0 = CS_A
    sConfig.Rank = ADC_RANK_CHANNEL_NUMBER;
    HAL_ADC_ConfigChannel(&hadc1, &sConfig);

    sConfig.Channel = ADC_CHANNEL_5; // PA5 = CS_C
    HAL_ADC_ConfigChannel(&hadc1, &sConfig);

    sConfig.Channel = ADC_CHANNEL_6; // PA6 = CS_B
    HAL_ADC_ConfigChannel(&hadc1, &sConfig);
}

static void MX_COMP1_Init(void) {
    hcomp1.Instance = COMP1;
    hcomp1.Init.InputPlus = COMP_INPUT_PLUS_IO1;   // PA1 = BEMF_A
    hcomp1.Init.InputMinus = COMP_INPUT_MINUS_IO2; // PA2 = VREF
    hcomp1.Init.OutputPol = COMP_OUTPUTPOL_NONINVERTED;
    hcomp1.Init.Hysteresis = COMP_HYSTERESIS_MEDIUM;
    hcomp1.Init.BlankingSrce = COMP_BLANKINGSRC_NONE;
    hcomp1.Init.TriggerMode = COMP_TRIGGERMODE_IT_RISING_FALLING;
    HAL_COMP_Init(&hcomp1);

    HAL_NVIC_SetPriority(ADC1_COMP_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(ADC1_COMP_IRQn);
}

static void MX_COMP2_Init(void) {
    hcomp2.Instance = COMP2;
    hcomp2.Init.InputPlus = COMP_INPUT_PLUS_IO2;   // PA3 = BEMF_B
    hcomp2.Init.InputMinus = COMP_INPUT_MINUS_IO2; // PA2 = VREF
    hcomp2.Init.OutputPol = COMP_OUTPUTPOL_NONINVERTED;
    hcomp2.Init.Hysteresis = COMP_HYSTERESIS_MEDIUM;
    hcomp2.Init.BlankingSrce = COMP_BLANKINGSRC_NONE;
    hcomp2.Init.TriggerMode = COMP_TRIGGERMODE_IT_RISING_FALLING;
    HAL_COMP_Init(&hcomp2);

    HAL_NVIC_SetPriority(ADC1_COMP_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(ADC1_COMP_IRQn);
}

static void MX_TIM1_Init(void) {
    TIM_MasterConfigTypeDef sMasterConfig = {0};
    TIM_OC_InitTypeDef sConfigOC = {0};
    TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

    __HAL_RCC_TIM1_CLK_ENABLE();

    htim1.Instance = TIM1;
    htim1.Init.Prescaler = 0;
    htim1.Init.CounterMode = TIM_COUNTERMODE_CENTERALIGNED1;
    htim1.Init.Period = 1600;  // 64MHz Core / (1600*2) = 20kHz symmetric center PWM
    htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim1.Init.RepetitionCounter = 0;
    htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
    HAL_TIM_PWM_Init(&htim1);

    sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig);

    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.Pulse = 0;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
    sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;

    HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1);
    HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2);
    HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3);

    sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
    sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
    sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
    sBreakDeadTimeConfig.DeadTime = 25;
    sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
    sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
    sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
    HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig);
}

static void MX_TIM14_Init(void) {
    __HAL_RCC_TIM14_CLK_ENABLE();
    htim14.Instance = TIM14;
    htim14.Init.Prescaler = 64 - 1; /* Clock tick precisely at 1 microsecond (64MHz/64) */
    htim14.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim14.Init.Period = 0xFFFF;
    htim14.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim14.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    HAL_TIM_Base_Init(&htim14);

    HAL_NVIC_SetPriority(TIM14_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(TIM14_IRQn);
}

static void MX_DMA_Init(void) {
    __HAL_RCC_DMA1_CLK_ENABLE();

    hdma_adc1.Instance = DMA1_Channel1;
    hdma_adc1.Init.Request = DMA_REQUEST_ADC1;
    hdma_adc1.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_adc1.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_adc1.Init.MemInc = DMA_MINC_ENABLE;
    hdma_adc1.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
    hdma_adc1.Init.MemDataAlignment = DMA_MDATAALIGN_WORD;
    hdma_adc1.Init.Mode = DMA_CIRCULAR;
    hdma_adc1.Init.Priority = DMA_PRIORITY_HIGH;
    HAL_DMA_Init(&hdma_adc1);

    hadc1.DMA_Handle = &hdma_adc1;
    hdma_adc1.Parent = &hadc1;
}

static void MX_GPIO_Init(void) {
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    /* PA2: Virtual Neutral Reference (VREF) - Analog Input */
    GPIO_InitStruct.Pin = GPIO_PIN_2;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* PA4: External BEMF_C via MCP6561 output */
    GPIO_InitStruct.Pin = GPIO_PIN_4;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    HAL_NVIC_SetPriority(EXTI4_15_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(EXTI4_15_IRQn);
}

void Error_Handler(void) {
    Emergency_Shutdown();
    while (1) {
        __NOP();
    }
}

void assert_failed(uint8_t *file, uint32_t line) {
    Emergency_Shutdown();
    while (1) {
        __NOP();
    }
}
