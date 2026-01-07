#include "stm32f411xe.h"
#include <stdint.h>
#include <stdlib.h>   // abs()

// ============================================================
// KONFIGURASI PIN
// ============================================================
// --- Input/Output umum
#define BUTTON_PIN      0   // PA0 (Trigger gacha - EXTI0)
#define LED_HIJAU_PIN   1   // PA1
#define LED_BIRU_PIN    2   // PA2
#define BUZZER_PIN      3   // PA3

// --- Motor driver (L298N)
#define MOTOR_EN_PIN    6   // PA6 (TIM3_CH1 PWM)
#define MOTOR_IN1_PIN   7   // PA7
#define MOTOR_IN2_PIN   0   // PB0

// --- Rotary Encoder (Quadrature)
// NOTE: SW pin encoder TIDAK dipakai (karena sudah ada tombol PA0).
#define ENCODER_CLK_PIN 4   // PB4 (EXTI4)  -> CLK/A
#define ENCODER_DT_PIN  5   // PB5          -> DT/B

// ============================================================
// SETTING GAME (UNO-ATTACK STYLE)
// ============================================================
#define BASE_CHANCE     20
#define ADD_CHANCE      15
#define COOLDOWN_TIME   300   // ms debounce/cooldown tombol gacha

// ============================================================
// SETTING TARGET ROTASI (DERAJAT)
// ============================================================
// Kartu 1: 90 derajat
// Kartu berikutnya: +30 derajat per fail
#define BASE_ANGLE_DEG  90.0f
#define ADD_ANGLE_DEG   30.0f

// ============================================================
// ENCODER & PID SETTING
// ============================================================
// PPR = pulses per revolution yang kamu hitung dari encoder kamu.
// Kalau encoder kamu KY-040 (mekanik) biasanya kecil (20/24 detent).
// Kalau encoder motor optical/gear bisa ratusan.
// Default pakai 600 seperti contoh kamu, tapi WAJIB disesuaikan.
#define ENCODER_PPR_DEFAULT  600U

// Toleransi berhenti (dalam pulse). Naikkan kalau motor masih hunting.
#define ENCODER_TOL_PULSES   2

// Timeout safety untuk gerak motor (ms)
#define MOVE_TIMEOUT_MS      6000

// PID default (silakan tuning)
#define KP_DEFAULT      2.0f
#define KI_DEFAULT      0.5f
#define KD_DEFAULT      0.1f

// Output PID dalam skala -255..255 (sesuai mapping PWM)
#define PID_MAX_OUTPUT  255.0f
#define PID_MIN_OUTPUT -255.0f

// PWM minimum supaya motor kuat mulai jalan (kalau perlu)
// 0 = disable (PID langsung map ke PWM)
#define MIN_EFFECTIVE_PWM    35

// ============================================================
// VARIABEL SYSTEM
// ============================================================
volatile uint32_t sysTick_ms = 0;
volatile uint8_t buttonPressed = 0;
uint32_t lastPressTime = 0;
int failCount = 0;

// --- Encoder state
volatile int32_t encoderCount = 0;  // count pulse (signed)
volatile int8_t encoderDir = 1;     // +1 CW, -1 CCW (indikatif)

// --- RNG State (LFSR)
static uint32_t lfsr = 0xACE1u;

// --- Encoder tunable runtime
static uint16_t encoderPPR = ENCODER_PPR_DEFAULT;

// ============================================================
// PID STRUCT
// ============================================================
typedef struct {
    float Kp;
    float Ki;
    float Kd;
    float setpoint;      // dalam pulse
    float integral;
    float prev_error;
    uint32_t lastTime;   // ms
} PID_Controller;

static PID_Controller pid = {
    .Kp = KP_DEFAULT,
    .Ki = KI_DEFAULT,
    .Kd = KD_DEFAULT,
    .setpoint = 0.0f,
    .integral = 0.0f,
    .prev_error = 0.0f,
    .lastTime = 0
};

// ============================================================
// PROTOTYPES
// ============================================================
void SystemClock_Config(void);
void GPIO_Init(void);
void EXTI_Init(void);
void TIM3_PWM_Init(void);
void SysTick_Init(void);

void delay_ms(uint32_t ms);
uint32_t millis(void);

// RNG
uint32_t random_lfsr(void);
uint32_t random_range(uint32_t min, uint32_t max);
void random_seed(uint32_t seed);

// Motor + PID
void motorStop(void);
void motorControl(int16_t speed);   // -255..255 (negatif = reverse)
float PID_Compute(PID_Controller* p, float current);
void resetEncoder(void);
float getEncoderAngleDeg(void);
void moveToAngleDeg(float targetDeg);

// Buzzer + LED + Game
void buzzerTone(uint16_t freq, uint32_t duration);
void startupSound(void);
void winSound(void);
void loseSound(void);
void ledOn(uint8_t pin);
void ledOff(uint8_t pin);
void blinkLED(uint8_t pin, uint8_t times, uint32_t delayTime);
void prosesGacha(void);

// ============================================================
// SYSTEM CLOCK - 100 MHz
// ============================================================
void SystemClock_Config(void) {
    RCC->CR |= RCC_CR_HSION;
    while (!(RCC->CR & RCC_CR_HSIRDY));

    FLASH->ACR |= FLASH_ACR_LATENCY_3WS;

    RCC->CR &= ~RCC_CR_PLLON;
    while (RCC->CR & RCC_CR_PLLRDY);

    RCC->PLLCFGR = 0;
    RCC->PLLCFGR |= (16 << RCC_PLLCFGR_PLLM_Pos);
    RCC->PLLCFGR |= (200 << RCC_PLLCFGR_PLLN_Pos);
    RCC->PLLCFGR |= (0 << RCC_PLLCFGR_PLLP_Pos);
    RCC->PLLCFGR &= ~RCC_PLLCFGR_PLLSRC; // HSI

    RCC->CR |= RCC_CR_PLLON;
    while (!(RCC->CR & RCC_CR_PLLRDY));

    RCC->CFGR |= RCC_CFGR_PPRE1_DIV2;
    RCC->CFGR &= ~RCC_CFGR_PPRE2;

    RCC->CFGR |= RCC_CFGR_SW_PLL;
    while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL);

    SystemCoreClock = 100000000;
}

// ============================================================
// GPIO INIT
// ============================================================
void GPIO_Init(void) {
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIOBEN;

    // --- PA0: Button Input Pull-up ---
    GPIOA->MODER &= ~(3U << (BUTTON_PIN * 2));
    GPIOA->PUPDR &= ~(3U << (BUTTON_PIN * 2));
    GPIOA->PUPDR |= (1U << (BUTTON_PIN * 2)); // pull-up

    // --- PA1: LED Hijau Output ---
    GPIOA->MODER &= ~(3U << (LED_HIJAU_PIN * 2));
    GPIOA->MODER |= (1U << (LED_HIJAU_PIN * 2));
    GPIOA->ODR &= ~(1U << LED_HIJAU_PIN);

    // --- PA2: LED Biru Output ---
    GPIOA->MODER &= ~(3U << (LED_BIRU_PIN * 2));
    GPIOA->MODER |= (1U << (LED_BIRU_PIN * 2));
    GPIOA->ODR &= ~(1U << LED_BIRU_PIN);

    // --- PA3: Buzzer Output ---
    GPIOA->MODER &= ~(3U << (BUZZER_PIN * 2));
    GPIOA->MODER |= (1U << (BUZZER_PIN * 2));
    GPIOA->ODR &= ~(1U << BUZZER_PIN);

    // --- PA6: Motor Enable (AF - TIM3_CH1) ---
    GPIOA->MODER &= ~(3U << (MOTOR_EN_PIN * 2));
    GPIOA->MODER |= (2U << (MOTOR_EN_PIN * 2));  // AF mode
    GPIOA->AFR[0] &= ~(0xF << (MOTOR_EN_PIN * 4));
    GPIOA->AFR[0] |= (2U << (MOTOR_EN_PIN * 4)); // AF2 = TIM3

    // --- PA7: Motor IN1 Output ---
    GPIOA->MODER &= ~(3U << (MOTOR_IN1_PIN * 2));
    GPIOA->MODER |= (1U << (MOTOR_IN1_PIN * 2));
    GPIOA->ODR &= ~(1U << MOTOR_IN1_PIN);

    // --- PB0: Motor IN2 Output ---
    GPIOB->MODER &= ~(3U << (MOTOR_IN2_PIN * 2));
    GPIOB->MODER |= (1U << (MOTOR_IN2_PIN * 2));
    GPIOB->ODR &= ~(1U << MOTOR_IN2_PIN);

    // --- Encoder pins: Input Pull-up (PB4, PB5) ---
    GPIOB->MODER &= ~(3U << (ENCODER_CLK_PIN * 2));
    GPIOB->PUPDR &= ~(3U << (ENCODER_CLK_PIN * 2));
    GPIOB->PUPDR |= (1U << (ENCODER_CLK_PIN * 2)); // pull-up

    GPIOB->MODER &= ~(3U << (ENCODER_DT_PIN * 2));
    GPIOB->PUPDR &= ~(3U << (ENCODER_DT_PIN * 2));
    GPIOB->PUPDR |= (1U << (ENCODER_DT_PIN * 2));  // pull-up
}

// ============================================================
// EXTI INTERRUPT
// - EXTI0: tombol gacha (PA0) falling edge
// - EXTI4: encoder CLK (PB4) rising edge, baca DT (PB5) buat arah
// ============================================================
void EXTI_Init(void) {
    RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;

    // --- EXTI0 -> PA0 ---
    SYSCFG->EXTICR[0] &= ~SYSCFG_EXTICR1_EXTI0; // PA0
    EXTI->FTSR |= EXTI_FTSR_TR0;
    EXTI->RTSR &= ~EXTI_RTSR_TR0;
    EXTI->IMR  |= EXTI_IMR_MR0;
    NVIC_SetPriority(EXTI0_IRQn, 2);
    NVIC_EnableIRQ(EXTI0_IRQn);

    // --- EXTI4 -> PB4 (Encoder CLK) ---
    SYSCFG->EXTICR[1] &= ~SYSCFG_EXTICR2_EXTI4;
    SYSCFG->EXTICR[1] |= SYSCFG_EXTICR2_EXTI4_PB; // PB4
    EXTI->RTSR |= EXTI_RTSR_TR4;   // rising edge
    EXTI->FTSR &= ~EXTI_FTSR_TR4;  // disable falling
    EXTI->IMR  |= EXTI_IMR_MR4;
    NVIC_SetPriority(EXTI4_IRQn, 1);
    NVIC_EnableIRQ(EXTI4_IRQn);
}

void EXTI0_IRQHandler(void) {
    if (EXTI->PR & EXTI_PR_PR0) {
        EXTI->PR = EXTI_PR_PR0;

        // debounce sederhana
        static uint32_t lastInterrupt = 0;
        if ((sysTick_ms - lastInterrupt) > 50) {
            buttonPressed = 1;
            lastInterrupt = sysTick_ms;
        }
    }
}

void EXTI4_IRQHandler(void) {
    if (EXTI->PR & EXTI_PR_PR4) {
        EXTI->PR = EXTI_PR_PR4;

        // Baca DT untuk arah (mirip contoh ESP32 dan kode clode)
        // NOTE: kalau arah kebalik, tinggal swap increment/decrement di sini.
        if (GPIOB->IDR & (1U << ENCODER_DT_PIN)) {
            encoderCount--;
            encoderDir = -1;
        } else {
            encoderCount++;
            encoderDir = 1;
        }
    }
}

// ============================================================
// TIM3 PWM untuk Motor Enable (PA6 TIM3_CH1)
// ============================================================
void TIM3_PWM_Init(void) {
    RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;

    // 50 MHz / 50 = 1 MHz, ARR 1000 => 1 kHz PWM
    TIM3->PSC = 49;
    TIM3->ARR = 999;

    // CH1 PWM Mode 1
    TIM3->CCMR1 &= ~TIM_CCMR1_OC1M;
    TIM3->CCMR1 |= (6U << TIM_CCMR1_OC1M_Pos);
    TIM3->CCMR1 |= TIM_CCMR1_OC1PE;

    // Enable CH1 output
    TIM3->CCER |= TIM_CCER_CC1E;

    // Start timer
    TIM3->CR1 |= TIM_CR1_CEN;

    // Start with 0 duty
    TIM3->CCR1 = 0;
}

// ============================================================
// SYSTICK 1ms
// ============================================================
void SysTick_Init(void) {
    SysTick->LOAD = (SystemCoreClock / 1000) - 1;
    SysTick->VAL = 0;
    SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk |
                    SysTick_CTRL_TICKINT_Msk |
                    SysTick_CTRL_ENABLE_Msk;
}

void SysTick_Handler(void) {
    sysTick_ms++;
}

uint32_t millis(void) {
    return sysTick_ms;
}

void delay_ms(uint32_t ms) {
    uint32_t start = sysTick_ms;
    while ((sysTick_ms - start) < ms);
}

// ============================================================
// RANDOM (LFSR)
// ============================================================
uint32_t random_lfsr(void) {
    uint32_t bit = ((lfsr >> 0) ^ (lfsr >> 2) ^ (lfsr >> 3) ^ (lfsr >> 5)) & 1u;
    lfsr = (lfsr >> 1) | (bit << 15);
    return lfsr;
}

uint32_t random_range(uint32_t min, uint32_t max) {
    // [min, max)
    return min + (random_lfsr() % (max - min));
}

void random_seed(uint32_t seed) {
    if (seed != 0) lfsr = seed;
}

// ============================================================
// MOTOR CONTROL (L298N) + PID
// speed: -255..255 (negatif = reverse)
// ============================================================
void motorStop(void) {
    GPIOA->ODR &= ~(1U << MOTOR_IN1_PIN);
    GPIOB->ODR &= ~(1U << MOTOR_IN2_PIN);
    TIM3->CCR1 = 0;
}

static inline uint32_t pwm_from_8bit(uint16_t v) {
    // v: 0..255 -> duty 0..999
    return (v * 999U) / 255U;
}

void motorControl(int16_t speed) {
    if (speed > 255) speed = 255;
    if (speed < -255) speed = -255;

    if (speed > 0) {
        // Forward
        GPIOA->ODR |= (1U << MOTOR_IN1_PIN);
        GPIOB->ODR &= ~(1U << MOTOR_IN2_PIN);

        uint16_t u = (uint16_t)speed;
        if (MIN_EFFECTIVE_PWM > 0 && u > 0 && u < MIN_EFFECTIVE_PWM) u = MIN_EFFECTIVE_PWM;
        TIM3->CCR1 = pwm_from_8bit(u);
    } else if (speed < 0) {
        // Reverse
        GPIOA->ODR &= ~(1U << MOTOR_IN1_PIN);
        GPIOB->ODR |= (1U << MOTOR_IN2_PIN);

        uint16_t u = (uint16_t)abs(speed);
        if (MIN_EFFECTIVE_PWM > 0 && u > 0 && u < MIN_EFFECTIVE_PWM) u = MIN_EFFECTIVE_PWM;
        TIM3->CCR1 = pwm_from_8bit(u);
    } else {
        motorStop();
    }
}

float PID_Compute(PID_Controller* p, float current) {
    uint32_t now = millis();
    float dt = (now - p->lastTime) / 1000.0f;
    if (dt <= 0.0f) dt = 0.001f;

    float error = p->setpoint - current;

    // P
    float P = p->Kp * error;

    // I (anti-windup)
    p->integral += error * dt;
    if (p->integral > 100.0f) p->integral = 100.0f;
    if (p->integral < -100.0f) p->integral = -100.0f;
    float I = p->Ki * p->integral;

    // D
    float D = p->Kd * (error - p->prev_error) / dt;

    float out = P + I + D;

    if (out > PID_MAX_OUTPUT) out = PID_MAX_OUTPUT;
    if (out < PID_MIN_OUTPUT) out = PID_MIN_OUTPUT;

    p->prev_error = error;
    p->lastTime = now;

    return out;
}

// ============================================================
// ENCODER HELPERS
// ============================================================
void resetEncoder(void) {
    encoderCount = 0;
    pid.integral = 0.0f;
    pid.prev_error = 0.0f;
}

float getEncoderAngleDeg(void) {
    return ((float)encoderCount * 360.0f) / (float)encoderPPR;
}

// Gerak sampai targetDeg (relative dari 0 karena kita resetEncoder sebelum gerak)
void moveToAngleDeg(float targetDeg) {
    // convert target ke setpoint pulse
    float targetPulses = (targetDeg * (float)encoderPPR) / 360.0f;

    pid.setpoint = targetPulses;
    pid.integral = 0.0f;
    pid.prev_error = 0.0f;
    pid.lastTime = millis();

    uint32_t timeoutAt = millis() + MOVE_TIMEOUT_MS;
    uint32_t stableCount = 0;

    while (millis() < timeoutAt) {
        float current = (float)encoderCount;
        float error = pid.setpoint - current;

        // Selesai kalau error kecil & stabil beberapa ms
        if (abs((int32_t)error) <= ENCODER_TOL_PULSES) {
            stableCount++;
            if (stableCount >= 50) { // ~50ms stabil
                motorStop();
                return;
            }
        } else {
            stableCount = 0;
        }

        float out = PID_Compute(&pid, current);
        motorControl((int16_t)out);

        delay_ms(1);
    }

    // timeout safety
    motorStop();
}

// ============================================================
// BUZZER (Software tone - blocking)
// ============================================================
void buzzerTone(uint16_t freq, uint32_t duration) {
    uint32_t halfPeriod = 500000 / freq;  // microseconds
    uint32_t cycles = (freq * duration) / 1000;

    for (uint32_t i = 0; i < cycles; i++) {
        GPIOA->ODR |= (1U << BUZZER_PIN);
        for (volatile uint32_t j = 0; j < halfPeriod * 10; j++);
        GPIOA->ODR &= ~(1U << BUZZER_PIN);
        for (volatile uint32_t j = 0; j < halfPeriod * 10; j++);
    }
}

void startupSound(void) {
    buzzerTone(523, 80);
    delay_ms(20);
    buzzerTone(659, 80);
    delay_ms(20);
    buzzerTone(784, 120);
    delay_ms(50);
}

void winSound(void) {
    buzzerTone(523, 80);
    delay_ms(20);
    buzzerTone(659, 80);
    delay_ms(20);
    buzzerTone(784, 80);
    delay_ms(20);
    buzzerTone(1047, 200);
    delay_ms(50);
}

void loseSound(void) {
    buzzerTone(300, 100);
    delay_ms(20);
    buzzerTone(200, 200);
    delay_ms(50);
}

// ============================================================
// LED CONTROL
// ============================================================
void ledOn(uint8_t pin) {
    GPIOA->ODR |= (1U << pin);
}

void ledOff(uint8_t pin) {
    GPIOA->ODR &= ~(1U << pin);
}

void blinkLED(uint8_t pin, uint8_t times, uint32_t delayTime) {
    for (uint8_t i = 0; i < times; i++) {
        ledOn(pin);
        delay_ms(delayTime);
        ledOff(pin);
        delay_ms(delayTime);
    }
}

// ============================================================
// PROSES GACHA
// - kalau MENANG: motor muter sampai target derajat (90 + fail*30)
// - kalau KALAH: failCount++ (target kartu berikutnya makin jauh)
// ============================================================
void prosesGacha(void) {
    ledOff(LED_HIJAU_PIN);
    ledOff(LED_BIRU_PIN);

    int currentChance = BASE_CHANCE + (failCount * ADD_CHANCE);
    if (currentChance > 100) currentChance = 100;

    float targetDeg = BASE_ANGLE_DEG + ((float)failCount * ADD_ANGLE_DEG);

    int rng = (int)random_range(0, 100);

    // beep konfirmasi
    buzzerTone(1000, 30);
    delay_ms(20);

    if (rng < currentChance) {
        // --- MENANG ---
        ledOn(LED_HIJAU_PIN);

        // sound dulu
        winSound();

        // reset posisi relatif, lalu gerak ke targetDeg pakai PID+encoder
        resetEncoder();
        moveToAngleDeg(targetDeg);

        blinkLED(LED_HIJAU_PIN, 3, 80);

        failCount = 0;

    } else {
        // --- GAGAL ---
        loseSound();
        blinkLED(LED_BIRU_PIN, 2, 100);
        failCount++;
    }
}

// ============================================================
// MAIN
// ============================================================
int main(void) {
    SystemClock_Config();
    GPIO_Init();
    EXTI_Init();
    TIM3_PWM_Init();
    SysTick_Init();

    // Seed RNG
    delay_ms(10);
    random_seed(sysTick_ms ^ 0xDEADBEEF);

    // Startup
    startupSound();

    while (1) {
        if (buttonPressed) {
            buttonPressed = 0;

            if ((millis() - lastPressTime) > COOLDOWN_TIME) {
                lastPressTime = millis();
                prosesGacha();
            }
        }
    }
}