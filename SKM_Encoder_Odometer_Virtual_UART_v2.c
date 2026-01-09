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

// --- Rotary Encoder
// HW encoder kamu "buta arah": mau diputar CW/CCW, count tetap naik.
// Jadi encoder dianggap ODOMETER (jarak tempuh), bukan posisi.
#define ENCODER_CLK_PIN 4   // PB4 (EXTI4)  -> CLK/A
#define ENCODER_DT_PIN  5   // PB5          -> DT/B (tidak dipakai)

// --- UART (USB-TTL) USART1 on PA9/PA10
#define UART_TX_PIN     9   // PA9  (USART1_TX)
#define UART_RX_PIN     10  // PA10 (USART1_RX) - optional kalau cuma print

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
// ENCODER SETTING
// ============================================================
// PPR = pulses per revolution (wajib disesuaikan biar derajat akurat)
#define ENCODER_PPR_DEFAULT  600U

// toleransi selesai (dalam pulse)
#define ENCODER_TOL_PULSES   2

// timeout safety untuk gerak motor (ms)
#define MOVE_TIMEOUT_MS      6000

// settle time setelah motor stop (ms)
// Untuk test manual: kecilin biar nggak jauh dari kenyataan.
// Untuk motor beneran (ada inertia): bisa naikin 50-100ms.
#define SETTLE_MS            20

// PWM minimum supaya motor kuat start (kalau perlu)
#define MIN_EFFECTIVE_PWM    35

// Print interval status move (ms)
#define PRINT_INTERVAL_MS    25

// max koreksi bolak-balik (anti infinite loop)
#define MAX_BOUNCE           12

// ============================================================
// VARIABEL SYSTEM
// ============================================================
volatile uint32_t sysTick_ms = 0;
volatile uint8_t buttonPressed = 0;
uint32_t lastPressTime = 0;
int failCount = 0;

// --- Encoder raw (odometer) : selalu naik
volatile int32_t encoderCount = 0;

// --- Virtual position (estimasi posisi) : bisa naik/turun
// Dibangun dari delta raw + arah gerak motor yang kita command.
volatile int32_t virtCount = 0;
static int32_t lastRawCount = 0;
static volatile int8_t lastMotionDir = +1; // +1 FWD, -1 REV (arah terakhir command)

// --- RNG State (LFSR)
static uint32_t lfsr = 0xACE1u;

// --- Encoder parameter
static uint16_t encoderPPR = ENCODER_PPR_DEFAULT;

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

// UART
void UART1_Init(uint32_t baud);
void uart_putc(char c);
void uart_puts(const char* s);
void uart_newline(void);
void uart_print_int(int32_t v);
void uart_print_deg_x10(int32_t deg_x10);

// RNG
uint32_t random_lfsr(void);
uint32_t random_range(uint32_t min, uint32_t max);
void random_seed(uint32_t seed);

// Motor
void motorStop(void);
void motorControl(int16_t speed);   // -255..255 (negatif = reverse)

// Encoder helpers
void resetEncoder(void);
static inline void updateVirtualFromRaw(void);
static inline void snapshotCounts(int32_t* rawOut, int32_t* virtOut);
int32_t pulses_to_deg_x10(int32_t pulses);
int32_t deg_to_pulses(float deg);
static inline int32_t rawAngle_x10_from(int32_t rawPulses);
static inline int32_t virtAngle_x10_from(int32_t virtPulses);

// Move
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

    // --- PA6: Motor Enable (AF - TIM3_CH1 PWM) ---
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
// UART1 (PA9 TX, PA10 RX)
// ============================================================
void UART1_Init(uint32_t baud) {
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
    RCC->APB2ENR |= RCC_APB2ENR_USART1EN;

    // PA9 AF7
    GPIOA->MODER &= ~(3U << (UART_TX_PIN * 2));
    GPIOA->MODER |=  (2U << (UART_TX_PIN * 2));
    GPIOA->AFR[1] &= ~(0xFU << ((UART_TX_PIN - 8) * 4));
    GPIOA->AFR[1] |=  (7U   << ((UART_TX_PIN - 8) * 4));

    // PA10 AF7 (optional)
    GPIOA->MODER &= ~(3U << (UART_RX_PIN * 2));
    GPIOA->MODER |=  (2U << (UART_RX_PIN * 2));
    GPIOA->AFR[1] &= ~(0xFU << ((UART_RX_PIN - 8) * 4));
    GPIOA->AFR[1] |=  (7U   << ((UART_RX_PIN - 8) * 4));

    uint32_t apb2 = SystemCoreClock;
    USART1->BRR = (apb2 + (baud / 2U)) / baud;

    USART1->CR1 = 0;
    USART1->CR1 |= USART_CR1_TE;
    USART1->CR1 |= USART_CR1_RE;
    USART1->CR1 |= USART_CR1_UE;
}

void uart_putc(char c) {
    while (!(USART1->SR & USART_SR_TXE));
    USART1->DR = (uint8_t)c;
}

void uart_puts(const char* s) {
    while (*s) uart_putc(*s++);
}

void uart_newline(void) {
    uart_putc('\r');
    uart_putc('\n');
}

void uart_print_int(int32_t v) {
    char buf[12];
    int i = 0;
    if (v == 0) { uart_putc('0'); return; }
    if (v < 0) { uart_putc('-'); v = -v; }
    while (v > 0 && i < (int)sizeof(buf)) {
        buf[i++] = '0' + (v % 10);
        v /= 10;
    }
    while (i--) uart_putc(buf[i]);
}

void uart_print_deg_x10(int32_t deg_x10) {
    if (deg_x10 < 0) { uart_putc('-'); deg_x10 = -deg_x10; }
    uart_print_int(deg_x10 / 10);
    uart_putc('.');
    uart_putc('0' + (deg_x10 % 10));
}

// ============================================================
// EXTI INIT
// ============================================================
void EXTI_Init(void) {
    RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;

    // EXTI0 -> PA0
    SYSCFG->EXTICR[0] &= ~SYSCFG_EXTICR1_EXTI0;
    EXTI->FTSR |= EXTI_FTSR_TR0;
    EXTI->RTSR &= ~EXTI_RTSR_TR0;
    EXTI->IMR  |= EXTI_IMR_MR0;
    NVIC_SetPriority(EXTI0_IRQn, 2);
    NVIC_EnableIRQ(EXTI0_IRQn);

    // EXTI4 -> PB4
    SYSCFG->EXTICR[1] &= ~SYSCFG_EXTICR2_EXTI4;
    SYSCFG->EXTICR[1] |= SYSCFG_EXTICR2_EXTI4_PB;
    EXTI->RTSR |= EXTI_RTSR_TR4;
    EXTI->FTSR &= ~EXTI_FTSR_TR4;
    EXTI->IMR  |= EXTI_IMR_MR4;
    NVIC_SetPriority(EXTI4_IRQn, 1);
    NVIC_EnableIRQ(EXTI4_IRQn);
}

void EXTI0_IRQHandler(void) {
    if (EXTI->PR & EXTI_PR_PR0) {
        EXTI->PR = EXTI_PR_PR0;

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
        encoderCount++; // always +
    }
}

// ============================================================
// TIM3 PWM for Motor
// ============================================================
void TIM3_PWM_Init(void) {
    RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;

    TIM3->PSC = 49;
    TIM3->ARR = 999;

    TIM3->CCMR1 &= ~TIM_CCMR1_OC1M;
    TIM3->CCMR1 |= (6U << TIM_CCMR1_OC1M_Pos);
    TIM3->CCMR1 |= TIM_CCMR1_OC1PE;

    TIM3->CCER |= TIM_CCER_CC1E;
    TIM3->CR1 |= TIM_CR1_CEN;

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

void SysTick_Handler(void) { sysTick_ms++; }

uint32_t millis(void) { return sysTick_ms; }

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
    return min + (random_lfsr() % (max - min));
}

void random_seed(uint32_t seed) {
    if (seed != 0) lfsr = seed;
}

// ============================================================
// MOTOR CONTROL (L298N)
// ============================================================
void motorStop(void) {
    GPIOA->ODR &= ~(1U << MOTOR_IN1_PIN);
    GPIOB->ODR &= ~(1U << MOTOR_IN2_PIN);
    TIM3->CCR1 = 0;
}

static inline uint32_t pwm_from_8bit(uint16_t v) {
    return (v * 999U) / 255U;
}

void motorControl(int16_t speed) {
    if (speed > 255) speed = 255;
    if (speed < -255) speed = -255;

    if (speed > 0) {
        lastMotionDir = +1;

        GPIOA->ODR |= (1U << MOTOR_IN1_PIN);
        GPIOB->ODR &= ~(1U << MOTOR_IN2_PIN);

        uint16_t u = (uint16_t)speed;
        if (MIN_EFFECTIVE_PWM > 0 && u > 0 && u < MIN_EFFECTIVE_PWM) u = MIN_EFFECTIVE_PWM;
        TIM3->CCR1 = pwm_from_8bit(u);

    } else if (speed < 0) {
        lastMotionDir = -1;

        GPIOA->ODR &= ~(1U << MOTOR_IN1_PIN);
        GPIOB->ODR |= (1U << MOTOR_IN2_PIN);

        uint16_t u = (uint16_t)abs(speed);
        if (MIN_EFFECTIVE_PWM > 0 && u > 0 && u < MIN_EFFECTIVE_PWM) u = MIN_EFFECTIVE_PWM;
        TIM3->CCR1 = pwm_from_8bit(u);

    } else {
        motorStop();
    }
}

// ============================================================
// ENCODER + VIRTUAL POSITION
// ============================================================
void resetEncoder(void) {
    encoderCount = 0;
    virtCount = 0;
    lastRawCount = 0;
    lastMotionDir = +1;
}

static inline void updateVirtualFromRaw(void) {
    int32_t raw = encoderCount;
    int32_t delta = raw - lastRawCount;
    if (delta != 0) {
        virtCount += (lastMotionDir > 0) ? delta : -delta;
        lastRawCount = raw;
    }
}

static inline void snapshotCounts(int32_t* rawOut, int32_t* virtOut) {
    __disable_irq();
    *rawOut = encoderCount;
    *virtOut = virtCount;
    __enable_irq();
}

static inline int32_t rawAngle_x10_from(int32_t rawPulses) {
    return (int32_t)(((int64_t)rawPulses * 3600LL) / (int64_t)encoderPPR);
}

static inline int32_t virtAngle_x10_from(int32_t virtPulses) {
    return (int32_t)(((int64_t)virtPulses * 3600LL) / (int64_t)encoderPPR);
}

int32_t pulses_to_deg_x10(int32_t pulses) {
    return (int32_t)(((int64_t)pulses * 3600LL) / (int64_t)encoderPPR);
}

int32_t deg_to_pulses(float deg) {
    float p = (deg * (float)encoderPPR) / 360.0f;
    if (p < 0) p = 0;
    return (int32_t)(p + 0.5f);
}

// ============================================================
// MOVE: target berdasarkan VIRTUAL POSITION
// RAW tetap naik terus, VIRT turun kalau reverse.
// Tambahan: setelah STOP+SETTLE, dicek lagi. Kalau drift, lanjut koreksi.
// ============================================================
void moveToAngleDeg(float targetDeg) {
    int32_t targetPulse = deg_to_pulses(targetDeg);

    // start relatif
    virtCount = 0;
    lastRawCount = encoderCount;

    int16_t pwm = 200;
    const int16_t pwmMin = (MIN_EFFECTIVE_PWM > 0) ? MIN_EFFECTIVE_PWM : 20;

    uint8_t bounce = 0;
    int32_t prevErr = 0;

    uint32_t timeoutAt = millis() + MOVE_TIMEOUT_MS;
    uint32_t lastPrint = 0;

    uart_puts("[MOVE] T=");
    uart_print_deg_x10((int32_t)(targetDeg * 10.0f + 0.5f));
    uart_puts("deg TP=");
    uart_print_int(targetPulse);
    uart_newline();

    updateVirtualFromRaw();
    prevErr = targetPulse - virtCount;

    while (millis() < timeoutAt) {
        updateVirtualFromRaw();

        int32_t err = targetPulse - virtCount;   // + kurang, - kebanyakan
        int32_t aerr = (err < 0) ? -err : err;

        // DONE check (awal)
        if (aerr <= ENCODER_TOL_PULSES) {
            motorStop();
            delay_ms(SETTLE_MS);
            updateVirtualFromRaw();

            // re-check setelah settle (anti "false done" kalau masih ada gerak)
            int32_t err2 = targetPulse - virtCount;
            int32_t aerr2 = (err2 < 0) ? -err2 : err2;

            if (aerr2 <= ENCODER_TOL_PULSES) {
                int32_t rawSnap, virtSnap;
                snapshotCounts(&rawSnap, &virtSnap);

                uart_puts("[DONE] V=");
                uart_print_deg_x10(virtAngle_x10_from(virtSnap));
                uart_puts(" RAW=");
                uart_print_deg_x10(rawAngle_x10_from(rawSnap));
                uart_newline();
                return;
            } else {
                // masih drift -> lanjut koreksi dengan PWM lebih kecil
                pwm = (int16_t)((pwm * 7) / 10);
                if (pwm < pwmMin) pwm = pwmMin;
                bounce++;
                if (bounce >= MAX_BOUNCE) {
                    uart_puts("[FAIL] drift after done, max bounce");
                    uart_newline();
                    motorStop();
                    return;
                }
                prevErr = err2;
                continue;
            }
        }

        // arah dari tanda error
        int dir = (err > 0) ? +1 : -1;

        // taper dekat target
        int16_t sp = pwm;
        int32_t th10 = deg_to_pulses(10.0f);
        int32_t th3  = deg_to_pulses(3.0f);
        if (aerr <= th10 && sp > 140) sp = 140;
        if (aerr <= th3  && sp > 90)  sp = 90;

        motorControl((dir > 0) ? sp : -sp);

        // status ringkas (pakai snapshot biar RAW/V konsisten)
        if ((millis() - lastPrint) >= PRINT_INTERVAL_MS) {
            lastPrint = millis();

            int32_t rawSnap, virtSnap;
            snapshotCounts(&rawSnap, &virtSnap);

            uart_puts("  D=");
            uart_puts((dir > 0) ? "F" : "R");
            uart_puts(" P=");
            uart_print_int(sp);
            uart_puts(" V=");
            uart_print_deg_x10(virtAngle_x10_from(virtSnap));
            uart_puts(" RAW=");
            uart_print_deg_x10(rawAngle_x10_from(rawSnap));
            uart_puts(" E=");
            uart_print_deg_x10(pulses_to_deg_x10(err));
            uart_newline();
        }

        // nyebrang target (error sign berubah) -> stop+settle+turunin pwm
        if (((prevErr > 0) && (err < 0)) || ((prevErr < 0) && (err > 0))) {
            motorStop();
            delay_ms(SETTLE_MS);
            updateVirtualFromRaw();

            int32_t err2 = targetPulse - virtCount;

            uart_puts("  [CROSS] b=");
            uart_print_int(bounce);
            uart_puts(" V=");
            uart_print_deg_x10(virtAngle_x10_from(virtCount));
            uart_puts(" E=");
            uart_print_deg_x10(pulses_to_deg_x10(err2));
            uart_newline();

            pwm = (int16_t)((pwm * 7) / 10);
            if (pwm < pwmMin) pwm = pwmMin;

            bounce++;
            if (bounce >= MAX_BOUNCE) {
                uart_puts("[FAIL] max bounce reached");
                uart_newline();
                motorStop();
                return;
            }

            prevErr = err2;
            continue;
        }

        prevErr = err;
        delay_ms(1);
    }

    uart_puts("[TIMEOUT] stop motor");
    uart_newline();
    motorStop();
}

// ============================================================
// BUZZER (blocking)
// ============================================================
void buzzerTone(uint16_t freq, uint32_t duration) {
    uint32_t halfPeriod = 500000 / freq;
    uint32_t cycles = (freq * duration) / 1000;

    for (uint32_t i = 0; i < cycles; i++) {
        GPIOA->ODR |= (1U << BUZZER_PIN);
        for (volatile uint32_t j = 0; j < halfPeriod * 10; j++);
        GPIOA->ODR &= ~(1U << BUZZER_PIN);
        for (volatile uint32_t j = 0; j < halfPeriod * 10; j++);
    }
}

void startupSound(void) {
    buzzerTone(523, 80); delay_ms(20);
    buzzerTone(659, 80); delay_ms(20);
    buzzerTone(784, 120); delay_ms(50);
}

void winSound(void) {
    buzzerTone(523, 80); delay_ms(20);
    buzzerTone(659, 80); delay_ms(20);
    buzzerTone(784, 80); delay_ms(20);
    buzzerTone(1047, 200); delay_ms(50);
}

void loseSound(void) {
    buzzerTone(300, 100); delay_ms(20);
    buzzerTone(200, 200); delay_ms(50);
}

// ============================================================
// LED CONTROL
// ============================================================
void ledOn(uint8_t pin) { GPIOA->ODR |= (1U << pin); }
void ledOff(uint8_t pin) { GPIOA->ODR &= ~(1U << pin); }

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
// ============================================================
void prosesGacha(void) {
    ledOff(LED_HIJAU_PIN);
    ledOff(LED_BIRU_PIN);

    int currentChance = BASE_CHANCE + (failCount * ADD_CHANCE);
    if (currentChance > 100) currentChance = 100;

    float targetDeg = BASE_ANGLE_DEG + ((float)failCount * ADD_ANGLE_DEG);
    int rng = (int)random_range(0, 100);

    uart_puts("=== GACHA === fail=");
    uart_print_int(failCount);
    uart_puts(" chance=");
    uart_print_int(currentChance);
    uart_puts("% target=");
    uart_print_deg_x10((int32_t)(targetDeg * 10.0f + 0.5f));
    uart_puts("deg rng=");
    uart_print_int(rng);
    uart_newline();

    buzzerTone(1000, 30);
    delay_ms(20);

    if (rng < currentChance) {
        uart_puts("[RESULT] WIN");
        uart_newline();

        ledOn(LED_HIJAU_PIN);
        winSound();

        resetEncoder();

        uart_puts("[ENC] start RAW=");
        uart_print_deg_x10(rawAngle_x10_from(encoderCount));
        uart_puts(" V=");
        uart_print_deg_x10(virtAngle_x10_from(virtCount));
        uart_newline();

        moveToAngleDeg(targetDeg);

        uart_puts("[ENC] end   RAW=");
        uart_print_deg_x10(rawAngle_x10_from(encoderCount));
        uart_puts(" V=");
        uart_print_deg_x10(virtAngle_x10_from(virtCount));
        uart_newline();

        blinkLED(LED_HIJAU_PIN, 3, 80);
        failCount = 0;

    } else {
        uart_puts("[RESULT] LOSE -> failCount=");
        uart_print_int(failCount + 1);
        uart_newline();

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

    UART1_Init(115200);
    uart_puts("BOOT OK (USART1 115200)  RAW=odometer  V=virtualPos");
    uart_newline();

    delay_ms(10);
    random_seed(sysTick_ms ^ 0xDEADBEEF);

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
