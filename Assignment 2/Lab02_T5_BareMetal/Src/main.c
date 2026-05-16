/*
 * ============================================================
 * CSE 2206 — Lab-02 Task 5: WS2812B Bare-Metal + DMA (FINAL)
 * ============================================================
 *
 * Target MCU      : STM32F446RE Nucleo-64
 * Protocol        : WS2812B (NeoPixel / Addressable RGB LED)
 * Interface       : TIM1_CH1 PWM with DMA2 Stream1 Transfers
 *
 * GPIO Mapping:
 *   • PA8 → TIM1_CH1 (Alternate Function 1)
 *
 * Timer Configuration:
 *   • TIM1 Clock     : 180 MHz (APB2)
 *   • PWM Frequency  : 720 kHz (ARR = 225)
 *   • Output Pin     : PA8 (AF1)
 *
 * DMA Mapping:
 *   • DMA2 Stream1 Channel6 → TIM1_CH1 (PWM Compare Register)
 *
 * Features Implemented:
 *   ✓ 10-colour static palette demonstration
 *   ✓ HSV-based rainbow hue sweep animation
 *   ✓ 4-LED colour chase animation with 3 rounds
 *   ✓ UART console logging with formatted output
 *   ✓ Interrupt-driven DMA completion handling
 *
 * ============================================================
 */

#include <stm32f446xx.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "helper.h"

/* ============================================================
 * External Function Declarations
 * ============================================================ */
extern void SystemClock_Config(void);
extern void USART2_Init(void);
extern void USART2_SendString(const char *s);
extern void TIM6_Init(void);
extern void delay_us(uint16_t us);
extern void delay_ms(uint32_t ms);


/* ============================================================
 * WS2812B Protocol Timing Constants
 * ============================================================
 *
 * System Clock      : 180 MHz → Timer tick = 5.55 ns
 * Protocol Bitrate  : 800 kHz → Bit period = 1.25 μs
 *
 * Timing Requirements (@ 180 MHz):
 *   • WS_ARR  : Period counter (225 ticks = 1.25 μs)
 *   • WS_T1H  : Logic-1 high time (150 ticks ≈ 0.833 μs)
 *   • WS_T0H  : Logic-0 high time (75 ticks ≈ 0.417 μs)
 *   • WS_RESET: Reset sequence duration (≥280 μs, uses 50 zero-ticks)
 *
 * ============================================================ */
#define WS_ARR      225U    /* Period: 1.25 μs */
#define WS_T1H      150U    /* Bit-1 pulse width */
#define WS_T0H       75U    /* Bit-0 pulse width */
#define WS_RESET     50U    /* Reset gap (zero values) */
#define NUM_LEDS      5U    /* Total addressable LEDs */

#define PWM_BUF_SIZE   ((NUM_LEDS * 24U) + WS_RESET)


/* ============================================================
 * Data Structures & Global Variables
 * ============================================================ */

/* PWM pulse width data buffer (DMA source) */
static uint16_t pwmData[PWM_BUF_SIZE];

/* RGB LED colour representation */
typedef struct {
    uint8_t r, g, b;
} LED_t;

/* LED state buffer (all 5 LEDs) */
static LED_t g_leds[NUM_LEDS];

/* DMA transfer completion flag (set by IRQ handler) */
static volatile uint8_t datasentflag = 0;


/* ============================================================
 * GPIO Initialization — PA8 → TIM1_CH1 (Alternate Function 1)
 * ============================================================
 *
 * Configuration:
 *   • Mode       : AF (Alternate Function)
 *   • AF Select  : AF1 (TIM1_CH1)
 *   • Output     : Push-Pull
 *   • Speed      : High (50 MHz)
 *   • Pull       : None
 *
 * ============================================================ */
static void GPIO_WS_Init(void)
{
    /* Enable GPIO Port A clock */
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
    __NOP(); __NOP();

    /* PA8 configuration */
    GPIOA->MODER   = (GPIOA->MODER  & ~(3U << 16)) | (2U << 16);  /* Mode: AF */
    GPIOA->OTYPER  &= ~(1U << 8);                                  /* Type: Push-Pull */
    GPIOA->OSPEEDR |=  (3U << 16);                                 /* Speed: High */
    GPIOA->PUPDR   &= ~(3U << 16);                                 /* Pull: None */

    /* Select TIM1_CH1 alternate function (AF1) */
    GPIOA->AFR[1] = (GPIOA->AFR[1] & ~0xFU) | 0x1U;
}


/* ============================================================
 * TIM1 PWM Mode Configuration
 * ============================================================
 *
 * Registers Configured:
 *   • CR1     : Timer control (enabled later)
 *   • PSC     : Prescaler (0 for direct 180 MHz clock)
 *   • ARR     : Auto-reload register (225 = 1.25 μs period)
 *   • CCR1    : Compare register (variable PWM pulse width)
 *   • CCMR1   : Output Compare Mode 1 (PWM Mode 1)
 *   • CCER    : Capture/Compare Enable Register (CH1 output enabled)
 *   • BDTR    : Break & Dead-Time (MOE = Master Output Enable)
 *
 * ============================================================ */
static void TIM1_WS_Init(void)
{
    /* Enable TIM1 clock on APB2 */
    RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;
    __NOP(); __NOP();

    /* Timer control: disabled initially */
    TIM1->CR1  = 0U;

    /* No prescaler: use full 180 MHz clock */
    TIM1->PSC  = 0U;

    /* Auto-reload value: creates 1.25 μs bit period */
    TIM1->ARR  = WS_ARR;

    /* Compare register cleared */
    TIM1->CCR1 = 0U;

    /* Repetition counter: single updates */
    TIM1->RCR  = 0U;

    /* Output Compare Mode 1: PWM Mode 1 + Preload Enable */
    TIM1->CCMR1 = (6U << TIM_CCMR1_OC1M_Pos) | TIM_CCMR1_OC1PE;

    /* Enable Channel 1 output */
    TIM1->CCER  = TIM_CCER_CC1E;

    /* Enable Master Output (required for TIM1) */
    TIM1->BDTR  = TIM_BDTR_MOE;

    /* Generate update event & clear status */
    TIM1->EGR = TIM_EGR_UG;
    TIM1->SR  = 0U;
}


/* ============================================================
 * DMA2 Stream1 Configuration for TIM1_CH1
 * ============================================================
 *
 * DMA Mapping:
 *   • Stream    : Stream1
 *   • Channel   : Channel6
 *   • Peripheral: TIM1_CH1 (CCR1 register)
 *
 * Transfer Properties:
 *   • Source    : pwmData[] buffer (SRAM)
 *   • Dest      : TIM1->CCR1 (peripheral register)
 *   • Direction : Memory-to-Peripheral (M2P)
 *   • Data Size : 16-bit halfword transfers
 *   • Increment : Source increments, Dest fixed
 *   • Mode      : Single-burst transfers on TIM1 updates
 *
 * Interrupt Configuration:
 *   • TCIF1     : Transfer Complete Interrupt Flag
 *   • Priority  : 0 (highest)
 *
 * ============================================================ */
static void DMA2_WS_Init(void)
{
    /* Enable DMA2 clock on AHB1 */
    RCC->AHB1ENR |= RCC_AHB1ENR_DMA2EN;
    __NOP(); __NOP();

    /* Disable DMA Stream1 and wait for disable completion */
    DMA2_Stream1->CR &= ~DMA_SxCR_EN;
    while (DMA2_Stream1->CR & DMA_SxCR_EN) {}

    /* Clear all interrupt flags for Stream1 */
    DMA2->LIFCR = DMA_LIFCR_CTCIF1  |
                  DMA_LIFCR_CHTIF1  |
                  DMA_LIFCR_CTEIF1  |
                  DMA_LIFCR_CDMEIF1 |
                  DMA_LIFCR_CFEIF1;

    /* Configure DMA Stream1 Control Register */
    DMA2_Stream1->CR =
          (6U << 25U)   /* Channel 6 selection */
        | (2U << 16U)   /* Data size: 16-bit (halfword) */
        | (1U << 13U)   /* Memory increment enable */
        | (1U << 11U)   /* Medium priority */
        | (1U << 10U)   /* Peripheral data size: 16-bit */
        | (1U <<  6U)   /* Direction: Memory-to-Peripheral */
        | (1U <<  4U);  /* Transfer Complete Interrupt Enable */

    /* Set peripheral address (TIM1->CCR1) */
    DMA2_Stream1->PAR = (uint32_t)&TIM1->CCR1;

    /* FIFO disabled (direct mode enabled) */
    DMA2_Stream1->FCR &= ~DMA_SxFCR_DMDIS;

    /* Configure NVIC interrupt for DMA2 Stream1 */
    NVIC_SetPriority(DMA2_Stream1_IRQn, 0U);
    NVIC_EnableIRQ(DMA2_Stream1_IRQn);
}


/* ============================================================
 * DMA2 Stream1 Interrupt Handler
 * ============================================================
 *
 * Purpose:
 *   Handles DMA transfer completion after all PWM data
 *   has been shifted to the WS2812B LEDs.
 *
 * Actions:
 *   1. Clear the Transfer Complete Interrupt Flag
 *   2. Stop the timer (PWM output ceases)
 *   3. Disable the DMA Stream
 *   4. Disable the DMA interrupt request from timer
 *   5. Set completion flag to unblock main thread
 *
 * ============================================================ */
void DMA2_Stream1_IRQHandler(void)
{
    if (DMA2->LISR & DMA_LISR_TCIF1)
    {
        /* Clear Transfer Complete Interrupt Flag */
        DMA2->LIFCR = DMA_LIFCR_CTCIF1;

        /* Stop timer (no more PWM output) */
        TIM1->CR1  &= ~TIM_CR1_CEN;

        /* Disable DMA stream */
        DMA2_Stream1->CR &= ~DMA_SxCR_EN;

        /* Disable DMA request from timer compare register */
        TIM1->DIER &= ~TIM_DIER_CC1DE;

        /* Signal completion to main thread */
        datasentflag = 1;
    }
}


/* ============================================================
 * WS2812B Data Transmission
 * ============================================================
 *
 * Process:
 *   1. Encode LED RGB values into PWM pulse widths
 *      • Bit=1 → T1H (150 ticks)
 *      • Bit=0 → T0H (75 ticks)
 *   2. Add reset sequence (≥280 μs of 0V)
 *   3. Configure DMA with encoded data buffer
 *   4. Start timer → PWM begins → DMA feeds data
 *   5. Wait for DMA completion interrupt
 *
 * Encoding Detail (GRB order):
 *   • Byte 0: Green (MSB-first, bits 23-16)
 *   • Byte 1: Red   (bits 15-8)
 *   • Byte 2: Blue  (LSB-first, bits 7-0)
 *
 * ============================================================ */
static void WS2812B_Send(void)
{
    uint32_t idx = 0;

    /* Encode LED colours into PWM pulse data */
    for (uint32_t led = 0; led < NUM_LEDS; led++)
    {
        /* Combine RGB values in GRB order for WS2812B protocol */
        uint32_t color = ((uint32_t)g_leds[led].g << 16)
                       | ((uint32_t)g_leds[led].r <<  8)
                       |  (uint32_t)g_leds[led].b;

        /* Extract each bit and create PWM pulse width entry */
        for (int bit = 23; bit >= 0; bit--)
        {
            pwmData[idx++] =
                (color & (1U << bit)) ? WS_T1H : WS_T0H;
        }
    }

    /* Add reset sequence (≥280 μs of low signal) */
    for (uint32_t i = 0; i < WS_RESET; i++)
        pwmData[idx++] = 0U;

    /* Stop timer if already running */
    TIM1->CR1 &= ~TIM_CR1_CEN;

    /* Disable and wait for DMA stream to be idle */
    DMA2_Stream1->CR &= ~DMA_SxCR_EN;
    while (DMA2_Stream1->CR & DMA_SxCR_EN) {}

    /* Clear all interrupt flags */
    DMA2->LIFCR = DMA_LIFCR_CTCIF1  |
                  DMA_LIFCR_CHTIF1  |
                  DMA_LIFCR_CTEIF1  |
                  DMA_LIFCR_CDMEIF1 |
                  DMA_LIFCR_CFEIF1;

    /* Configure DMA source buffer and transfer count */
    DMA2_Stream1->M0AR = (uint32_t)pwmData;
    DMA2_Stream1->NDTR = idx;

    /* Reset timer and compare register */
    TIM1->CCR1 = 0U;
    TIM1->CNT  = 0U;
    TIM1->SR   = 0U;

    /* Enable DMA request from Compare Register (CC1DE) */
    TIM1->DIER |= TIM_DIER_CC1DE;

    /* Enable DMA stream */
    DMA2_Stream1->CR |= DMA_SxCR_EN;

    /* Start timer → PWM begins → DMA transfers begin */
    TIM1->CR1 |= TIM_CR1_CEN;

    /* Block until DMA transfer complete */
    while (!datasentflag) {}
    datasentflag = 0;
}


/* ============================================================
 * LED Helper Functions
 * ============================================================ */

/* --------- Set All LEDs to Fixed RGB Color --------- */
static void WS2812_SetAll(uint8_t r, uint8_t g, uint8_t b)
{
    for (uint32_t i = 0; i < NUM_LEDS; i++)
        g_leds[i] = (LED_t){r, g, b};

    WS2812B_Send();
}


/* ============================================================
 * HSV to RGB Colour Space Conversion
 * ============================================================
 *
 * Algorithm:
 *   Converts HSV (Hue, Saturation, Value) to RGB space
 *   with fixed Saturation=100% and Value=100%
 *   (pure, full-brightness rainbow colours)
 *
 * Input:
 *   • H : Hue angle in degrees [0°-359°]
 *   • S : Fixed at 255 (100% saturation)
 *   • V : Fixed at 255 (100% value/brightness)
 *
 * Hue Sectors (6 segments):
 *   • [0°-60°)   : Red→Yellow   (R=255, G↑)
 *   • [60°-120°) : Yellow→Green (R↓, G=255)
 *   • [120°-180°): Green→Cyan   (G=255, B↑)
 *   • [180°-240°): Cyan→Blue    (G↓, B=255)
 *   • [240°-300°): Blue→Magenta (B=255, R↑)
 *   • [300°-360°): Magenta→Red  (B↓, R=255)
 *
 * ============================================================ */
static void WS2812_SetHue(uint16_t H)
{
    /* Wrap hue to [0°-359°] */
    H = H % 360U;

    /* Determine hue sector [0-5] and fractional position */
    uint8_t seg  = (uint8_t)(H / 60U);
    uint8_t frac = (uint8_t)(H % 60U);

    /* Interpolation values for RGB components */
    uint8_t q = (uint8_t)(255U * (60U - frac) / 60U);
    uint8_t t = (uint8_t)(255U * frac / 60U);

    uint8_t r, g, b;

    /* Apply hue sector transformation */
    switch (seg)
    {
        case 0: r=255; g=t;   b=0;   break;  /* Red → Yellow */
        case 1: r=q;   g=255; b=0;   break;  /* Yellow → Green */
        case 2: r=0;   g=255; b=t;   break;  /* Green → Cyan */
        case 3: r=0;   g=q;   b=255; break;  /* Cyan → Blue */
        case 4: r=t;   g=0;   b=255; break;  /* Blue → Magenta */
        case 5: r=255; g=0;   b=q;   break;  /* Magenta → Red */
        default:r=0;   g=0;   b=0;   break;  /* Should not occur */
    }

    /* Apply RGB colour to all LEDs and transmit */
    WS2812_SetAll(r, g, b);
}


/* ============================================================
 * 10-Colour Static Palette
 * ============================================================
 *
 * Standard colour definitions used for demonstration.
 * Each entry includes:
 *   • Human-readable name
 *   • Red, Green, Blue intensity (0-255)
 *
 * ============================================================ */
typedef struct {
    const char *name;
    uint8_t r, g, b;
} Colour_t;

static const Colour_t palette[] =
{
    {"Red",        255,   0,   0},
    {"Green",        0, 255,   0},
    {"Blue",         0,   0, 255},
    {"Yellow",     255, 255,   0},
    {"Cyan",         0, 255, 255},
    {"Magenta",    255,   0, 255},
    {"White",      255, 255, 255},
    {"Warm White", 255, 200,  80},
    {"DU Blue",     31,  56, 100},
    {"Off",          0,   0,   0},
};

#define PALETTE_COUNT  (sizeof(palette) / sizeof(palette[0]))


/* ============================================================
 * MAIN PROGRAM
 * ============================================================
 *
 * Execution Flow:
 *   1. Initialize system clock (180 MHz)
 *   2. Initialize UART2 for console logging
 *   3. Initialize TIM6 (system timer)
 *   4. Configure GPIO, TIM1, and DMA2
 *   5. Execute three demonstration sequences:
 *      a) 10-colour static palette cycle
 *      b) HSV rainbow hue sweep
 *      c) Colour chase animation (4 LEDs, 3 rounds)
 *   6. Loop indefinitely
 *
 * ============================================================ */
int main(void)
{
    char buf[128];

    /* Initialize system clock, UART, and timers */
    SystemClock_Config();
    USART2_Init();
    TIM6_Init();

    /* Initialize WS2812B interface hardware */
    GPIO_WS_Init();
    TIM1_WS_Init();
    DMA2_WS_Init();

    /* Display welcome banner */
    USART2_SendString(
    "\r\n"
    "============================================================\r\n"
    "      CSE 2206 — Lab-02 Task 5 : WS2812B + DMA\r\n"
    "============================================================\r\n"
    "Board      : STM32F446RE Nucleo-64\r\n"
    "Protocol   : WS2812B (NeoPixel)\r\n"
    "Interface  : TIM1_CH1 PWM + DMA2 Stream1\r\n"
    "PWM Pin    : PA8  (AF1)\r\n"
    "LED Count  : 5\r\n"
    "Timer CLK  : 180 MHz\r\n"
    "============================================================\r\n"
    "\r\n"
    );

    /* Initialize LEDs to OFF state */
    WS2812_SetAll(0, 0, 0);
    delay_ms(100U);

    /* ============================================================
     * REQUIREMENT 1: Static 10-Colour Palette Demonstration
     * ============================================================
     *
     * Description:
     *   Cycle through all 10 colours in the palette, displaying
     *   each colour on all 5 LEDs for 1 second with formatted
     *   logging to console.
     *
     * Parameters:
     *   • Delay      : 1000 ms per colour
     *   • LED Target : All 5 LEDs (uniform colour)
     *   • Pattern    : Colour → Delay → Log → Repeat
     *
     * ============================================================ */
    USART2_SendString(
    "\r\n"
    "------------------------------------------------------------\r\n"
    "[REQ-1] 10-Colour Palette Demonstration\r\n"
    "------------------------------------------------------------\r\n"
    "Mode        : Static Full-Strip Colour\r\n"
    "Delay       : 1000 ms per colour\r\n"
    "LED Target  : All 5 LEDs\r\n"
    "------------------------------------------------------------\r\n"
    "\r\n"
    );

    for (uint32_t i = 0; i < PALETTE_COUNT; i++)
    {
        /* Set all LEDs to current palette colour */
        WS2812_SetAll(
            palette[i].r,
            palette[i].g,
            palette[i].b
        );

        /* Log colour information to UART */
        snprintf(buf, sizeof(buf),
            "[%02lu/%02lu] %-12s | "
            "RGB=(%3u,%3u,%3u) | "
            "GRB=[0x%02X 0x%02X 0x%02X]\r\n",
            (unsigned long)(i + 1U),
            (unsigned long)PALETTE_COUNT,
            palette[i].name,
            palette[i].r,
            palette[i].g,
            palette[i].b,
            palette[i].g,
            palette[i].r,
            palette[i].b);

        USART2_SendString(buf);

        /* Hold colour for 1 second */
        delay_ms(1000U);
    }

    /* ============================================================
     * REQUIREMENT 2: HSV Rainbow Hue Sweep Animation
     * ============================================================
     *
     * Description:
     *   Perform a smooth rainbow hue sweep from 0° to 359°,
     *   creating a continuous spectrum effect across all 5 LEDs.
     *
     * Parameters:
     *   • Hue Range  : 0° → 359°
     *   • Step Size  : 3° (120 steps for full sweep)
     *   • Frame Delay: 25 ms per frame
     *   • Duration   : ~3 seconds (120 × 25 ms)
     *   • Effect     : Smooth rainbow colour transition
     *
     * ============================================================ */
    USART2_SendString(
    "\r\n"
    "------------------------------------------------------------\r\n"
    "[REQ-2] HSV Hue Sweep Demonstration\r\n"
    "------------------------------------------------------------\r\n"
    "Hue Range   : 0° → 359°\r\n"
    "Step Size   : 3°\r\n"
    "Frame Delay : 25 ms\r\n"
    "Effect      : Smooth Rainbow Transition\r\n"
    "------------------------------------------------------------\r\n"
    "\r\n"
    );

    for (uint16_t h = 0; h < 360U; h += 3U)
    {
        /* Set all LEDs to current hue (100% saturation & value) */
        WS2812_SetHue(h);

        /* Log hue angle */
        snprintf(buf, sizeof(buf),
            "Hue Angle : %3u deg\r\n",
            h);

        USART2_SendString(buf);

        /* Frame delay for animation smoothness */
        delay_ms(25U);
    }

    /* Turn off all LEDs after hue sweep */
    WS2812_SetAll(0, 0, 0);

    USART2_SendString(
    "\r\n"
    "[OK] Hue Sweep Complete.\r\n"
    );

    /* ============================================================
     * REQUIREMENT 3: Colour Chase Animation
     * ============================================================
     *
     * Description:
     *   Animate a rotating red LED across the first 4 LEDs,
     *   with LED-5 always remaining OFF. Perform 3 complete
     *   chase cycles (12 total LED activations).
     *
     * Parameters:
     *   • Active LEDs : 4 (LED-0 through LED-3)
     *   • Total Rounds: 3 full cycles
     *   • Step Delay  : 200 ms per LED activation
     *   • Pattern     : LED-0 → LED-1 → LED-2 → LED-3 → repeat
     *   • LED-5       : Always OFF
     *   • Total Time  : 3 × 4 × 200 ms = 2.4 seconds
     *
     * ============================================================ */
    USART2_SendString(
    "\r\n"
    "------------------------------------------------------------\r\n"
    "[REQ-3] Colour Chase Animation\r\n"
    "------------------------------------------------------------\r\n"
    "Active LEDs : 4\r\n"
    "Rounds      : 3\r\n"
    "Step Delay  : 200 ms\r\n"
    "Pattern     : Rotating RED LED\r\n"
    "LED-5       : Always OFF\r\n"
    "------------------------------------------------------------\r\n"
    "\r\n"
    );

    for (int round = 0; round < 3; round++)
    {
        for (uint32_t active = 0; active < 4U; active++)
        {
            /* Clear all LEDs */
            for (uint32_t j = 0; j < NUM_LEDS; j++)
                g_leds[j] = (LED_t){0, 0, 0};

            /* Activate red LED at current position */
            g_leds[active] = (LED_t){255, 0, 0};

            /* Transmit to WS2812B strip */
            WS2812B_Send();

            /* Log active LED position */
            snprintf(buf, sizeof(buf),
                "[Round %d/3] Active LED --> LED-%lu\r\n",
                round + 1,
                (unsigned long)(active + 1U));

            USART2_SendString(buf);

            /* Delay before moving to next LED */
            delay_ms(200U);
        }
    }

    /* Turn off all LEDs */
    WS2812_SetAll(0, 0, 0);

    /* Display completion banner */
    USART2_SendString(
    "\r\n"
    "============================================================\r\n"
    "               ALL DEMONSTRATIONS COMPLETE\r\n"
    "============================================================\r\n"
    "\r\n"
    );

    /* Loop indefinitely */
    while (1)
    {
        /* System idle */
    }
}
