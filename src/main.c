// ============================================================
// main.c ÃƒÆ’Ã‚Â¢?? Display "Hello" on ST7735 1.8" TFT (128x160)
//          PIC32CM5164LS00048 @ 48 MHz  (SERCOM1 SPI)
//
// FULLY SELF-CONTAINED ÃƒÆ’Ã‚Â¢?? no external ST7735 library needed.
//
// Pin wiring:
//   SDA (MOSI)  -> PA08  (SERCOM1 PAD[0])
//   SCK (Clock) -> PA09  (SERCOM1 PAD[1])
//   CS          -> PA10  (GPIO)
//   DC          -> PA11  (GPIO)
//   RESET       -> PA12  (GPIO)
//   LED / VCC   -> 3.3 V
// ============================================================

#include "config/default/definitions.h"
#include "config/default/peripheral/adc/plib_adc.h"
#include "config/default/peripheral/sercom/usart/plib_sercom3_usart.h"
#include "logo.h" // RGB565 bitmap
#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

/* ---- WiFi + MQTT (ESP-01S via SERCOM5 PB16/PB17) ---- */
#include "esp8266_at.c"   /* unity-build: same pattern as initialization.c uses for SERCOM3 */
#include "wifi_mqtt.c"

// TrustZone Native Register Aliases (using standard addressing)

// PA17 button helper – reads bit 17 of PORT GROUP[0] IN register (Secure alias)
#define PA17_IS_HIGH() ((PORT_SEC_REGS->GROUP[0].PORT_IN >> 17u) & 1u)

static void delay_us(int us) {
  for (volatile int i = 0; i < us * 10; i++)
    ;
}

static bool check_button_toggle(void);

static void adc_init(void) {
// Just use a tiny delay instead of polling SERCOM3_USART_WriteIsBusy
// so we don't accidentally hang there if it faults.
#define P(str)                                                                 \
  do {                                                                         \
    while (SERCOM3_USART_WriteIsBusy())                                        \
      ;                                                                        \
    SERCOM3_USART_Write((uint8_t *)str, strlen(str));                          \
  } while (0)

  P("adc_init 1\r\n");
  // Force pin PMUX to Analog (Function B = 0x1) for active sensors
  // PA02 = AIN0 (Dust), PA03 = GPIO (Dust LED)
  PORT_SEC_REGS->GROUP[0].PORT_PMUX[1] = 0x01U;
  // PA04 = AIN2 (CO2), PA05 = AIN3 (MQ131)
  PORT_SEC_REGS->GROUP[0].PORT_PMUX[2] = 0x11U;
  // PA06 = AIN4 (MQ7), PA07 = AIN5 (MQ135)
  PORT_SEC_REGS->GROUP[0].PORT_PMUX[3] = 0x11U;

  // Enable the Multiplexer for all these specific analog pins
  PORT_SEC_REGS->GROUP[0].PORT_PINCFG[2] |= PORT_PINCFG_PMUXEN_Msk;
  // PA03 does NOT get PMUXEN (Digital GPIO)
  PORT_SEC_REGS->GROUP[0].PORT_PINCFG[4] |= PORT_PINCFG_PMUXEN_Msk;
  PORT_SEC_REGS->GROUP[0].PORT_PINCFG[5] |= PORT_PINCFG_PMUXEN_Msk;
  PORT_SEC_REGS->GROUP[0].PORT_PINCFG[6] |= PORT_PINCFG_PMUXEN_Msk;
  PORT_SEC_REGS->GROUP[0].PORT_PINCFG[7] |= PORT_PINCFG_PMUXEN_Msk;

  // Do the same for the non-secure alias just to be incredibly robust
  PORT_REGS->GROUP[0].PORT_PMUX[1] = 0x01U;
  PORT_REGS->GROUP[0].PORT_PMUX[2] = 0x11U;
  PORT_REGS->GROUP[0].PORT_PMUX[3] = 0x11U;
  PORT_REGS->GROUP[0].PORT_PINCFG[2] |= PORT_PINCFG_PMUXEN_Msk;
  // PA03 is GPIO
  PORT_REGS->GROUP[0].PORT_PINCFG[4] |= PORT_PINCFG_PMUXEN_Msk;
  PORT_REGS->GROUP[0].PORT_PINCFG[5] |= PORT_PINCFG_PMUXEN_Msk;
  PORT_REGS->GROUP[0].PORT_PINCFG[6] |= PORT_PINCFG_PMUXEN_Msk;
  PORT_REGS->GROUP[0].PORT_PINCFG[7] |= PORT_PINCFG_PMUXEN_Msk;

  // Enable APB clock for ADC
  MCLK_REGS->MCLK_APBCMASK |= (1 << 14);

  P("adc_init 2\r\n");
  // Enable GCLK for ADC (Generator 0)
  GCLK_REGS->GCLK_PCHCTRL[28] =
      0x40; // GCLK_PCHCTRL_CHEN_Msk is 0x40 (bit 6), GEN(0) is 0
  while ((GCLK_REGS->GCLK_PCHCTRL[28] & 0x40) == 0)
    ;

  P("adc_init 3\r\n");
  // Reset ADC
  ADC_REGS->ADC_CTRLA = 1; // SWRST
  while (ADC_REGS->ADC_SYNCBUSY & 1)
    ;

  P("adc_init 4\r\n");
  ADC_REGS->ADC_CTRLB = 4;        // DIV32
  ADC_REGS->ADC_REFCTRL = 0x05;   // INTVCC2 (3.3V)
  ADC_REGS->ADC_CTRLC = (1 << 4); // 12-bit
  ADC_REGS->ADC_SAMPCTRL =
      63; // Maximize sampling time for high-impedance MQ sensors

  // Enable ADC
  ADC_REGS->ADC_CTRLA |= 2; // ENABLE
  while (ADC_REGS->ADC_SYNCBUSY & 2)
    ;

  P("adc_init 5\r\n");
}

// Uses direct hardware registers to guarantee conversion completion
static uint16_t read_adc_avg(uint8_t channel) {
  static uint8_t last_channel = 0xFF; // Keep track of the multiplexer state

  // Switch the multiplexer to the new channel
  ADC_REGS->ADC_INPUTCTRL =
      channel | (0x18 << 8); // Positive = channel, Negative = GND
  while (ADC_REGS->ADC_SYNCBUSY & (1 << 2)) // INPUTCTRL sync
    ;

  // If we just switched to a new sensor, the internal ADC capacitor needs time
  // to charge to the new voltage, especially for high-impedance gas sensors!
  if (channel != last_channel) {
    delay_us(200); // Allow physical voltage to settle through the multiplexer

    // Perform one dummy conversion to flush the ADC pipeline/capacitor
    ADC_REGS->ADC_SWTRIG |= (1 << 1);
    while (ADC_REGS->ADC_SYNCBUSY & (1 << 10))
      ;
    while ((ADC_REGS->ADC_INTFLAG & (1 << 0)) == 0)
      ;
    uint16_t dummy = ADC_REGS->ADC_RESULT;
    (void)dummy;
    ADC_REGS->ADC_INTFLAG = (1 << 0); // clear flag

    last_channel = channel;
  }

  uint32_t sum = 0;
  for (int i = 0; i < 5; i++) {
    ADC_REGS->ADC_SWTRIG |= (1 << 1);          // START bit
    while (ADC_REGS->ADC_SYNCBUSY & (1 << 10)) // SWTRIG sync
      ;

    while ((ADC_REGS->ADC_INTFLAG & (1 << 0)) == 0) // Wait for RESRDY
      ;

    sum += ADC_REGS->ADC_RESULT;
    ADC_REGS->ADC_INTFLAG = (1 << 0); // clear flag
    delay_us(50);
  }
  return sum / 5;
}

static float adc_to_voltage(uint16_t adc_value) {
  if (adc_value == 0)
    adc_value = 1; // Prevent 0V
  if (adc_value >= 4095)
    adc_value = 4094; // Prevent 3.3V division by zero
  return (adc_value / 4095.0f) * 3.3f;
}

static float calculate_Rs(float voltage) {
  // Guard against divide by zero or negative resistance
  if (voltage <= 0.01f)
    voltage = 0.01f;
  if (voltage >= 3.29f)
    voltage = 3.29f;

  float Rs = ((3.3f - voltage) / voltage) * 10000.0f;
  if (Rs < 0.1f)
    return 0.1f;
  return Rs;
}

static float read_dust() {
  float sum = 0;
  for (int i = 0; i < 5; i++) {
    PORT_REGS->GROUP[0].PORT_OUTSET = (1U << 3); // PA03 HIGH (Dust LED)
    delay_us(280);

    uint16_t adc = read_adc_avg(0); // AIN0 (PA02)

    delay_us(40);
    PORT_REGS->GROUP[0].PORT_OUTCLR = (1U << 3); // PA03 LOW
    delay_us(100);

    sum += adc_to_voltage(adc);
  }

  float voltage = sum / 5.0f;
  static float baseline = 0;
  if (baseline == 0)
    baseline = voltage;
  baseline = (baseline * 0.99f) + (voltage * 0.01f);

  float dust = (voltage - baseline) * 1000.0f / 0.5f;
  if (dust < 0)
    dust = 0;
  if (dust > 300)
    dust = 300;
  return dust;
}

static float read_mg811_co2() {
  uint16_t adc = read_adc_avg(2); // PA04 is AIN2
  float voltage = adc_to_voltage(adc);
  float co2 = 400.0f + voltage * 500.0f;
  if (co2 < 400.0f)
    co2 = 400.0f;
  if (co2 > 2000.0f)
    co2 = 2000.0f;
  return co2;
}

// ======================== DELAY =============================
static void delay_ms(uint32_t ms) {
  for (uint32_t i = 0; i < ms; i++)
    for (volatile uint32_t j = 0; j < 12000UL; j++)
      ;
}

// ======================== GPIO HELPERS ======================
// Software Bit-Banged SPI ÃƒÆ’Ã‚Â¢?? Bypasses all SERCOM and TrustZone issues.
// MOSI = PA08, SCK = PA09

// TRUSTZONE-PROOF GPIO MACROS:
// These write to BOTH Secure and Non-Secure register aliases.
// TrustZone silently ignores alias writes that don't match the current CPU
// state. Writing to both guarantees the pins will toggle no matter what mode
// the MCU is in!
#define TFT_MOSI_HIGH()                                                        \
  do {                                                                         \
    PORT_SEC_REGS->GROUP[0].PORT_OUTSET = (1U << 8U);                          \
    PORT_REGS->GROUP[0].PORT_OUTSET = (1U << 8U);                              \
  } while (0)
#define TFT_MOSI_LOW()                                                         \
  do {                                                                         \
    PORT_SEC_REGS->GROUP[0].PORT_OUTCLR = (1U << 8U);                          \
    PORT_REGS->GROUP[0].PORT_OUTCLR = (1U << 8U);                              \
  } while (0)

#define TFT_SCK_HIGH()                                                         \
  do {                                                                         \
    PORT_SEC_REGS->GROUP[0].PORT_OUTSET = (1U << 9U);                          \
    PORT_REGS->GROUP[0].PORT_OUTSET = (1U << 9U);                              \
  } while (0)
#define TFT_SCK_LOW()                                                          \
  do {                                                                         \
    PORT_SEC_REGS->GROUP[0].PORT_OUTCLR = (1U << 9U);                          \
    PORT_REGS->GROUP[0].PORT_OUTCLR = (1U << 9U);                              \
  } while (0)

#define TFT_CS_HIGH()                                                          \
  do {                                                                         \
    PORT_SEC_REGS->GROUP[0].PORT_OUTSET = (1U << 10U);                         \
    PORT_REGS->GROUP[0].PORT_OUTSET = (1U << 10U);                             \
  } while (0)
#define TFT_CS_LOW()                                                           \
  do {                                                                         \
    PORT_SEC_REGS->GROUP[0].PORT_OUTCLR = (1U << 10U);                         \
    PORT_REGS->GROUP[0].PORT_OUTCLR = (1U << 10U);                             \
  } while (0)

#define TFT_DC_HIGH()                                                          \
  do {                                                                         \
    PORT_SEC_REGS->GROUP[0].PORT_OUTSET = (1U << 11U);                         \
    PORT_REGS->GROUP[0].PORT_OUTSET = (1U << 11U);                             \
  } while (0)
#define TFT_DC_LOW()                                                           \
  do {                                                                         \
    PORT_SEC_REGS->GROUP[0].PORT_OUTCLR = (1U << 11U);                         \
    PORT_REGS->GROUP[0].PORT_OUTCLR = (1U << 11U);                             \
  } while (0)

#define TFT_RST_HIGH()                                                         \
  do {                                                                         \
    PORT_SEC_REGS->GROUP[0].PORT_OUTSET = (1U << 12U);                         \
    PORT_REGS->GROUP[0].PORT_OUTSET = (1U << 12U);                             \
  } while (0)
#define TFT_RST_LOW()                                                          \
  do {                                                                         \
    PORT_SEC_REGS->GROUP[0].PORT_OUTCLR = (1U << 12U);                         \
    PORT_REGS->GROUP[0].PORT_OUTCLR = (1U << 12U);                             \
  } while (0)

#define PIN_OUTPUT_ENABLE(pin)                                                 \
  do {                                                                         \
    PORT_SEC_REGS->GROUP[0].PORT_DIRSET = (1U << pin);                         \
    PORT_REGS->GROUP[0].PORT_DIRSET = (1U << pin);                             \
  } while (0)

static void spi_send(uint8_t byte) {
  for (uint8_t i = 0; i < 8; i++) {
    // Mode 3 (CKP=1, CKE=0): Clock idles HIGH.
    // Data changes on High->Low transition, sampled on Low->High.
    TFT_SCK_LOW();

    if (byte & 0x80) {
      TFT_MOSI_HIGH();
    } else {
      TFT_MOSI_LOW();
    }
    byte <<= 1;

    // Tiny delay for Data Setup Time
    for (volatile int d = 0; d < 0; d++)
      ;

    // Clock High (sampled on this rising edge)
    TFT_SCK_HIGH();

    // Tiny delay for Clock High Time
    for (volatile int d = 0; d < 0; d++)
      ;
  }
}

// ======================== ST7735 LOW-LEVEL ==================
static void tft_write_cmd(uint8_t cmd) {
  TFT_DC_LOW();
  for (volatile int d = 0; d < 0; d++)
    ; // delay
  TFT_CS_LOW();
  for (volatile int d = 0; d < 0; d++)
    ; // delay
  spi_send(cmd);
  TFT_CS_HIGH();
  for (volatile int d = 0; d < 0; d++)
    ; // delay
}

static void tft_write_data(uint8_t data) {
  TFT_DC_HIGH();
  for (volatile int d = 0; d < 0; d++)
    ; // delay
  TFT_CS_LOW();
  for (volatile int d = 0; d < 0; d++)
    ; // delay
  spi_send(data);
  TFT_CS_HIGH();
  for (volatile int d = 0; d < 0; d++)
    ; // delay
}
#define ST7735_SWRESET 0x01
#define ST7735_SLPOUT 0x11
#define ST7735_FRMCTR1 0xB1
#define ST7735_FRMCTR2 0xB2
#define ST7735_FRMCTR3 0xB3
#define ST7735_INVCTR 0xB4
#define ST7735_PWCTR1 0xC0
#define ST7735_PWCTR2 0xC1
#define ST7735_PWCTR3 0xC2
#define ST7735_PWCTR4 0xC3
#define ST7735_PWCTR5 0xC4
#define ST7735_VMCTR1 0xC5
#define ST7735_INVOFF 0x20
#define ST7735_MADCTL 0x36
#define ST7735_COLMOD 0x3A
#define ST7735_CASET 0x2A
#define ST7735_RASET 0x2B
#define ST7735_RAMWR 0x2C
#define ST7735_GMCTRP1 0xE0
#define ST7735_GMCTRN1 0xE1
#define ST7735_NORON 0x13
#define ST7735_DISPON 0x29

// Colors (RGB565)
#define COLOR_BLACK 0x0000
#define COLOR_WHITE 0xFFFF
#define COLOR_RED 0xF800
#define COLOR_GREEN 0x07E0
#define COLOR_BLUE 0x001F
#define COLOR_YELLOW 0xFFE0

static void tft_reset(void) {
  // Not used directly, handled in tft_init()
}

static void tft_init(void) {
  // -------------------------------------------------------------
  // EXACT PORT from Github: ArmstrongSubero/PIC32-Projects ST7735
  // -------------------------------------------------------------
  TFT_RST_HIGH();
  delay_ms(1); // 500us
  TFT_RST_LOW();
  delay_ms(1); // 500us
  TFT_RST_HIGH();
  delay_ms(1); // 500us

  TFT_CS_LOW();

  tft_write_cmd(ST7735_SWRESET); // software reset
  delay_ms(150);

  tft_write_cmd(ST7735_SLPOUT); // out of sleep mode
  delay_ms(500);

  tft_write_cmd(ST7735_COLMOD); // set color mode
  tft_write_data(0x05);         // 16-bit color
  delay_ms(1);                  // 10us

  tft_write_cmd(ST7735_FRMCTR1); // frame rate control - normal mode
  tft_write_data(0x01); // frame rate = fosc / (1 x 2 + 40) * (LINE + 2C + 2D)
  tft_write_data(0x2C);
  tft_write_data(0x2D);

  tft_write_cmd(ST7735_FRMCTR2); // frame rate control - idle mode
  tft_write_data(0x01);
  tft_write_data(0x2C);
  tft_write_data(0x2D);

  tft_write_cmd(ST7735_FRMCTR3); // frame rate control - partial mode
  tft_write_data(0x01);          // dot inversion mode
  tft_write_data(0x2C);
  tft_write_data(0x2D);
  tft_write_data(0x01); // line inversion mode
  tft_write_data(0x2C);
  tft_write_data(0x2D);

  tft_write_cmd(ST7735_INVCTR); // display inversion control
  tft_write_data(0x07);         // no inversion

  tft_write_cmd(ST7735_PWCTR1); // power control
  tft_write_data(0xA2);
  tft_write_data(0x02); // -4.6V
  tft_write_data(0x84); // AUTO mode

  tft_write_cmd(ST7735_PWCTR2); // power control
  tft_write_data(0xC5);         // VGH25 = 2.4C VGSEL = -10 VGH = 3 * AVDD

  tft_write_cmd(ST7735_PWCTR3); // power control
  tft_write_data(0x0A);         // Opamp current small
  tft_write_data(0x00);         // Boost frequency

  tft_write_cmd(ST7735_PWCTR4); // power control
  tft_write_data(0x8A);         // BCLK/2, Opamp current small & Medium low
  tft_write_data(0x2A);

  tft_write_cmd(ST7735_PWCTR5); // power control
  tft_write_data(0x8A);
  tft_write_data(0xEE);

  tft_write_cmd(ST7735_VMCTR1); // power control
  tft_write_data(0x0E);

  tft_write_cmd(ST7735_INVOFF); // don't invert display

  tft_write_cmd(ST7735_MADCTL); // memory access control
  tft_write_data(0xA0);         // Landscape: 160 wide x 128 tall

  tft_write_cmd(ST7735_COLMOD); // set color mode
  tft_write_data(0x05);         // 16-bit color

  tft_write_cmd(ST7735_CASET); // column addr set (landscape: 0..159)
  tft_write_data(0x00);
  tft_write_data(0x00); // XSTART = 0
  tft_write_data(0x00);
  tft_write_data(0x9F); // XEND = 159

  tft_write_cmd(ST7735_RASET); // row addr set (landscape: 0..127)
  tft_write_data(0x00);
  tft_write_data(0x00); // YSTART = 0
  tft_write_data(0x00);
  tft_write_data(0x7F); // YEND = 127

  // Gamma Adjustments - Exact from ArmstrongSubero
  tft_write_cmd(ST7735_GMCTRP1);
  tft_write_data(0x0f);
  tft_write_data(0x1a);
  tft_write_data(0x0f);
  tft_write_data(0x18);
  tft_write_data(0x2f);
  tft_write_data(0x28);
  tft_write_data(0x20);
  tft_write_data(0x22);
  tft_write_data(0x1f);
  tft_write_data(0x1b);
  tft_write_data(0x23);
  tft_write_data(0x37);
  tft_write_data(0x00);
  tft_write_data(0x07);
  tft_write_data(0x02);
  tft_write_data(0x10);

  tft_write_cmd(ST7735_GMCTRN1);
  tft_write_data(0x0f);
  tft_write_data(0x1b);
  tft_write_data(0x0f);
  tft_write_data(0x17);
  tft_write_data(0x33);
  tft_write_data(0x2c);
  tft_write_data(0x29);
  tft_write_data(0x2e);
  tft_write_data(0x30);
  tft_write_data(0x30);
  tft_write_data(0x39);
  tft_write_data(0x3f);
  tft_write_data(0x00);
  tft_write_data(0x07);
  tft_write_data(0x03);
  tft_write_data(0x10);

  tft_write_cmd(0xF6); // Disable ram power save mode
  tft_write_data(0x00);

  tft_write_cmd(ST7735_DISPON);
  delay_ms(100);

  tft_write_cmd(ST7735_NORON); // normal display on
  delay_ms(10);

  // Return CS high
  TFT_CS_HIGH();
}

// ======================== DRAWING FUNCTIONS ==================
static void tft_set_window(uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1) {
  tft_write_cmd(ST7735_CASET);
  tft_write_data(0x00);
  tft_write_data(x0);
  tft_write_data(0x00);
  tft_write_data(x1);

  tft_write_cmd(ST7735_RASET);
  tft_write_data(0x00);
  tft_write_data(y0);
  tft_write_data(0x00);
  tft_write_data(y1);

  tft_write_cmd(ST7735_RAMWR);
}

static void tft_fill_screen(uint16_t color) {
  uint8_t hi = color >> 8;
  uint8_t lo = color & 0xFF;

  tft_set_window(0, 0, 159, 127);

  TFT_DC_HIGH();
  TFT_CS_LOW();
  for (uint32_t i = 0; i < (160UL * 128UL); i++) {
    spi_send(hi);
    spi_send(lo);
  }
  TFT_CS_HIGH();
}

static void tft_draw_pixel(uint8_t x, uint8_t y, uint16_t color) {
  if (x >= 160 || y >= 128)
    return;
  tft_set_window(x, y, x, y);
  tft_write_data(color >> 8);
  tft_write_data(color & 0xFF);
}

static void tft_fill_rect(uint8_t x, uint8_t y, uint8_t w, uint8_t h,
                          uint16_t color) {
  uint8_t hi = color >> 8;
  uint8_t lo = color & 0xFF;

  if (x >= 160 || y >= 128)
    return;
  if (x + w > 160)
    w = 160 - x;
  if (y + h > 128)
    h = 128 - y;

  tft_set_window(x, y, x + w - 1, y + h - 1);

  TFT_DC_HIGH();
  TFT_CS_LOW();
  for (uint16_t i = 0; i < (uint16_t)w * h; i++) {
    spi_send(hi);
    spi_send(lo);
  }
  TFT_CS_HIGH();
}

// ======================== 5x7 FONT ==========================
static const uint8_t font5x7[] = {
    0x00, 0x00, 0x00, 0x00, 0x00, // (space)
    0x00, 0x00, 0x5F, 0x00, 0x00, // !
    0x00, 0x07, 0x00, 0x07, 0x00, // "
    0x14, 0x7F, 0x14, 0x7F, 0x14, // #
    0x24, 0x2A, 0x7F, 0x2A, 0x12, // $
    0x23, 0x13, 0x08, 0x64, 0x62, // %
    0x36, 0x49, 0x55, 0x22, 0x50, // &
    0x00, 0x05, 0x03, 0x00, 0x00, // '
    0x00, 0x1C, 0x22, 0x41, 0x00, // (
    0x00, 0x41, 0x22, 0x1C, 0x00, // )
    0x08, 0x2A, 0x1C, 0x2A, 0x08, // *
    0x08, 0x08, 0x3E, 0x08, 0x08, // +
    0x00, 0x50, 0x30, 0x00, 0x00, // ,
    0x08, 0x08, 0x08, 0x08, 0x08, // -
    0x00, 0x60, 0x60, 0x00, 0x00, // .
    0x20, 0x10, 0x08, 0x04, 0x02, // /
    0x3E, 0x51, 0x49, 0x45, 0x3E, // 0
    0x00, 0x42, 0x7F, 0x40, 0x00, // 1
    0x42, 0x61, 0x51, 0x49, 0x46, // 2
    0x21, 0x41, 0x45, 0x4B, 0x31, // 3
    0x18, 0x14, 0x12, 0x7F, 0x10, // 4
    0x27, 0x45, 0x45, 0x45, 0x39, // 5
    0x3C, 0x4A, 0x49, 0x49, 0x30, // 6
    0x01, 0x71, 0x09, 0x05, 0x03, // 7
    0x36, 0x49, 0x49, 0x49, 0x36, // 8
    0x06, 0x49, 0x49, 0x29, 0x1E, // 9
    0x00, 0x36, 0x36, 0x00, 0x00, // :
    0x00, 0x56, 0x36, 0x00, 0x00, // ;
    0x00, 0x08, 0x14, 0x22, 0x41, // <
    0x14, 0x14, 0x14, 0x14, 0x14, // =
    0x41, 0x22, 0x14, 0x08, 0x00, // >
    0x02, 0x01, 0x51, 0x09, 0x06, // ?
    0x32, 0x49, 0x79, 0x41, 0x3E, // @
    0x7E, 0x11, 0x11, 0x11, 0x7E, // A
    0x7F, 0x49, 0x49, 0x49, 0x36, // B
    0x3E, 0x41, 0x41, 0x41, 0x22, // C
    0x7F, 0x41, 0x41, 0x22, 0x1C, // D
    0x7F, 0x49, 0x49, 0x49, 0x41, // E
    0x7F, 0x09, 0x09, 0x01, 0x01, // F
    0x3E, 0x41, 0x41, 0x51, 0x32, // G
    0x7F, 0x08, 0x08, 0x08, 0x7F, // H
    0x00, 0x41, 0x7F, 0x41, 0x00, // I
    0x20, 0x40, 0x41, 0x3F, 0x01, // J
    0x7F, 0x08, 0x14, 0x22, 0x41, // K
    0x7F, 0x40, 0x40, 0x40, 0x40, // L
    0x7F, 0x02, 0x04, 0x02, 0x7F, // M
    0x7F, 0x04, 0x08, 0x10, 0x7F, // N
    0x3E, 0x41, 0x41, 0x41, 0x3E, // O
    0x7F, 0x09, 0x09, 0x09, 0x06, // P
    0x3E, 0x41, 0x51, 0x21, 0x5E, // Q
    0x7F, 0x09, 0x19, 0x29, 0x46, // R
    0x46, 0x49, 0x49, 0x49, 0x31, // S
    0x01, 0x01, 0x7F, 0x01, 0x01, // T
    0x3F, 0x40, 0x40, 0x40, 0x3F, // U
    0x1F, 0x20, 0x40, 0x20, 0x1F, // V
    0x7F, 0x20, 0x18, 0x20, 0x7F, // W
    0x63, 0x14, 0x08, 0x14, 0x63, // X
    0x03, 0x04, 0x78, 0x04, 0x03, // Y
    0x61, 0x51, 0x49, 0x45, 0x43, // Z
    0x00, 0x00, 0x7F, 0x41, 0x41, // [
    0x02, 0x04, 0x08, 0x10, 0x20, // backslash
    0x41, 0x41, 0x7F, 0x00, 0x00, // ]
    0x04, 0x02, 0x01, 0x02, 0x04, // ^
    0x40, 0x40, 0x40, 0x40, 0x40, // _
    0x00, 0x01, 0x02, 0x04, 0x00, // `
    0x20, 0x54, 0x54, 0x54, 0x78, // a
    0x7F, 0x48, 0x44, 0x44, 0x38, // b
    0x38, 0x44, 0x44, 0x44, 0x20, // c
    0x38, 0x44, 0x44, 0x48, 0x7F, // d
    0x38, 0x54, 0x54, 0x54, 0x18, // e
    0x08, 0x7E, 0x09, 0x01, 0x02, // f
    0x08, 0x14, 0x54, 0x54, 0x3C, // g
    0x7F, 0x08, 0x04, 0x04, 0x78, // h
    0x00, 0x44, 0x7D, 0x40, 0x00, // i
    0x20, 0x40, 0x44, 0x3D, 0x00, // j
    0x00, 0x7F, 0x10, 0x28, 0x44, // k
    0x00, 0x41, 0x7F, 0x40, 0x00, // l
    0x7C, 0x04, 0x18, 0x04, 0x78, // m
    0x7C, 0x08, 0x04, 0x04, 0x78, // n
    0x38, 0x44, 0x44, 0x44, 0x38, // o
    0x7C, 0x14, 0x14, 0x14, 0x08, // p
    0x08, 0x14, 0x14, 0x18, 0x7C, // q
    0x7C, 0x08, 0x04, 0x04, 0x08, // r
    0x48, 0x54, 0x54, 0x54, 0x20, // s
    0x04, 0x3F, 0x44, 0x40, 0x20, // t
    0x3C, 0x40, 0x40, 0x20, 0x7C, // u
    0x1C, 0x20, 0x40, 0x20, 0x1C, // v
    0x3C, 0x40, 0x30, 0x40, 0x3C, // w
    0x44, 0x28, 0x10, 0x28, 0x44, // x
    0x0C, 0x50, 0x50, 0x50, 0x3C, // y
    0x44, 0x64, 0x54, 0x4C, 0x44, // z
    0x00, 0x08, 0x36, 0x41, 0x00, // {
    0x00, 0x00, 0x7F, 0x00, 0x00, // |
    0x00, 0x41, 0x36, 0x08, 0x00, // }
    0x08, 0x08, 0x2A, 0x1C, 0x08, // ~
};

// Draw a single character at (x, y) with given color, bg, and scale
static void tft_draw_char(uint8_t x, uint8_t y, char c, uint16_t fg,
                          uint16_t bg, uint8_t size) {
  if (c < ' ' || c > '~')
    c = '?';
  uint8_t idx = c - ' ';

  for (uint8_t col = 0; col < 5; col++) {
    uint8_t line = font5x7[idx * 5 + col];
    for (uint8_t row = 0; row < 7; row++) {
      uint16_t color = (line & (1 << row)) ? fg : bg;
      if (size == 1) {
        tft_draw_pixel(x + col, y + row, color);
      } else {
        tft_fill_rect(x + col * size, y + row * size, size, size, color);
      }
    }
  }
  // 1-pixel gap between characters (background)
  if (size == 1) {
    for (uint8_t row = 0; row < 7; row++)
      tft_draw_pixel(x + 5, y + row, bg);
  } else {
    tft_fill_rect(x + 5 * size, y, size, 7 * size, bg);
  }
}

// Draw a null-terminated string
static void tft_draw_string(uint8_t x, uint8_t y, const char *str, uint16_t fg,
                            uint16_t bg, uint8_t size) {
  while (*str) {
    tft_draw_char(x, y, *str, fg, bg, size);
    x += 6 * size; // 5 pixel char + 1 pixel gap, times scale
    str++;
  }
}

// ======================== BITMAP DRAW ========================
// Draws the full bitmap.
static void tft_draw_bitmap(uint8_t x, uint8_t y, uint8_t w, uint8_t h,
                            const uint16_t *bitmap) {
  tft_set_window(x, y, x + w - 1, y + h - 1);
  TFT_DC_HIGH();
  TFT_CS_LOW();
  uint32_t total = (uint32_t)w * h;
  for (uint32_t i = 0; i < total; i++) {
    uint16_t color = bitmap[i];
    spi_send((uint8_t)(color >> 8));
    spi_send((uint8_t)(color & 0xFF));
  }
  TFT_CS_HIGH();
}

// Draws only the top rows rows of the bitmap (for wipe animation).
static void tft_draw_bitmap_rows(uint8_t x, uint8_t y, uint8_t w, uint8_t h,
                                 const uint16_t *bitmap, uint8_t rows) {
  if (rows == 0)
    return;
  if (rows > h)
    rows = h;
  tft_set_window(x, y, x + w - 1, y + rows - 1);
  TFT_DC_HIGH();
  TFT_CS_LOW();
  uint32_t total = (uint32_t)w * rows;
  for (uint32_t i = 0; i < total; i++) {
    uint16_t color = bitmap[i];
    spi_send((uint8_t)(color >> 8));
    spi_send((uint8_t)(color & 0xFF));
  }
  TFT_CS_HIGH();
}

// Draws a rectangular sub-region of a bitmap onto the screen.
// dest_x/y  = top-left on screen
// bmp       = full bitmap pointer, bmp_w = bitmap's total width
// src_x/y   = top-left corner within the source bitmap
// draw_w/h  = how many pixels wide/tall to copy
static void tft_draw_bitmap_region(uint8_t dest_x, uint8_t dest_y,
                                   const uint16_t *bmp, uint8_t bmp_w,
                                   uint8_t src_x, uint8_t src_y, uint8_t draw_w,
                                   uint8_t draw_h) {
  tft_set_window(dest_x, dest_y, dest_x + draw_w - 1, dest_y + draw_h - 1);
  TFT_DC_HIGH();
  TFT_CS_LOW();
  for (uint8_t row = 0; row < draw_h; row++) {
    uint32_t offset = (uint32_t)(src_y + row) * bmp_w + src_x;
    for (uint8_t col = 0; col < draw_w; col++) {
      uint16_t color = bmp[offset + col];
      spi_send((uint8_t)(color >> 8));
      spi_send((uint8_t)(color & 0xFF));
    }
  }
  TFT_CS_HIGH();
}

// ======================== CIRCLE DRAW ========================
// Bresenham midpoint circle ÃƒÆ’Ã‚Â¢?? filled
static void tft_fill_circle(uint8_t cx, uint8_t cy, uint8_t r, uint16_t color) {
  int16_t x = 0, y = (int16_t)r, d = 1 - (int16_t)r;
  while (x <= y) {
    // Draw horizontal spans for each octant pair
    int16_t x0, x1, ys;
    // span at ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â±y rows
    ys = (int16_t)cy - y;
    if (ys >= 0) {
      x0 = (int16_t)cx - x;
      if (x0 < 0)
        x0 = 0;
      x1 = (int16_t)cx + x;
      if (x1 > 127)
        x1 = 127;
      if (x0 <= x1)
        tft_fill_rect((uint8_t)x0, (uint8_t)ys, (uint8_t)(x1 - x0 + 1), 1,
                      color);
    }
    ys = (int16_t)cy + y;
    if (ys <= 159) {
      x0 = (int16_t)cx - x;
      if (x0 < 0)
        x0 = 0;
      x1 = (int16_t)cx + x;
      if (x1 > 127)
        x1 = 127;
      if (x0 <= x1)
        tft_fill_rect((uint8_t)x0, (uint8_t)ys, (uint8_t)(x1 - x0 + 1), 1,
                      color);
    }
    // span at ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â±x rows
    ys = (int16_t)cy - x;
    if (ys >= 0) {
      x0 = (int16_t)cx - y;
      if (x0 < 0)
        x0 = 0;
      x1 = (int16_t)cx + y;
      if (x1 > 127)
        x1 = 127;
      if (x0 <= x1)
        tft_fill_rect((uint8_t)x0, (uint8_t)ys, (uint8_t)(x1 - x0 + 1), 1,
                      color);
    }
    ys = (int16_t)cy + x;
    if (ys <= 159) {
      x0 = (int16_t)cx - y;
      if (x0 < 0)
        x0 = 0;
      x1 = (int16_t)cx + y;
      if (x1 > 127)
        x1 = 127;
      if (x0 <= x1)
        tft_fill_rect((uint8_t)x0, (uint8_t)ys, (uint8_t)(x1 - x0 + 1), 1,
                      color);
    }
    if (d < 0) {
      d += 2 * x + 3;
    } else {
      d += 2 * (x - y) + 5;
      y--;
    }
    x++;
  }
}

// ======================== AQI DASHBOARD ========================
#define C_BG 0x0000       // Pure black / very dark background
#define C_TXT_MAIN 0xF800 // Bright red for primary text
#define C_TXT_SUB 0x9000  // Dim red for secondary text
#define C_GRAY 0x52AA     // Grey for offline status
#define C_CIRCLE 0x7800   // Deep red for the main gauge circle
#define C_BAR1 0xF800     // Bright red for bar
#define C_BAR2 0xB000     // Medium red for bar
#define C_BAR3 0x6000     // Dark red for bar

// Rough approximation of the IOTA / microchip dot logo
static void draw_dot_logo(uint8_t cx, uint8_t cy) {
  tft_fill_circle(cx, cy, 3, C_TXT_MAIN);
  tft_fill_circle(cx - 7, cy - 6, 2, C_TXT_MAIN);
  tft_fill_circle(cx + 7, cy - 6, 2, C_TXT_MAIN);
  tft_fill_circle(cx - 7, cy + 6, 2, C_TXT_MAIN);
  tft_fill_circle(cx + 7, cy + 6, 2, C_TXT_MAIN);
  tft_fill_circle(cx - 12, cy, 1, C_TXT_MAIN);
  tft_fill_circle(cx + 12, cy, 1, C_TXT_MAIN);
  tft_fill_circle(cx, cy - 11, 1, C_TXT_MAIN);
  tft_fill_circle(cx, cy + 11, 1, C_TXT_MAIN);
  tft_fill_circle(cx - 12, cy - 9, 1, C_TXT_MAIN);
  tft_fill_circle(cx + 12, cy + 9, 1, C_TXT_MAIN);
  tft_fill_circle(cx + 5, cy - 13, 1, C_TXT_MAIN);
  tft_fill_circle(cx - 5, cy + 13, 1, C_TXT_MAIN);
}

static void tft_draw_circle_outline(int16_t cx, int16_t cy, int16_t r,
                                    uint16_t color) {
  int16_t f = 1 - r;
  int16_t ddF_x = 1;
  int16_t ddF_y = -2 * r;
  int16_t x = 0;
  int16_t y = r;

  tft_draw_pixel(cx, cy + r, color);
  tft_draw_pixel(cx, cy - r, color);
  tft_draw_pixel(cx + r, cy, color);
  tft_draw_pixel(cx - r, cy, color);

  while (x < y) {
    if (f >= 0) {
      y--;
      ddF_y += 2;
      f += ddF_y;
    }
    x++;
    ddF_x += 2;
    f += ddF_x;
    tft_draw_pixel(cx + x, cy + y, color);
    tft_draw_pixel(cx - x, cy + y, color);
    tft_draw_pixel(cx + x, cy - y, color);
    tft_draw_pixel(cx - x, cy - y, color);
    tft_draw_pixel(cx + y, cy + x, color);
    tft_draw_pixel(cx - y, cy + x, color);
    tft_draw_pixel(cx + y, cy - x, color);
    tft_draw_pixel(cx - y, cy - x, color);
  }
}

static uint16_t get_aqi_color(int aqi) {
  if (aqi <= 50)
    return 0x24C8; // Good: Emerald Green
  if (aqi <= 100)
    return 0xEC40; // Moderate: Golden/Orange
  if (aqi <= 150)
    return 0xFA48; // High: Salmon Red
  if (aqi <= 200)
    return 0xE248; // Unhealthy: Distinct Red
  if (aqi <= 300)
    return 0x933B; // Very Unhealthy: Vibrant Purple
  return 0x8186;   // Hazardous: Deep Maroon Red
}

static const char *get_aqi_label(int aqi) {
  if (aqi <= 50)
    return "Good";
  if (aqi <= 100)
    return "Moderate";
  if (aqi <= 150)
    return "High";
  if (aqi <= 200)
    return "Unhealthy";
  if (aqi <= 300)
    return "Very poor";
  return "Hazardous";
}

static void tft_draw_rect(int x, int y, int w, int h, uint16_t color) {
  tft_fill_rect(x, y, w, 1, color);
  tft_fill_rect(x, y + h - 1, w, 1, color);
  tft_fill_rect(x, y, 1, h, color);
  tft_fill_rect(x + w - 1, y, 1, h, color);
}

static void draw_chip(int x, int y, int w, const char *text, uint16_t color) {
  tft_draw_rect(x, y, w, 11, color);
  // Center text roughly in chip
  int len = 0;
  while (text[len] != '\0')
    len++;
  int tw = len * 6;
  tft_draw_string(x + (w - tw) / 2, y + 2, text, color, 0xFFFF, 1);
}

static const uint8_t chalf[43] = {42, 42, 42, 42, 42, 42, 42, 41, 41, 41, 41,
                                  41, 40, 40, 40, 39, 39, 38, 38, 37, 37, 36,
                                  36, 35, 34, 34, 33, 32, 31, 30, 29, 28, 27,
                                  26, 25, 23, 22, 20, 18, 16, 13, 9,  0};
static const int8_t swave[50] = {
    0,  1,  1,  2,  3,  4,  4,  5,  5,  5,  6,  6,  6,  6,  6,  6,  5,
    5,  5,  4,  4,  3,  2,  1,  1,  0,  -1, -1, -2, -3, -4, -4, -5, -5,
    -5, -6, -6, -6, -6, -6, -6, -5, -5, -5, -4, -4, -3, -2, -1, -1};

static uint16_t get_pastel(uint16_t c) {
  uint8_t r = (c >> 11) & 0x1F;
  uint8_t g = (c >> 5) & 0x3F;
  uint8_t b = c & 0x1F;
  // Blend tightly 50/50 with white for a beautiful glass soft pastel effect
  r = (r + 31) / 2;
  g = (g + 63) / 2;
  b = (b + 31) / 2;
  return (r << 11) | (g << 5) | b;
}

static void draw_clipped_column(int x, int y_start, int y_end, uint16_t color,
                                int num_sx, int num_ex) {
  if (y_start > y_end)
    return;
  int run_start = -1;

  for (int y = y_start; y <= y_end + 1; y++) {
    bool skip = false;
    if (y <= y_end) {
      if (y >= 22 && y <= 29 && x >= 53 && x <= 106)
        skip = true;
      if (y >= 72 && y <= 79 && x >= 47 && x <= 112)
        skip = true;
      if (y >= 36 && y <= 63 && x >= num_sx && x <= num_ex)
        skip = true;
    } else {
      skip = true;
    }

    if (!skip) {
      if (run_start == -1)
        run_start = y;
    } else {
      if (run_start != -1) {
        tft_fill_rect(x, run_start, 1, y - run_start, color);
        run_start = -1;
      }
    }
  }
}

static void draw_bottom_wave(uint16_t active_color, uint8_t wt,
                             int current_aqi) {
  uint16_t wave_col = get_pastel(active_color);
  uint16_t bg = 0xFFFF;

  int char_w = 24;
  int len = current_aqi >= 100 ? 3 : (current_aqi >= 10 ? 2 : 1);
  int num_sx = 80 - ((len * char_w) / 2);
  int num_ex = num_sx + (len * char_w) - 1;

  int fill_h = (current_aqi * 84) / 500;
  if (fill_h > 84)
    fill_h = 84;
  int base_wy = 96 - fill_h;

  for (int dx = 0; dx <= 42; dx++) {
    int c_bot = 54 + chalf[dx] - 1;
    int c_top = 54 - chalf[dx] + 1;

    int lx = 80 - dx;
    int l_wy = base_wy + swave[(lx + wt) % 50];
    if (l_wy < c_top)
      l_wy = c_top;
    if (l_wy > c_bot)
      l_wy = c_bot;

    if (l_wy > c_top)
      draw_clipped_column(lx, c_top, l_wy - 1, bg, num_sx, num_ex);
    if (c_bot >= l_wy)
      draw_clipped_column(lx, l_wy, c_bot, wave_col, num_sx, num_ex);

    if (dx > 0) {
      int rx = 80 + dx;
      int r_wy = base_wy + swave[(rx + wt) % 50];
      if (r_wy < c_top)
        r_wy = c_top;
      if (r_wy > c_bot)
        r_wy = c_bot;

      if (r_wy > c_top)
        draw_clipped_column(rx, c_top, r_wy - 1, bg, num_sx, num_ex);
      if (c_bot >= r_wy)
        draw_clipped_column(rx, r_wy, c_bot, wave_col, num_sx, num_ex);
    }
  }
}

// ============================================================
// PAGE 1 â€“ Logo Splash
// Renders the exact bitmap from logo.h (bannari_logo, 160x128)
// which contains the pre-rendered Microchip + Bannari Amman logos.
// Holds for 5 seconds then fades out with a topâ†’bottom white wipe.
// ============================================================
static void draw_logo_page(void) {
  // ---- Display the exact combined logo bitmap (full screen 160x128) ----
  tft_draw_bitmap(0, 0, LOGO_W, LOGO_H, bannari_logo);

  // ---- Hold for 5 seconds ----
  delay_ms(5000);

  // ---- Slow FADE-OUT: top-to-bottom white curtain wipe (~900 ms) ----
  // 32 bands Ã— 4 px each Ã— 28 ms delay = ~896 ms
  for (uint8_t y = 0; y < 128; y += 4) {
    tft_fill_rect(0, y, 160, 4, 0xFFFF); // overwrite strip with white
    delay_ms(28);
  }
  // Final safety clear
  tft_fill_screen(0xFFFF);
}

// ============================================================
// Helper: read and compute all 5 sensor values into floats
// ============================================================
static void read_all_sensors(float *ppm_o3, float *ppm_co, float *ppm_nh3,
                             float *pm25, float *co2) {
  float R0_MQ131 = 20000.0f;
  float R0_MQ7 = 20000.0f;
  float R0_MQ135 = 20000.0f;

  float v1 = adc_to_voltage(read_adc_avg(3));
  *ppm_o3 = pow(10, ((log10(calculate_Rs(v1) / R0_MQ131) - 0.8f) / -0.7f));
  if (*ppm_o3 > 9999.0f)
    *ppm_o3 = 9999.0f;
  if (*ppm_o3 < 0.0f)
    *ppm_o3 = 0.0f;

  float v2 = adc_to_voltage(read_adc_avg(4));
  *ppm_co = pow(10, ((log10(calculate_Rs(v2) / R0_MQ7) - 0.77f) / -0.47f));
  if (*ppm_co > 9999.0f)
    *ppm_co = 9999.0f;
  if (*ppm_co < 0.0f)
    *ppm_co = 0.0f;

  float v3 = adc_to_voltage(read_adc_avg(5));
  *ppm_nh3 = pow(10, ((log10(calculate_Rs(v3) / R0_MQ135) - 0.42f) / -0.48f));
  if (*ppm_nh3 > 9999.0f)
    *ppm_nh3 = 9999.0f;
  if (*ppm_nh3 < 0.0f)
    *ppm_nh3 = 0.0f;

  *pm25 = read_dust();
  *co2 = read_mg811_co2();
}

// ============================================================
// AQI EPA breakpoint linear interpolation
// Returns the AQI sub-index for a given pollutant concentration.
// ============================================================
static int aqi_linear(int lo, int hi, float clo, float chi, float c) {
  return (int)(((float)(hi - lo) / (chi - clo)) * (c - clo) + (float)lo);
}

static int aqi_from_pm25(float c) {
  if (c < 0.0f)
    c = 0.0f;
  if (c <= 12.0f)
    return aqi_linear(0, 50, 0.0f, 12.0f, c);
  if (c <= 35.4f)
    return aqi_linear(51, 100, 12.1f, 35.4f, c);
  if (c <= 55.4f)
    return aqi_linear(101, 150, 35.5f, 55.4f, c);
  if (c <= 150.4f)
    return aqi_linear(151, 200, 55.5f, 150.4f, c);
  if (c <= 250.4f)
    return aqi_linear(201, 300, 150.5f, 250.4f, c);
  if (c <= 500.4f)
    return aqi_linear(301, 500, 250.5f, 500.4f, c);
  return 500;
}

static int aqi_from_co(float c) { // c in ppm
  if (c < 0.0f)
    c = 0.0f;
  if (c <= 4.4f)
    return aqi_linear(0, 50, 0.0f, 4.4f, c);
  if (c <= 9.4f)
    return aqi_linear(51, 100, 4.5f, 9.4f, c);
  if (c <= 12.4f)
    return aqi_linear(101, 150, 9.5f, 12.4f, c);
  if (c <= 15.4f)
    return aqi_linear(151, 200, 12.5f, 15.4f, c);
  if (c <= 30.4f)
    return aqi_linear(201, 300, 15.5f, 30.4f, c);
  if (c <= 50.4f)
    return aqi_linear(301, 400, 30.5f, 50.4f, c);
  return 500;
}

static int aqi_from_o3(float c) { // c in ppb (1-hr standard)
  if (c < 0.0f)
    c = 0.0f;
  if (c <= 54.0f)
    return aqi_linear(0, 50, 0.0f, 54.0f, c);
  if (c <= 70.0f)
    return aqi_linear(51, 100, 55.0f, 70.0f, c);
  if (c <= 85.0f)
    return aqi_linear(101, 150, 71.0f, 85.0f, c);
  if (c <= 105.0f)
    return aqi_linear(151, 200, 86.0f, 105.0f, c);
  if (c <= 200.0f)
    return aqi_linear(201, 300, 106.0f, 200.0f, c);
  return 301;
}

// Calculates combined AQI using US EPA method: max of all sub-indices.
static int calculate_real_aqi(void) {
  float o3, co, nh3, dust, co2;
  read_all_sensors(&o3, &co, &nh3, &dust, &co2);
  int a_pm = aqi_from_pm25(dust);
  int a_co = aqi_from_co(co);
  int a_o3 = aqi_from_o3(o3);
  // Take the worst (highest) sub-index as overall AQI
  int aqi = a_pm;
  if (a_co > aqi)
    aqi = a_co;
  if (a_o3 > aqi)
    aqi = a_o3;
  if (aqi > 500)
    aqi = 500;
  if (aqi < 0)
    aqi = 0;
  return aqi;
}

// ============================================================
// PAGE 2 â€“ AQI Default display
// Shows wave gauge, AQI number, live location, status label.
// Returns when PA17 goes HIGH (switch to sensor page).
// ============================================================
static void run_aqi_page(int *saved_aqi, int *saved_target) {
  uint16_t bg = 0xFFFF;
  tft_fill_screen(bg);

  uint16_t txt_faint = 0x0000;
  tft_draw_string(35, 2, "LIVE.COIMBATORE", txt_faint, bg, 1);
  tft_draw_string(47, 72, "US EPA \STD.", txt_faint, bg, 1);

  int current_aqi = *saved_aqi;
  int target_aqi = *saved_target;
  uint16_t last_color = 0x0000;
  uint8_t wave_t = 0;
  while (1) {
    if (check_button_toggle()) {
      *saved_aqi = current_aqi;
      *saved_target = target_aqi;
      return;
    }

    uint16_t active_color = get_aqi_color(current_aqi);

    draw_bottom_wave(active_color, wave_t, current_aqi);
    wave_t = (wave_t + 1) % 50;

    // Smoothly animate current_aqi toward target_aqi
    if (current_aqi < target_aqi) {
      current_aqi += 2;
      if (current_aqi > target_aqi)
        current_aqi = target_aqi;
    } else if (current_aqi > target_aqi) {
      current_aqi -= 2;
      if (current_aqi < target_aqi)
        current_aqi = target_aqi;
    }
    active_color = get_aqi_color(current_aqi);

    if (active_color != last_color) {
      last_color = active_color;
      tft_draw_circle_outline(25, 5, 5, active_color);
      tft_fill_circle(25, 5, 3, active_color);
      tft_draw_circle_outline(80, 54, 42, active_color);
      tft_draw_circle_outline(80, 54, 43, active_color);
    }

    // AQI number string
    char buf[4];
    if (current_aqi >= 100) {
      buf[0] = '0' + (current_aqi / 100);
      buf[1] = '0' + ((current_aqi / 10) % 10);
      buf[2] = '0' + (current_aqi % 10);
      buf[3] = '\0';
    } else if (current_aqi >= 10) {
      buf[0] = '0' + ((current_aqi / 10) % 10);
      buf[1] = '0' + (current_aqi % 10);
      buf[2] = '\0';
    } else {
      buf[0] = '0' + (current_aqi % 10);
      buf[1] = '\0';
    }
    tft_fill_rect(20, 36, 120, 28, bg);
    int cw = 24;
    int len = current_aqi >= 100 ? 3 : (current_aqi >= 10 ? 2 : 1);
    tft_draw_string(80 - ((len * cw) / 2), 36, buf, active_color, bg, 4);

    // Gauge bar
    int fill_w = (current_aqi * 100) / 500;
    if (fill_w > 100)
      fill_w = 100;
    uint16_t pastel_bar = get_pastel(active_color);
    tft_fill_rect(30, 122, 100, 2, pastel_bar);
    tft_fill_rect(30, 122, fill_w, 2, active_color);

    // Percentage text
    char pct[5];
    int p_val = (current_aqi * 100) / 500;
    if (p_val >= 100) {
      pct[0] = '1';
      pct[1] = '0';
      pct[2] = '0';
      pct[3] = '%';
      pct[4] = '\0';
    } else if (p_val >= 10) {
      pct[0] = '0' + (p_val / 10);
      pct[1] = '0' + (p_val % 10);
      pct[2] = '%';
      pct[3] = '\0';
    } else {
      pct[0] = '0' + p_val;
      pct[1] = '%';
      pct[2] = '\0';
    }
    tft_fill_rect(134, 119, 24, 8, bg);
    tft_draw_string(134, 119, pct, 0x0000, bg, 1);

    // Status label
    {
      const char *label = get_aqi_label(current_aqi);
      int w = 0;
      while (label[w] != '\0')
        w++;
      tft_fill_rect(0, 102, 160, 16, bg);
      tft_draw_string(80 - (w * 12) / 2, 102, label, active_color, bg, 2);
    }

    // ---- Read sensors every loop tick – continuous serial + AQI update ----
    {
      float o3, co, nh3, dust, co2;
      read_all_sensors(&o3, &co, &nh3, &dust, &co2);

      // Compute real AQI from the same readings (no second ADC read)
      int a_pm = aqi_from_pm25(dust);
      int a_co = aqi_from_co(co);
      int a_o3 = aqi_from_o3(o3);
      int new_aqi = a_pm;
      if (a_co > new_aqi)
        new_aqi = a_co;
      if (a_o3 > new_aqi)
        new_aqi = a_o3;
      if (new_aqi > 500)
        new_aqi = 500;
      if (new_aqi < 0)
        new_aqi = 0;
      target_aqi = new_aqi;

      // UART – sent every loop (as fast as ADC sampling allows)
      char tbuf[200];
      sprintf(tbuf, "O3:%.1f CO:%.1f NH3:%.1f Dust:%.1f CO2:%.1f AQI:%d\r\n",
              o3, co, nh3, dust, co2, target_aqi);
      while (SERCOM3_USART_WriteIsBusy())
        ;
      SERCOM3_USART_Write((uint8_t *)tbuf, strlen(tbuf));

      // ---- WiFi MQTT publish (throttled — every WIFI_PUBLISH_EVERY loops) ----
      static uint32_t wifi_aqi_tick = 0;
      if (++wifi_aqi_tick >= WIFI_PUBLISH_EVERY)
      {
        wifi_aqi_tick = 0;
        /* Reuse tbuf but strip the trailing \r\n for the MQTT payload */
        char mqtt_buf[200];
        sprintf(mqtt_buf, "O3:%.1f CO:%.1f NH3:%.1f Dust:%.1f CO2:%.1f AQI:%d",
                o3, co, nh3, dust, co2, target_aqi);
        wifi_mqtt_publish(mqtt_buf);   /* non-fatal if WiFi is not ready */
      }
    }
  }
}

// ============================================================
// PAGE 3 – Sensor Detail Screen (Dashboard Grid)
// ============================================================

static void draw_static_card(int x, int y, const char *label) {
  uint16_t card_bg = 0xFFFF;   // White
  uint16_t txt_fg = 0x8410;    // Medium Gray
  uint16_t screen_bg = 0x4516; // Teal/Cyan Main Background

  tft_fill_rect(x, y, 74, 38, card_bg);
  // Fake rounded corners
  tft_draw_pixel(x, y, screen_bg);
  tft_draw_pixel(x + 1, y, screen_bg);
  tft_draw_pixel(x, y + 1, screen_bg);
  tft_draw_pixel(x + 73, y, screen_bg);
  tft_draw_pixel(x + 72, y, screen_bg);
  tft_draw_pixel(x + 73, y + 1, screen_bg);
  tft_draw_pixel(x, y + 37, screen_bg);
  tft_draw_pixel(x + 1, y + 37, screen_bg);
  tft_draw_pixel(x, y + 36, screen_bg);
  tft_draw_pixel(x + 73, y + 37, screen_bg);
  tft_draw_pixel(x + 72, y + 37, screen_bg);
  tft_draw_pixel(x + 73, y + 36, screen_bg);

  // Label centered at bottom
  int lbl_len = strlen(label);
  tft_draw_string(x + (74 - (lbl_len * 6)) / 2, y + 26, label, txt_fg, card_bg,
                  1);
}

static void update_card_value(int x, int y, const char *val, const char *unit) {
  uint16_t card_bg = 0xFFFF;
  uint16_t val_fg = 0x0000;  // Black
  uint16_t unit_fg = 0x4208; // Dark Gray

  // Clear the top half
  tft_fill_rect(x + 4, y + 4, 66, 18, card_bg);

  int val_len = strlen(val);
  int unit_len = strlen(unit);
  int total_w = (val_len * 12) + (unit_len * 6);

  int cx = x + (74 - total_w) / 2;
  // Fallback if it's too wide
  if (cx < x + 2)
    cx = x + 2;

  // Draw value (scale 2) and unit (scale 1) side by side
  tft_draw_string(cx, y + 6, val, val_fg, card_bg, 2);
  tft_draw_string(cx + (val_len * 12), y + 14, unit, unit_fg, card_bg, 1);
}

// Standard Edge Detection (Toggle Button logic)
static bool check_button_toggle(void) {
  static bool btn_prev = false;
  bool btn_curr = PA17_IS_HIGH();

  if (btn_curr && !btn_prev) {
    delay_ms(50); // debounce
    if (PA17_IS_HIGH()) {
      btn_prev = true;
      return true; // Button was just pressed!
    }
  } else if (!btn_curr) {
    btn_prev = false;
  }
  return false;
}

static void run_sensor_page(void) {
  uint16_t bg = 0x4516; // Teal/Cyan Main Background
  tft_fill_screen(bg);

  // Draw the 6 static cards
  draw_static_card(4, 4, "O3");
  draw_static_card(82, 4, "CO");

  draw_static_card(4, 46, "NH3");
  draw_static_card(82, 46, "Dust");

  draw_static_card(4, 88, "CO2");
  draw_static_card(82, 88, "AQI");

  uint8_t disp_ticks = 0;

  while (1) {
    if (check_button_toggle()) {
      return;
    }

    // ---- Update sensor values continuously ----
    {
      float o3, co, nh3, dust, co2;
      read_all_sensors(&o3, &co, &nh3, &dust, &co2);

      // Compute AQI
      int a_pm = aqi_from_pm25(dust);
      int a_co = aqi_from_co(co);
      int a_o3 = aqi_from_o3(o3);
      int new_aqi = a_pm;
      if (a_co > new_aqi)
        new_aqi = a_co;
      if (a_o3 > new_aqi)
        new_aqi = a_o3;
      if (new_aqi > 500)
        new_aqi = 500;
      if (new_aqi < 0)
        new_aqi = 0;

      // Update the TFT at a slower 10-15Hz rate to prevent screen
      // tearing/flicker
      if (++disp_ticks >= 5) {
        disp_ticks = 0;
        char vbuf[16];

        sprintf(vbuf, "%.1f", o3);
        update_card_value(4, 4, vbuf, "ppb");

        sprintf(vbuf, "%.1f", co);
        update_card_value(82, 4, vbuf, "ppm");

        sprintf(vbuf, "%.1f", nh3);
        update_card_value(4, 46, vbuf, "ppm");

        sprintf(vbuf, "%.1f", dust);
        update_card_value(82, 46, vbuf, "ug/m3");

        sprintf(vbuf, "%.1f", co2);
        update_card_value(4, 88, vbuf, "ppm");

        sprintf(vbuf, "%d", new_aqi);
        update_card_value(82, 88, vbuf, "idx");
      }

      // UART mirror (Continuous)
      char tbuf[160];
      sprintf(tbuf, "O3:%.1f CO:%.1f NH3:%.1f Dust:%.1f CO2:%.1f AQI:%d\r\n",
              o3, co, nh3, dust, co2, new_aqi);
      while (SERCOM3_USART_WriteIsBusy())
        ;
      SERCOM3_USART_Write((uint8_t *)tbuf, strlen(tbuf));

      // ---- WiFi MQTT publish (throttled — every WIFI_PUBLISH_EVERY loops) ----
      // Payload is JSON so backend handler.js JSON.parse() succeeds.
      // Fields match what handler.js reads: pm25, co2, o3, co, nh3
      static uint32_t wifi_sensor_tick = 0;
      if (++wifi_sensor_tick >= WIFI_PUBLISH_EVERY)
      {
        wifi_sensor_tick = 0;
        char mqtt_buf[160];
        sprintf(mqtt_buf,
                "{\"o3\":%.2f,\"co\":%.2f,\"nh3\":%.2f,\"pm25\":%.2f,\"co2\":%.2f}",
                o3, co, nh3, dust, co2);
        wifi_mqtt_publish(mqtt_buf);
      }
    }
  }
}

// ============================================================
// Top-level display controller – orchestrates all 3 pages
// ============================================================
static void run_display(void) {
  // Page 1 – Logo splash (one-shot, untouched)
  draw_logo_page();

  int count = 0; // State variable: 0 = AQI, 1 = Sensor
  int aqi_cur = 0;
  int aqi_target = 42;

  while (1) {
    if (count == 0) {
      run_aqi_page(&aqi_cur, &aqi_target);
      count = 1;
    } else if (count == 1) {
      run_sensor_page();
      count++;
    }

    if (count >= 2) {
      count = 0; // Reset the count
    }
  }
}

// ======================== MAIN ==============================
int main(void) {
  SYS_Initialize(NULL); // SERCOM3_USART_Initialize() is called inside here

  // Call manual ADC initialization to completely bypass TrustZone blocks
  // that MCC library might introduce.
  adc_init();

  // Startup verification
  char hello[] = "HELLO WORLD INIT\r\n";
  while (SERCOM3_USART_WriteIsBusy())
    ;
  SERCOM3_USART_Write((uint8_t *)hello, strlen(hello));

  // ---- WiFi + MQTT init (ESP-01S on SERCOM5, PB16/PB17) ----
  // wifi_mqtt_init() blocks while joining WiFi (~10-20 s).
  // If the module is absent or credentials are wrong, it returns false
  // and the rest of the firmware continues normally without WiFi.
  wifi_mqtt_init();

  // PA03 is the Dust LED. Set it to OUTPUT.
  PIN_OUTPUT_ENABLE(3);

  PIN_OUTPUT_ENABLE(8);
  PIN_OUTPUT_ENABLE(9);
  PIN_OUTPUT_ENABLE(10);
  PIN_OUTPUT_ENABLE(11);
  PIN_OUTPUT_ENABLE(12);

  PORT_SEC_REGS->GROUP[0].PORT_PINCFG[8] = 0x00;
  PORT_SEC_REGS->GROUP[0].PORT_PINCFG[9] = 0x00;
  PORT_SEC_REGS->GROUP[0].PORT_PINCFG[10] = 0x00;
  PORT_SEC_REGS->GROUP[0].PORT_PINCFG[11] = 0x00;
  PORT_SEC_REGS->GROUP[0].PORT_PINCFG[12] = 0x00;
  PORT_REGS->GROUP[0].PORT_PINCFG[8] = 0x00;
  PORT_REGS->GROUP[0].PORT_PINCFG[9] = 0x00;
  PORT_REGS->GROUP[0].PORT_PINCFG[10] = 0x00;
  PORT_REGS->GROUP[0].PORT_PINCFG[11] = 0x00;
  PORT_REGS->GROUP[0].PORT_PINCFG[12] = 0x00;

  TFT_SCK_HIGH();
  TFT_MOSI_LOW();
  TFT_CS_HIGH();
  TFT_DC_HIGH();
  TFT_RST_HIGH();

  delay_ms(200);
  tft_init();

  // Configure PA17 as digital INPUT with internal PULL-DOWN
  // INEN (Input Enable) MUST be set to read the pin state!
  // PINCFG 0x06 -> INEN=1 (bit 1) and PULLEN=1 (bit 2)
  PORT_SEC_REGS->GROUP[0].PORT_DIRCLR = (1U << 17U); // direction = input
  PORT_REGS->GROUP[0].PORT_DIRCLR = (1U << 17U);

  PORT_SEC_REGS->GROUP[0].PORT_OUTCLR = (1U << 17U); // output=0 means Pull-Down
  PORT_REGS->GROUP[0].PORT_OUTCLR = (1U << 17U);     // when PULLEN is 1.

  PORT_SEC_REGS->GROUP[0].PORT_PINCFG[17] = 0x06U; // INEN and PULLEN enabled
  PORT_REGS->GROUP[0].PORT_PINCFG[17] = 0x06U;

  run_display();

  return 0;
}
