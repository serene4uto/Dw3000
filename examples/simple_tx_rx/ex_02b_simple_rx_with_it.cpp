#include <Arduino.h>
#include <SPI.h>

#include "deca_regs.h" 
#include "deca_device_api.h"
#include "deca_probe_interface.h"
#include "port.h"
#include "shared_defines.h"

/* Default communication configuration. We use default non-STS DW mode. */
static dwt_config_t config = {
    5,                /* Channel number. */
    DWT_PLEN_128,     /* Preamble length. Used in TX only. */
    DWT_PAC8,         /* Preamble acquisition chunk size. Used in RX only. */
    9,                /* TX preamble code. Used in TX only. */
    9,                /* RX preamble code. Used in RX only. */
    1,                /* 0 to use standard 8 symbol SFD, 1 to use non-standard 8 symbol, 2 for non-standard 16 symbol SFD and 3 for 4z 8 symbol SDF type */
    DWT_BR_6M8,       /* Data rate. */
    DWT_PHRMODE_STD,  /* PHY header mode. */
    DWT_PHRRATE_STD,  /* PHY header rate. */
    (129 + 8 - 8),    /* SFD timeout (preamble length + 1 + SFD length - PAC size). Used in RX only. */
    DWT_STS_MODE_OFF, /* STS disabled */
    DWT_STS_LEN_64,   /* STS length see allowed values in Enum dwt_sts_lengths_e */
    DWT_PDOA_M0       /* PDOA mode off */
};

/* Buffer to store received frame. See NOTE 1 below. */
static uint8_t rx_buffer[FRAME_LEN_MAX];

/* Hold copy of status register state here for reference so that it can be examined at a debug breakpoint. */
uint32_t status_reg;
/* Hold copy of frame length of frame received (if good) so that it can be examined at a debug breakpoint. */
uint16_t frame_len;

volatile bool rx_ok = false;
volatile bool rx_error = false;

void rx_ok_cb(const dwt_cb_data_t *cb_data)
{
  rx_ok = true;
}

void rx_error_cb(const dwt_cb_data_t *cb_data)
{
  rx_error = true;
}

void setup() {

  Serial.begin(115200);
  while (!Serial);

  SPI.begin();
  pinMode(DW_CS_PIN, OUTPUT);
  digitalWrite(DW_CS_PIN, HIGH);

  init_dw3000_irq(); // Initialize DW3000 IRQ

  reset_DW3000();

  // Initialize DW3000
  if (dwt_probe((struct dwt_probe_s *)&dw3000_probe_interf) != DWT_SUCCESS) {
    Serial.println("Failed to probe DW3000");
    while (1);
  }

  delay(100);

  while (!dwt_checkidlerc()) // Need to make sure DW IC is in IDLE_RC before proceeding
  {
    Serial.println("IDLE FAILED");
    while (1);
  }

  dwt_softreset();
  delay(200);

  if (dwt_initialise(DWT_DW_INIT) == DWT_ERROR)
  {
    Serial.println("INIT FAILED");
    while (1);
  }

  // Enabling LEDs here for debug so that for each TX the D1 LED will flash on DW3000 red eval-shield boards.
  dwt_setleds(DWT_LEDS_ENABLE | DWT_LEDS_INIT_BLINK);

  // Configure DW IC. See NOTE 5 below.
  if (dwt_configure(&config)) // if the dwt_configure returns DWT_ERROR either the PLL or RX calibration has failed the host should reset the device
  {
    Serial.println("CONFIG FAILED");
    while (1);
  }

  // Set event callbacks
  dwt_setcallbacks(
    NULL,
    rx_ok_cb,
    NULL, 
    NULL,
    NULL,
    NULL
  );

  // set IRQ event
  dwt_setinterrupt(
    (SYS_STATUS_RXFCG_BIT_MASK | SYS_STATUS_ALL_RX_ERR), 
    0, 
    DWT_ENABLE_INT_ONLY
  );

  /* NOTE: Enabling Interrupts should be done after the dwt_setcallbacks() and dwt_setinterrupt() calls.
   * Otherwise, the DW IC may trigger unknown interrupt before the application is ready to handle it.
   * It can cause the crash of the application.
   */
  enable_dw3000_irq();
}

void loop()
{
  /* TESTING BREAKPOINT LOCATION #1 */

  /* Clear local RX buffer to avoid having leftovers from previous receptions  This is not necessary but is included here to aid reading
   * the RX buffer.
   * This is a good place to put a breakpoint. Here (after first time through the loop) the local status register will be set for last event
   * and if a good receive has happened the data buffer will have the data in it, and frame_len will be set to the length of the RX frame. */
  memset(rx_buffer, 0, sizeof(rx_buffer));

  /* Activate reception immediately. See NOTE 2 below. */
  dwt_rxenable(DWT_START_RX_IMMEDIATE);

  while (!rx_ok && !rx_error)
  {
    delay(1);
  }

  if(rx_ok)
  {
    /* A frame has been received, copy it to our local buffer. */
    frame_len = dwt_read32bitreg(RX_FINFO_ID) & RX_FINFO_RXFLEN_BIT_MASK;
    if (frame_len <= FRAME_LEN_MAX)
    {
      dwt_readrxdata(rx_buffer, frame_len - FCS_LEN, 0); /* No need to read the FCS/CRC. */
    }

    /* Clear good RX frame event in the DW IC status register. */
    dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG_BIT_MASK);

    Serial.println("Received frame");
  }
  
  if(rx_error)
  {
    /* Clear RX error events in the DW IC status register. */
    dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_ERR);
  }

  rx_ok = false;
  rx_error = false;

}