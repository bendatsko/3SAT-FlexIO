#include <Arduino.h>
#include <DMAChannel.h>
// Referencing:
// https://forum.pjrc.com/index.php?threads/teensy-4-1-how-to-start-using-flexio.66201/

#define FLEXIO2_SHIFTCTL4 (IMXRT_FLEXIO2.offset090)
#define FLEXIO2_SHIFTCTL5 (IMXRT_FLEXIO2.offset094)
#define FLEXIO2_SHIFTCTL6 (IMXRT_FLEXIO2.offset098)
#define FLEXIO2_SHIFTCTL7 (IMXRT_FLEXIO2.offset09C)
#define FLEXIO2_SHIFTCFG4 (IMXRT_FLEXIO2.offset110)
#define FLEXIO2_SHIFTCFG5 (IMXRT_FLEXIO2.offset114)
#define FLEXIO2_SHIFTCFG6 (IMXRT_FLEXIO2.offset118)
#define FLEXIO2_SHIFTCFG7 (IMXRT_FLEXIO2.offset11C)

DMAChannel dmaChannel;

unsigned long dmaStartTime;
unsigned long dmaEndTime;

unsigned long prevTime;
unsigned long currTime;

uint32_t lastSeenHalf = 0;

#define DMABUFFER_SIZE 4096

// data written by the DMA
uint32_t dmaBuffer[DMABUFFER_SIZE];
uint32_t dmaBufferHalfCount = 0;

// deinterleaved data
uint32_t processedBuffer[DMABUFFER_SIZE * 2];

// data consistency check
uint32_t prevVal = 0;
bool dataCorrect = true;

void xbar_connect(unsigned int input, unsigned int output) {
  if (input >= 88)
    return;
  if (output >= 132)
    return;

  volatile uint16_t *xbar = &XBARA1_SEL0 + (output / 2);
  uint16_t val = *xbar;
  if (!(output & 1)) {
    val = (val & 0xFF00) | input;
  } else {
    val = (val & 0x00FF) | (input << 8);
  }
  *xbar = val;
}

void inputDMAInterrupt() {
  dataCorrect = true;

  prevTime = currTime;
  currTime = micros();

  dmaStartTime = micros();

  uint32_t *dmaData =
      dmaBuffer + (DMABUFFER_SIZE / 2) * (dmaBufferHalfCount & 1);
  uint32_t *processedData =
      processedBuffer + (DMABUFFER_SIZE / 2) * (dmaBufferHalfCount & 1) * 2;

  uint32_t inData[] = {dmaData[2], // pins 0-3
                       dmaData[3],
                       dmaData[6], // pins 16-19
                       dmaData[7],
                       dmaData[0],  // pin 10
                       dmaData[1],  // pin 11
                       dmaData[4],  // pin 28
                       dmaData[5]}; // pin 29

  for (int batch = 0; batch < (DMABUFFER_SIZE / 2) / 8; ++batch) {
    for (int i = 0; i < 16; ++i) {
      uint32_t pins_00_03 =
          (((i < 8) ? dmaData[2] : dmaData[3]) >> ((i & 0x07) * 4)) & 0x0F;
      uint32_t pins_16_19 =
          (((i < 8) ? dmaData[6] : dmaData[7]) >> ((i & 0x07) * 4)) & 0x0F;
      uint32_t pin_10 = (dmaData[0] >> (16 + i)) & 1;
      uint32_t pin_11 = (dmaData[1] >> (16 + i)) & 1;
      uint32_t pin_28 = (dmaData[4] >> (16 + i)) & 1;
      uint32_t pin_29 = (dmaData[5] >> (16 + i)) & 1;

      uint32_t outData = (pins_00_03) | (pins_16_19 << 4) | (pin_10 << 8) |
                         (pin_11 << 9) | (pin_28 << 10) | (pin_29 << 11);

      processedData[i] = outData;

      if (((prevVal + 1) & 4095) != outData) {
        dataCorrect = false;
      }
      prevVal = outData;
    }

    dmaData += 8;
    processedData += 16;
  }

  dmaEndTime = micros();

  ++dmaBufferHalfCount;

  dmaChannel.clearInterrupt(); // tell system we processed it.
  asm("DSB");                  // this is a memory barrier
}

void setupFlexIOInput() {
  // FlexIO2 works at 30Mhz by default, we need it faster!

  // set the FlexIO2 clock divider to 2 instead of 8
  CCM_CS1CDR &= ~(CCM_CS1CDR_FLEXIO2_CLK_PODF(7));
  CCM_CS1CDR |= CCM_CS1CDR_FLEXIO2_CLK_PODF(1);

  // CCM_CS1CDR |= CCM_CS1CDR_FLEXIO2_CLK_PODF( 0 );	// even faster seems to
  // work ;-)

  // enable clock for FlexIO2
  CCM_CCGR3 |= CCM_CCGR3_FLEXIO2(CCM_CCGR_ON);

  // enable clock for clock XBAR
  CCM_CCGR2 |= CCM_CCGR2_XBAR1(CCM_CCGR_ON);

  // enable FlexIO2
  FLEXIO2_CTRL |= 1;

  // fast mode -if it's on, the 0/1 DMA requests do not work
  // FLEXIO2_CTRL |= 1 << 2;

  ///////////////////////////////////////////////
  // set the pads corresponding to the FlexIO2 pins 0-3, 10, 11, 28, 29, 16-19
  // to proper mode

  IOMUXC_SW_MUX_CTL_PAD_GPIO_B0_00 = 4; // 0
  IOMUXC_SW_MUX_CTL_PAD_GPIO_B0_01 = 4; // 1
  IOMUXC_SW_MUX_CTL_PAD_GPIO_B0_02 = 4; // 2
  IOMUXC_SW_MUX_CTL_PAD_GPIO_B0_03 = 4; // 3
  IOMUXC_SW_MUX_CTL_PAD_GPIO_B0_10 = 4; // 10
  IOMUXC_SW_MUX_CTL_PAD_GPIO_B0_11 = 4; // 11

  IOMUXC_SW_MUX_CTL_PAD_GPIO_B1_00 = 4; // 16
  IOMUXC_SW_MUX_CTL_PAD_GPIO_B1_01 = 4; // 17
  IOMUXC_SW_MUX_CTL_PAD_GPIO_B1_02 = 4; // 18
  IOMUXC_SW_MUX_CTL_PAD_GPIO_B1_03 = 4; // 19
  IOMUXC_SW_MUX_CTL_PAD_GPIO_B1_12 = 4; // 28
  IOMUXC_SW_MUX_CTL_PAD_GPIO_B1_13 = 4; // 29

  // set the mode for the clock pin
  IOMUXC_SW_MUX_CTL_PAD_GPIO_B0_12 = 4; // 12

  //////////////////////////////////////////////
  // setup shifters and timer

  //		0 - single bits from pin 10
  FLEXIO2_SHIFTCTL0 =
      FLEXIO_SHIFTCTL_TIMSEL(0) | // timer 0
                                  // FLEXIO_SHIFTCTL_TIMPOL |
                                  // // on positive edge
      FLEXIO_SHIFTCTL_PINCFG(0) |  // pin output disabled
      FLEXIO_SHIFTCTL_PINSEL(10) | // pin 0
      // FLEXIO_SHIFTCTL_PINPOL			|			//
      // active high
      FLEXIO_SHIFTCTL_SMOD(1); // receive mode

  FLEXIO2_SHIFTCFG0 =
      FLEXIO_SHIFTCFG_PWIDTH(0) | // single bit
                                  // FLEXIO_SHIFTCFG_INSRC |
                                  // // from pin
      FLEXIO_SHIFTCFG_SSTOP(0) | // stop bit disabled
      FLEXIO_SHIFTCFG_SSTART(0); // start bit disabled

  ////		1 - single bits from pin 11
  FLEXIO2_SHIFTCTL1 = FLEXIO_SHIFTCTL_TIMSEL(0) | FLEXIO_SHIFTCTL_PINCFG(0) |
                      FLEXIO_SHIFTCTL_PINSEL(11) | FLEXIO_SHIFTCTL_SMOD(1);
  FLEXIO2_SHIFTCFG1 = FLEXIO_SHIFTCFG_PWIDTH(0) | FLEXIO_SHIFTCFG_SSTOP(0) |
                      FLEXIO_SHIFTCFG_SSTART(0);

  //		2 - 4 bits from shifter 3
  FLEXIO2_SHIFTCTL2 = FLEXIO_SHIFTCTL_TIMSEL(0) | FLEXIO_SHIFTCTL_PINCFG(0) |
                      FLEXIO_SHIFTCTL_PINSEL(0) | FLEXIO_SHIFTCTL_SMOD(1);
  FLEXIO2_SHIFTCFG2 = FLEXIO_SHIFTCFG_PWIDTH(3) | FLEXIO_SHIFTCFG_INSRC |
                      FLEXIO_SHIFTCFG_SSTOP(0) | FLEXIO_SHIFTCFG_SSTART(0);

  //		3 - 4 bits from pins 0-3
  FLEXIO2_SHIFTCTL3 = FLEXIO_SHIFTCTL_TIMSEL(0) | FLEXIO_SHIFTCTL_PINCFG(0) |
                      FLEXIO_SHIFTCTL_PINSEL(0) | FLEXIO_SHIFTCTL_SMOD(1);
  FLEXIO2_SHIFTCFG3 = FLEXIO_SHIFTCFG_PWIDTH(3) | FLEXIO_SHIFTCFG_SSTOP(0) |
                      FLEXIO_SHIFTCFG_SSTART(0);

  //		4 - single bit from pin 28
  FLEXIO2_SHIFTCTL4 = FLEXIO_SHIFTCTL_TIMSEL(0) | FLEXIO_SHIFTCTL_PINCFG(0) |
                      FLEXIO_SHIFTCTL_PINSEL(28) | FLEXIO_SHIFTCTL_SMOD(1);
  FLEXIO2_SHIFTCFG4 = FLEXIO_SHIFTCFG_PWIDTH(0) | FLEXIO_SHIFTCFG_SSTOP(0) |
                      FLEXIO_SHIFTCFG_SSTART(0);

  //		5 - single bit from pin 29
  FLEXIO2_SHIFTCTL5 = FLEXIO_SHIFTCTL_TIMSEL(0) | FLEXIO_SHIFTCTL_PINCFG(0) |
                      FLEXIO_SHIFTCTL_PINSEL(29) | FLEXIO_SHIFTCTL_SMOD(1);
  FLEXIO2_SHIFTCFG5 = FLEXIO_SHIFTCFG_PWIDTH(0) | FLEXIO_SHIFTCFG_SSTOP(0) |
                      FLEXIO_SHIFTCFG_SSTART(0);

  //		6 - 4 bits from shifter 7
  FLEXIO2_SHIFTCTL6 = FLEXIO_SHIFTCTL_TIMSEL(0) | FLEXIO_SHIFTCTL_PINCFG(0) |
                      FLEXIO_SHIFTCTL_PINSEL(0) | FLEXIO_SHIFTCTL_SMOD(1);
  FLEXIO2_SHIFTCFG6 = FLEXIO_SHIFTCFG_PWIDTH(3) | FLEXIO_SHIFTCFG_INSRC |
                      FLEXIO_SHIFTCFG_SSTOP(0) | FLEXIO_SHIFTCFG_SSTART(0);

  //		7 - 4 bits from pins 16-19
  FLEXIO2_SHIFTCTL7 = FLEXIO_SHIFTCTL_TIMSEL(0) | FLEXIO_SHIFTCTL_PINCFG(0) |
                      FLEXIO_SHIFTCTL_PINSEL(16) | FLEXIO_SHIFTCTL_SMOD(1);
  FLEXIO2_SHIFTCFG7 = FLEXIO_SHIFTCFG_PWIDTH(3) | FLEXIO_SHIFTCFG_SSTOP(0) |
                      FLEXIO_SHIFTCFG_SSTART(0);

  // timer 0 - clocked from pin 12, enabled by an external trigger rise,
  // disabled by the external trigger fall
  FLEXIO2_TIMCTL0 =
      FLEXIO_TIMCTL_TRGSEL(0) | // src trigger 0
                                // FLEXIO_TIMCTL_TRGPOL		|
                                // // trigger active high FLEXIO_TIMCTL_TRGSRC
                                // |			// exeternal trigger
      FLEXIO_TIMCTL_PINCFG(0) |  // timer pin output disabled
      FLEXIO_TIMCTL_PINSEL(12) | // timer pin 12
      // FLEXIO_TIMCTL_PINPOL		|			// timer pin
      // active high
      FLEXIO_TIMCTL_TIMOD(3); // timer mode 16bit

  FLEXIO2_TIMCFG0 =
      FLEXIO_TIMCFG_TIMOUT(0) | // timer output = low, not affcted by reset
      FLEXIO_TIMCFG_TIMDEC(
          2) | // decrement on pin input (both edges), shift clock = pin input
      FLEXIO_TIMCFG_TIMRST(6) | // timer reset on trigger rising (this resets
                                // the timer when line valid becomes asserted)
      FLEXIO_TIMCFG_TIMDIS(6) | // disable timer on trigger falling
      FLEXIO_TIMCFG_TIMENA(6) | // enable timer on trigger rising
      FLEXIO_TIMCFG_TSTOP(0);   // stop bit disabled
                                // FLEXIO_TIMCFG_TSTART					// start bit
  // disabled

  FLEXIO2_TIMCMP0 = 31; // move from shift to shiftbuf every 32 timer ticks (so
                        // 16 shift clock cycles)

  /////////////////////////////////////////////////////////
  // setup external trigger (line valid signal)

  // set the IOMUX mode to 3, to route it to XBAR
  IOMUXC_SW_MUX_CTL_PAD_GPIO_EMC_06 = 3;

  // set XBAR1_IO008 to INPUT
  IOMUXC_GPR_GPR6 &= ~(IOMUXC_GPR_GPR6_IOMUXC_XBAR_DIR_SEL_8);

  // daisy chaining - select between EMC06 and SD_B0_04
  IOMUXC_XBAR1_IN08_SELECT_INPUT = 0;

  // connect the IOMUX_XBAR_INOUT08 to FlexIO2 trigger 0
  xbar_connect(XBARA1_IN_IOMUX_XBAR_INOUT08, XBARA1_OUT_FLEXIO2_TRIGGER_IN0);

  ////////////////////////////////////////////////////////
  // setup dma to pick up the data

  // configure DMA channels
  dmaChannel.begin();

  dmaChannel.TCD->SADDR = &FLEXIO2_SHIFTBUF0;
  dmaChannel.TCD->SOFF = 4;
  dmaChannel.TCD->ATTR_SRC = (5 << 3) | 2; // 32 bit reads + 2^5 modulo
  dmaChannel.TCD->SLAST = 0;

  dmaChannel.TCD->DADDR = dmaBuffer;
  dmaChannel.TCD->DOFF = 4;
  dmaChannel.TCD->ATTR_DST = 2; // 32 bit writes
  dmaChannel.TCD->DLASTSGA = -DMABUFFER_SIZE * 4;

  dmaChannel.TCD->NBYTES = 8 * 4; // write 32 bytes - all the shiftbuf registers
  dmaChannel.TCD->BITER = DMABUFFER_SIZE / 8;
  dmaChannel.TCD->CITER = DMABUFFER_SIZE / 8;

  dmaChannel.TCD->CSR &=
      ~(DMA_TCD_CSR_DREQ); // do not disable the channel after it completes - so
                           // it just keeps going
  dmaChannel.TCD->CSR |=
      DMA_TCD_CSR_INTMAJOR |
      DMA_TCD_CSR_INTHALF; // interrupt at completion and at half completion

  dmaChannel.attachInterrupt(inputDMAInterrupt);
  dmaChannel.triggerAtHardwareEvent(DMAMUX_SOURCE_FLEXIO2_REQUEST0);

  // enable DMA on shifter status
  FLEXIO2_SHIFTSDEN |= 1 << 0;

  // enable DMA
  dmaChannel.enable();
}

void setup() {
  Serial.begin(115200);

  setupFlexIOInput();
}

void loop() {
  delay(100);

  if (lastSeenHalf != dmaBufferHalfCount) {
    uint32_t *dmaData =
        processedBuffer + 2 * (DMABUFFER_SIZE / 2) * (dmaBufferHalfCount & 1);

    Serial.printf("%s %8u, %8u, 0x%08X 0x%08X 0x%08X 0x%08X 0x%08X 0x%08X "
                  "0x%08X 0x%08X\n",
                  dataCorrect ? "" : "DATA INCORRECT!", currTime - prevTime,
                  dmaEndTime - dmaStartTime, dmaData[0], dmaData[1], dmaData[2],
                  dmaData[3], dmaData[4], dmaData[5], dmaData[6], dmaData[7]);

    lastSeenHalf = dmaBufferHalfCount;
  } else {
    // Serial.printf("Nothing\n" );
  }
}