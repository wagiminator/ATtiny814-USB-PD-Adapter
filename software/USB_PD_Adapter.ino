// ===================================================================================
// Project:   USB PD Adapter
// Version:   1.0
// Year:      2022
// Author:    Stefan Wagner
// Github:    https://github.com/wagiminator
// EasyEDA:   https://easyeda.com/wagiminator
// License:   http://creativecommons.org/licenses/by-sa/3.0/
// ===================================================================================
//
// Description:
// ------------
// With the USB PD Adapter you can use almost any USB Type-C PD power supply to power
// your projects with different selectable voltages and high currents. Important 
// values such as voltage, current, power and energy are displayed on the OLED.
// The USB PD Adapter is based on the cheap and easy-to-use CH224K multi fast
// charging protocol power receiving chip, the INA219 voltage and current sensor IC,
// and an ATtiny204, 214, 404, 414, 804, 814, 1604 or 1614 microcontroller.
//
// Wiring:
// -------
//                                +-\/-+
//                          Vcc  1|°   |14  GND
//             --- !SS AIN4 PA4  2|    |13  PA3 AIN3 SCK ---- 
//             ------- AIN5 PA5  3|    |12  PA2 AIN2 MISO --- KEY2
// CH224K   PG --- DAC AIN6 PA6  4|    |11  PA1 AIN1 MOSI --- KEY1
// CH224K CFG1 ------- AIN7 PA7  5|    |10  PA0 AIN0 UPDI --- UPDI
// CH224K CFG3 -------- RXD PB3  6|    |9   PB0 AIN11 SCL --- INA219/OLED
// CH224K CFG2 ---------TXD PB2  7|    |8   PB1 AIN10 SDA --- INA219/OLED
//                                +----+
//
// Compilation Settings:
// ---------------------
// Core:    megaTinyCore (https://github.com/SpenceKonde/megaTinyCore)
// Board:   ATtiny1614/1604/814/804/414/404/214/204
// Chip:    choose the chip you have installed
// Clock:   1 MHz internal
//
// Leave the rest on default settings. Don't forget to "Burn bootloader"!
// Compile and upload the code.
//
// No Arduino core functions or libraries are used. To compile and upload without
// Arduino IDE download AVR 8-bit toolchain at:
// https://www.microchip.com/mplab/avr-support/avr-and-arm-toolchains-c-compilers
// and extract to tools/avr-gcc. Use the makefile to compile and upload.
//
// Fuse Settings: 0:0x00 1:0x00 2:0x01 4:0x00 5:0xC5 6:0x04 7:0x00 8:0x00
//
// Operating Instructions:
// -----------------------
// 1. Connect the USB PD Adapter to a USB Type-C PD power supply using a USB-C cable.
// 2. Use the SET button to select the desired output voltage. An hourglass appears 
//    on the display while the device is communicating with the power supply. If 
//    the negotiation was successful, a tick is displayed and the desired voltage 
//    is present at the output.
// 3. Connect the device to the power consumer via the output screw terminal.
// 4. Use the RESET button to clear the energy counter.


// ===================================================================================
// Libraries, Definitions and Macros
// ===================================================================================

// Libraries
#include <avr/io.h>                     // for GPIO
#include <avr/interrupt.h>              // for interrupts
#include <util/delay.h>                 // for delays

// Pin definitions
#define PIN_SCL     PB0                 // I2C SCL, connected to INA219 and OLED
#define PIN_SDA     PB1                 // I2C SDA, connected to INA219 and OLED
#define PIN_CFG1    PA7                 // CFG1 of CH224K
#define PIN_CFG2    PB2                 // CFG2 of CH224K
#define PIN_CFG3    PB3                 // CFG3 of CH224K
#define PIN_PG      PA6                 // Power Good of CH224K
#define PIN_KEY1    PA1                 // Key 1
#define PIN_KEY2    PA2                 // Key 2

// Pin manipulation macros
enum {PA0, PA1, PA2, PA3, PA4, PA5, PA6, PA7, PB0, PB1, PB2, PB3};  // enumerate pin designators
#define pinInput(x)     (&VPORTA.DIR)[((x)&8)>>1] &= ~(1<<((x)&7))  // set pin to INPUT
#define pinOutput(x)    (&VPORTA.DIR)[((x)&8)>>1] |=  (1<<((x)&7))  // set pin to OUTPUT
#define pinLow(x)       (&VPORTA.OUT)[((x)&8)>>1] &= ~(1<<((x)&7))  // set pin to LOW
#define pinHigh(x)      (&VPORTA.OUT)[((x)&8)>>1] |=  (1<<((x)&7))  // set pin to HIGH
#define pinToggle(x)    (&VPORTA.IN )[((x)&8)>>1] |=  (1<<((x)&7))  // TOGGLE pin
#define pinRead(x)      ((&VPORTA.IN)[((x)&8)>>1] &   (1<<((x)&7))) // READ pin
#define pinDisable(x)   (&PORTA.PIN0CTRL)[(((x)&8)<<2)+((x)&7)] |= PORT_ISC_INPUT_DISABLE_gc
#define pinPullup(x)    (&PORTA.PIN0CTRL)[(((x)&8)<<2)+((x)&7)] |= PORT_PULLUPEN_bm

// ===================================================================================
// I2C Master Implementation (Read/Write, Conservative)
// ===================================================================================

#define I2C_FREQ  100000                          // I2C clock frequency in Hz
#define I2C_BAUD  ((F_CPU / I2C_FREQ) - 10) / 2;  // simplified BAUD calculation

// I2C init function
void I2C_init(void) {
  TWI0.MBAUD   = I2C_BAUD;                        // set TWI master BAUD rate
  TWI0.MCTRLA  = TWI_ENABLE_bm;                   // enable TWI master
  TWI0.MSTATUS = TWI_BUSSTATE_IDLE_gc;            // set bus state to idle
}

// I2C start transmission
void I2C_start(uint8_t addr) {
  TWI0.MADDR = addr;                              // start sending address
  while(!(TWI0.MSTATUS&(TWI_WIF_bm|TWI_RIF_bm))); // wait for transfer to complete
}

// I2C restart transmission
void I2C_restart(uint8_t addr) {
  I2C_start(addr);                                // start sending address
}

// I2C stop transmission
void I2C_stop(void) {
  TWI0.MCTRLB = TWI_MCMD_STOP_gc;                 // send stop condition
}

// I2C transmit one data byte to the slave, ignore ACK bit
void I2C_write(uint8_t data) {
  TWI0.MDATA = data;                              // start sending data byte
  while(~TWI0.MSTATUS & TWI_WIF_bm);              // wait for transfer to complete
}

// I2C receive one data byte from slave; ack=0: last byte, ack>0: more bytes to follow
uint8_t I2C_read(uint8_t ack) {
  while(~TWI0.MSTATUS & TWI_RIF_bm);              // wait for transfer to complete
  uint8_t data = TWI0.MDATA;                      // get received data byte
  if(ack) TWI0.MCTRLB = TWI_MCMD_RECVTRANS_gc;    // ACK:  read more bytes
  else    TWI0.MCTRLB = TWI_ACKACT_NACK_gc;       // NACK: this was the last byte
  return data;                                    // return received byte
}

// ===================================================================================
// INA219 Implementation
// ===================================================================================

// INA219 register values
#define INA_ADDR        0x80                      // I2C write address of INA219
#define INA_CONFIG      0b0010011111111111        // INA config register according to datasheet
#define INA_CALIB       4096                      // INA calibration register according to R_SHUNT
#define INA_REG_CONFIG  0x00                      // INA configuration register address
#define INA_REG_CALIB   0x05                      // INA calibration register address
#define INA_REG_SHUNT   0x01                      // INA shunt voltage register address
#define INA_REG_VOLTAGE 0x02                      // INA bus voltage register address
#define INA_REG_POWER   0x03                      // INA power register address
#define INA_REG_CURRENT 0x04                      // INA current register address

// INA219 write a register value
void INA_write(uint8_t reg, uint16_t value) {
  I2C_start(INA_ADDR);                            // start transmission to INA219
  I2C_write(reg);                                 // write register address
  I2C_write(value >> 8);                          // write register content high byte
  I2C_write(value);                               // write register content low  byte
  I2C_stop();                                     // stop transmission
}

// INA219 read a register
uint16_t INA_read(uint8_t reg) {
  uint16_t result;                                // result variable
  I2C_start(INA_ADDR);                            // start transmission to INA219
  I2C_write(reg);                                 // write register address
  I2C_restart(INA_ADDR | 0x01);                   // restart for reading
  result = (uint16_t)(I2C_read(1) << 8) | I2C_read(0);  // read register content
  I2C_stop();                                     // stop transmission
  return result;                                  // return result
}

// INA219 write inital configuration and calibration values
void INA_init(void) {
  INA_write(INA_REG_CONFIG, INA_CONFIG);          // write INA219 configuration
  INA_write(INA_REG_CALIB,  INA_CALIB);           // write INA219 calibration
}

// INA219 read voltage
uint16_t INA_readVoltage(void) {
  return((INA_read(INA_REG_VOLTAGE) >> 1) & 0xFFFC);
}

// INA219 read sensor values
uint16_t INA_readCurrent(void) {
  uint16_t result =  INA_read(INA_REG_CURRENT);   // read current from INA
  if(result > 32767) result = 0;                  // ignore nagtive currents
  return result;                                  // return result
}

// ===================================================================================
// OLED Implementation
// ===================================================================================

// OLED definitions
#define OLED_ADDR       0x78    // OLED write address
#define OLED_CMD_MODE   0x00    // set command mode
#define OLED_DAT_MODE   0x40    // set data mode

// OLED init settings
const uint8_t OLED_INIT_CMD[] = {
  0xA8, 0x1F,                   // set multiplex for 128x32
  0x20, 0x01,                   // set vertical memory addressing mode
  0xDA, 0x02,                   // set COM pins hardware configuration to sequential
  0x8D, 0x14,                   // enable charge pump
  0xAF                          // switch on OLED
};

// OLED 5x16 font
const uint8_t OLED_FONT[] = {
  0x7C, 0x1F, 0x02, 0x20, 0x02, 0x20, 0x02, 0x20, 0x7C, 0x1F, // 0  0
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x7C, 0x1F, // 1  1
  0x00, 0x1F, 0x82, 0x20, 0x82, 0x20, 0x82, 0x20, 0x7C, 0x00, // 2  2
  0x00, 0x00, 0x82, 0x20, 0x82, 0x20, 0x82, 0x20, 0x7C, 0x1F, // 3  3
  0x7C, 0x00, 0x80, 0x00, 0x80, 0x00, 0x80, 0x00, 0x7C, 0x1F, // 4  4
  0x7C, 0x00, 0x82, 0x20, 0x82, 0x20, 0x82, 0x20, 0x00, 0x1F, // 5  5
  0x7C, 0x1F, 0x82, 0x20, 0x82, 0x20, 0x82, 0x20, 0x00, 0x1F, // 6  6
  0x7C, 0x00, 0x02, 0x00, 0x02, 0x00, 0x02, 0x00, 0x7C, 0x1F, // 7  7
  0x7C, 0x1F, 0x82, 0x20, 0x82, 0x20, 0x82, 0x20, 0x7C, 0x1F, // 8  8
  0x7C, 0x00, 0x82, 0x20, 0x82, 0x20, 0x82, 0x20, 0x7C, 0x1F, // 9  9
  0x7C, 0x3F, 0x82, 0x00, 0x82, 0x00, 0x82, 0x00, 0x7C, 0x3F, // A 10
  0x7C, 0x03, 0x00, 0x0C, 0x00, 0x30, 0x00, 0x0C, 0x7C, 0x03, // V 11
  0x7C, 0x1F, 0x00, 0x20, 0x00, 0x3F, 0x00, 0x20, 0x7C, 0x1F, // W 12
  0x7C, 0x3F, 0x80, 0x00, 0x80, 0x00, 0x80, 0x00, 0x00, 0x3F, // h 13
  0x00, 0x3F, 0x80, 0x00, 0x80, 0x3F, 0x80, 0x00, 0x00, 0x3F, // m 14
  0x7C, 0x1F, 0x82, 0x20, 0x82, 0x20, 0x82, 0x20, 0x00, 0x00, // E 15
  0x02, 0x00, 0x02, 0x00, 0x7E, 0x3F, 0x02, 0x00, 0x02, 0x00, // T 16
  0x00, 0x00, 0x30, 0x06, 0x30, 0x06, 0x00, 0x00, 0x00, 0x00, // : 17
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, //   18 SPACE
  0x3E, 0x3E, 0x72, 0x39, 0xE2, 0x3C, 0x72, 0x39, 0x3E, 0x3E, //   19 hourglass
  0x60, 0x00, 0x80, 0x01, 0x00, 0x06, 0x80, 0x01, 0x60, 0x00  //   20 checkmark
};

// Character definitions
#define COLON   17
#define SPACE   18
#define GLASS   19
#define CHECK   20

// BCD conversion array
const uint16_t DIVIDER[] = {1, 10, 100, 1000, 10000};

// OLED init function
void OLED_init(void) {
  I2C_start(OLED_ADDR);                           // start transmission to OLED
  I2C_write(OLED_CMD_MODE);                       // set command mode
  for (uint8_t i = 0; i < sizeof(OLED_INIT_CMD); i++)
    I2C_write(OLED_INIT_CMD[i]);                  // send the command bytes
  I2C_stop();                                     // stop transmission
}

// OLED set the cursor
void OLED_setCursor(uint8_t xpos, uint8_t ypos) {
  I2C_start(OLED_ADDR);                           // start transmission to OLED
  I2C_write(OLED_CMD_MODE);                       // set command mode
  I2C_write(0x22);                                // command for min/max page
  I2C_write(ypos); I2C_write(ypos+1);             // min: ypos; max: ypos+1
  I2C_write(xpos & 0x0F);                         // set low nibble of start column
  I2C_write(0x10 | (xpos >> 4));                  // set high nibble of start column
  I2C_write(0xB0 | (ypos));                       // set start page
  I2C_stop();                                     // stop transmission
}

// OLED clear a line
void OLED_clearLine(uint8_t ypos) {
  OLED_setCursor(0, ypos);                        // set cursor at line start
  I2C_start(OLED_ADDR);                           // start transmission to OLED
  I2C_write(OLED_DAT_MODE);                       // set data mode
  uint8_t i = 0;                                  // count variable
  do {I2C_write(0x00);} while(--i);               // clear upper half
  I2C_stop();                                     // stop transmission
}

// OLED clear screen
void OLED_clearScreen(void) {
  OLED_clearLine(0); OLED_clearLine(2);           // clear both lines
}

// OLED plot a single character
void OLED_plotChar(uint8_t ch) {
  ch = (ch << 1) + (ch << 3);                     // calculate position of character in font array
  I2C_write(0x00); I2C_write(0x00);               // print spacing between characters
  I2C_write(0x00); I2C_write(0x00);
  for(uint8_t i=10; i; i--) I2C_write(OLED_FONT[ch++]); // print character
}

// OLED print a character
void OLED_printChar(uint8_t ch) {
  I2C_start(OLED_ADDR);                           // start transmission to OLED
  I2C_write(OLED_DAT_MODE);                       // set data mode
  OLED_plotChar(ch);                              // plot the character
  I2C_stop();                                     // stop transmission
}

// OLED print a "string"; terminator: 255
void OLED_printStr(const uint8_t* p) {
  I2C_start(OLED_ADDR);                           // start transmission to OLED
  I2C_write(OLED_DAT_MODE);                       // set data mode
  while(*p < 255) OLED_plotChar(*p++);            // plot each character of the string
  I2C_stop();                                     // stop transmission
}

// OLED print value (BCD conversion by substraction method)
void OLED_printVal(uint16_t value) {
  uint8_t digits   = 5;                           // print 5 digits
  uint8_t leadflag = 0;                           // flag for leading spaces
  I2C_start(OLED_ADDR);                           // start transmission to OLED
  I2C_write(OLED_DAT_MODE);                       // set data mode
  while(digits--) {                               // for all digits digits
    uint8_t digitval = 0;                         // start with digit value 0
    uint16_t divider = DIVIDER[digits];           // read current divider
    while(value >= divider) {                     // if current divider fits into the value
      leadflag = 1;                               // end of leading spaces
      digitval++;                                 // increase digit value
      value -= divider;                           // decrease value by divider
    }
    if(!digits)  leadflag++;                      // least digit has to be printed
    if(leadflag) OLED_plotChar(digitval);         // print the digit
    else         OLED_plotChar(SPACE);            // or print leading space
  }
  I2C_stop();                                     // stop transmission
}

// OLED print 8-bit value as 2-digit decimal (BCD conversion by substraction method)
void OLED_printDec(uint8_t value, uint8_t lead) {
  I2C_start(OLED_ADDR);                           // start transmission to OLED
  I2C_write(OLED_DAT_MODE);                       // set data mode
  uint8_t digitval = 0;                           // start with digit value 0
  while(value >= 10) {                            // if current divider fits into the value
    digitval++;                                   // increase digit value
    value -= 10;                                  // decrease value by divider
  }
  if(digitval) OLED_plotChar(digitval);           // print first digit
  else         OLED_plotChar(lead);
  OLED_plotChar(value);                           // print second digit
  I2C_stop();                                     // stop transmission
}

// ===================================================================================
// Millis Counter Implementation for TCB0
// ===================================================================================

volatile uint32_t MIL_counter = 0;                // millis counter variable

// Init millis counter
void MIL_init(void) {
  TCB0.CCMP    = (F_CPU / 1000) - 1;              // set TOP value (period)
  TCB0.CTRLA   = TCB_ENABLE_bm;                   // enable timer/counter
  TCB0.INTCTRL = TCB_CAPT_bm;                     // enable periodic interrupt
}

// Read millis counter
uint32_t MIL_read(void) {
  cli();                                          // disable interrupt for atomic read
  uint32_t result = MIL_counter;                  // read millis counter
  sei();                                          // enable interrupt again
  return result;                                  // return millis counter value
}

// TCB0 interrupt service routine (every millisecond)
ISR(TCB0_INT_vect) {
  TCB0.INTFLAGS = TCB_CAPT_bm;                    // clear interrupt flag
  MIL_counter++;                                  // increase millis counter
}

// ===================================================================================
// CH224K Implementation
// ===================================================================================

// Some variables
enum {SET_5V, SET_9V, SET_12V, SET_15V, SET_20V};
const uint8_t VOLTAGES[] = {5, 9, 12, 15, 20};
uint8_t CH224K_volt = 0;                          // current voltage pointer

// Some macros
#define CH224K_getVolt()  (VOLTAGES[CH224K_volt]) // get voltage
#define CH224K_isGood()   (!pinRead(PIN_PG))      // power good?

// CH224K init
void CH224K_init(void) {
  pinHigh(PIN_CFG1);                              // start with 5V
  pinOutput(PIN_CFG1);                            // CFG pins as output...
  pinOutput(PIN_CFG2);
  pinOutput(PIN_CFG3);
  pinPullup(PIN_PG);                              // pullup for Power Good pin
}

// CH224K set voltage
void CH224K_setVolt(uint8_t volt) {
  CH224K_volt = volt;
  switch(CH224K_volt) {                           // set CFG pins according to voltage
    case SET_5V:  pinHigh(PIN_CFG1); break;
    case SET_9V:  pinLow (PIN_CFG1); pinLow (PIN_CFG2); pinLow (PIN_CFG3); break;
    case SET_12V: pinLow (PIN_CFG1); pinLow (PIN_CFG2); pinHigh(PIN_CFG3); break;
    case SET_15V: pinLow (PIN_CFG1); pinHigh(PIN_CFG2); pinHigh(PIN_CFG3); break;
    case SET_20V: pinLow (PIN_CFG1); pinHigh(PIN_CFG2); pinLow (PIN_CFG3); break;
    default:      break;
  }
}

// CH224K set next voltage
void CH224K_nextVolt(void) {
  if(++CH224K_volt > SET_20V) CH224K_volt = SET_5V; // next voltage
  switch(CH224K_volt) {                             // change pins according to voltage
    case SET_5V:  pinHigh(PIN_CFG1); pinLow(PIN_CFG2); break;
    case SET_9V:  pinLow (PIN_CFG1); break;
    case SET_12V: pinHigh(PIN_CFG3); break;
    case SET_15V: pinHigh(PIN_CFG2); break;
    case SET_20V: pinLow (PIN_CFG3); break;
    default:      break;
  }
}

// ===================================================================================
// Main Function
// ===================================================================================

// Some "strings"
const uint8_t mA[]  = { 14, 10, 255 };                  // "mA"
const uint8_t mV[]  = { 14, 11, 255 };                  // "mV"
const uint8_t mW[]  = { 14, 12, 18, 255 };              // "mW "
const uint8_t Ah[]  = { 10, 13, 18, 255 };              // "Ah "
const uint8_t mAh[] = { 14, 10, 13, 255 };              // "mAh"
const uint8_t Wt[]  = { 12, 18, 18, 255 };              // "W  "
const uint8_t Wh[]  = { 12, 13, 18, 255 };              // "Wh "
const uint8_t mWh[] = { 14, 12, 13, 255 };              // "mWh"
const uint8_t SET[] = {  5, 15, 16, 17, 18, 255 };      // "SET: "
const uint8_t HGL[] = { 11, SPACE, GLASS, SPACE, 255};  // hourglass
const uint8_t CMK[] = { 11, SPACE, CHECK, SPACE, 255};  // checkmark
const uint8_t SEP[] = { SPACE, SPACE, SPACE, 255};      // seperator

// Main function
int main(void) {
  // Setup
  _PROTECTED_WRITE(CLKCTRL.MCLKCTRLB, 7);         // set clock frequency to 1 MHz
  CH224K_init();                                  // init CH224K
  I2C_init();                                     // init I2C
  INA_init();                                     // init INA219
  OLED_init();                                    // init OLED
  MIL_init();                                     // init TCB for millis counter
  sei();                                          // enable interrupts
  pinPullup(PIN_KEY1); pinPullup(PIN_KEY2);       // pullup for keys
  OLED_clearScreen();                             // clear OLED

  // Local variables
  uint16_t volt, curr;                            // voltage in mV, current in mA
  uint32_t power;                                 // power in mW
  uint32_t energy = 0, charge = 0;                // counter for energy and charge
  uint32_t interval, nowmillis, lastmillis = 0;   // for timing calculation in millis
  uint32_t duration = 0;                          // total duration in ms
  uint16_t seconds = 0;                           // total duration in seconds
  uint8_t  lastkey1 = 0, lastkey2 = 0;            // for key pressed dectection

  // Loop
  while(1) {                                      // loop until forever
    // Read sensor values
    volt = INA_readVoltage();                     // read voltage in mV from INA219
    curr = INA_readCurrent();                     // read current in mA from INA219  

    // Calculate timings
    nowmillis   = MIL_read();                     // read millis counter
    interval    = nowmillis - lastmillis;         // calculate recent time interval
    lastmillis  = nowmillis;                      // reset lastmillis
    duration   += interval;                       // calculate total duration in millis
    seconds     = duration / 1000;                // calculate total duration in seconds
  
    // Calculate power, capacity and energy
    power   = (uint32_t)volt * curr / 1000;       // calculate power  in mW
    energy += interval * power / 3600;            // calculate energy in uWh
    charge += interval * curr  / 3600;            // calculate charge in uAh

    // Check SET button
    if(pinRead(PIN_KEY1)) lastkey1 = 0;
    else if(!lastkey1) {
      CH224K_nextVolt();
      lastkey1++;
    }

    // Check RESET button
    if(pinRead(PIN_KEY2)) lastkey2 = 0;
    else if(!lastkey2) {
      duration = 0; seconds = 0; energy = 0; charge = 0;
      lastkey2++;
    }

    // Display values on the OLED
    OLED_setCursor(0,0);
    OLED_printStr(SET); OLED_printDec(CH224K_getVolt(), SPACE);
    OLED_printStr(CH224K_isGood() ? CMK : HGL);
    OLED_printVal(volt); OLED_printStr(mV);

    OLED_setCursor(0,2);
    switch(seconds & 0x0C) {
      case 0x00:  if(power > 65535) {
                    OLED_printVal(power / 1000);
                    OLED_printStr(Wt);
                  } else {
                    OLED_printVal(power);
                    OLED_printStr(mW);
                  }
                  break;
      case 0x04:  if(energy > 65535) {
                    OLED_printVal(energy / 1000000);
                    OLED_printStr(Wh);
                  } else {
                    OLED_printVal(energy / 1000);
                    OLED_printStr(mWh);
                  }
                  break;
      case 0x08:  if(charge > 65535) {
                    OLED_printVal(charge / 1000000);
                    OLED_printStr(Ah);
                  } else {
                    OLED_printVal(charge / 1000);
                    OLED_printStr(mAh);
                  }
                  break;
      case 0x0C:  OLED_printDec(seconds / 3600, 0); OLED_printChar(COLON);
                  seconds %= 3600;
                  OLED_printDec(seconds / 60  , 0); OLED_printChar(COLON);            
                  OLED_printDec(seconds % 60  , 0);
                  break;
      default:    break;
    }
    OLED_printStr(SEP);
    OLED_printVal(curr); OLED_printStr(mA);
    _delay_ms(50);
  }
}
