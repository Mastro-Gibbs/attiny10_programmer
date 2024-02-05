/**********************************************************************************
 * TPI programmer for:                                                            *
 * - ATtiny4                                                                      *
 * - ATtiny5                                                                      *
 * - ATtiny9                                                                      *
 * - ATtiny10                                                                     *
 *                                                                                *
 * Make the connections as shown below.                                           *
 *                                                                                *
 *      Arduino                                       ATtiny10                    *
 *      ----------+                              +-------U-------+                *
 *      (SCK)  13 |--[R]------+   +---(TPIDATA)--| 1 PB0   PB3 6 |---(RESET)      *
 *                |           |   |              |               |      |         *
 *      (MOSI) 12 |--[R]--+---C---+            --| 2 GND   Vcc 5 |--    |         *   
 *                |       |   |                  |               |      |         *
 *      (MISO) 11 |--[R]--+   +--------(TPICLK)--| 3 PB1   PB2 4 |--    |         *
 *                |                              +---------------+      |         *
 *      (SS#)  10 |--[R]------------------------------------------------+         *
 *      ----------+                                                               *
 *                                                                                *
 *  -[R]- is a few kOhm resistor.                                                 *
 *  Tested successfully with 4.67 kOhm resistor                                   *
 *                                                                                *
 *  03/02/2024  - MastroGibbs                                                     *
 *                                                                                *
 * thanks to https://github.com/ericheisler from which this code is inspired      *
 *                                                                                *
 **********************************************************************************/

#include <SPI.h>
#include "pins_arduino.h"

// instructions
#define SLD                 0x20
#define SLDp                0x24
#define SST                 0x60
#define SSTp                0x64
#define SSTPRH              0x69
#define SSTPRL              0x68
#define SKEY                0xE0
#define NVM_PROGRAM_ENABLE  0x1289AB45CDD888FFULL
#define NVMCMD              0x33
#define NVMCSR              0x32
#define NVM_NOP             0x00
#define NVM_CHIP_ERASE      0x10
#define NVM_SECTION_ERASE   0x14
#define NVM_WORD_WRITE      0x1D

// reprogramming status
#define VALID_DEVICE_ID     0x01
#define INVALID_DEVICE_ID   0x00
#define PROG_MODE_EN        0x01
#define PROG_MODE_NEN       0x00
#define VALID_PROGRAM       0x01
#define INVALID_PROGRAM     0x00

// max UART program buffer
#define PROGRAM_MAX_LEN     1024

// fuse Actions
#define FUSE_CLR            0x00
#define FUSE_SET            0x01

// memmap ranges
#define MEMMAP_REGS_BEGIN   0x0000
#define MEMMAP_REGS_END     0x003F
#define MEMMAP_SRAM_BEGIN   0x0040
#define MEMMAP_SRAM_END     0x005F
#define MEMMAP_LOCK_BITS    0x3F00
#define MEMMAP_FUSE         0x3F40
#define MEMMAP_CALIBRATION  0x3F80
#define MEMMAP_DEVICE_ID    0x3FC0
#define MEMMAP_FLASH_BEGIN  0x4000
#define MEMMAP_FLASH_END    0x4400

// attiny IDs
#define ATTINY4_DEVID       0x001E8F0A
#define ATTINY5_DEVID       0x001E8F09
#define ATTINY9_DEVID       0x001E9008
#define ATTINY10_DEVID      0x001E9003

// I/O Space registers addresses
#define IO_PINB             0x0000
#define IO_DDRB             0x0001
#define IO_PORTB            0x0002
#define IO_PUEB             0x0003
#define IO_PORTCR           0x000C
#define IO_PCMSK            0x0010
#define IO_PCIFR            0x0011
#define IO_PCICR            0x0012
#define IO_EIMSK            0x0013
#define IO_EIFR             0x0014
#define IO_EICRA            0x0015
#define IO_DIDR0            0x0017
#define IO_ADCL             0x0019
#define IO_ADMUX            0x001B
#define IO_ADCSRB           0x001C
#define IO_ADCSRA           0x001D
#define IO_ACSR             0x001F
#define IO_ICR0L            0x0022
#define IO_ICR0H            0x0023
#define IO_OCR0BL           0x0024
#define IO_OCR0BH           0x0025
#define IO_OCR0AL           0x0026
#define IO_OCR0AH           0x0027
#define IO_TCNT0L           0x0028
#define IO_TCNT0H           0x0029
#define IO_TIFR0            0x002A
#define IO_TIMSK0           0x002B
#define IO_TCCR0C           0x002C
#define IO_TCCR0B           0x002D
#define IO_TCCR0A           0x002E
#define IO_GTCCR            0x002F
#define IO_WDTCSR           0x0031
#define IO_NVMCSR           0x0032
#define IO_NVMCMD           0x0033
#define IO_VLMCSR           0x0034
#define IO_PRR              0x0035
#define IO_CLKPSR           0x0036
#define IO_CLKMSR           0x0037
#define IO_OSCCAL           0x0039
#define IO_SMCR             0x003A
#define IO_RSTFLR           0x003B
#define IO_CCP              0x003C
#define IO_SPL              0x003D
#define IO_SPH              0x003E
#define IO_SREG             0x003F


uint8_t  cmd;                       // used to detect main menu action
uint8_t  fuse_cmd;                  // used to detect fuse setting action

uint32_t device_id;                 // device id
uint8_t  device_id_valid;           // device id control variable

uint8_t  program_valid;             // UART program control
uint8_t  program[PROGRAM_MAX_LEN];  // UART program buffer 
uint32_t program_size;              // UART program size

uint8_t  prog_mode_enabled;         // control device status [under programming or not]


void setup()
{
  Serial.begin(115200);

  while (!Serial)
  {
    delay(1000);
  }

  // set up SPI
  SPI.begin();
  SPI.setBitOrder(LSBFIRST);
  SPI.setDataMode(SPI_MODE0);
  SPI.setClockDivider(SPI_CLOCK_DIV32);

  memset(program, 0x00, PROGRAM_MAX_LEN);
  program_size  = 0;

  device_id_valid   = INVALID_DEVICE_ID;
  program_valid      = INVALID_PROGRAM;
  prog_mode_enabled = PROG_MODE_NEN;

  Serial.println(F("Programmer ready!"));
}

void loop()
{
  action_menu();
  get_menu_command(&cmd);

  switch (cmd)
  {
  case 'W':
  case 'w':
    enter_prog_mode();
    break;

  case 'I':
  case 'i':
    check_device_id();
    break;

  case 'R':
  case 'r':
    receive_program();
    break;

  case 'O':
  case 'o':
    show_program();
    break;

  case 'P':
  case 'p':
    write_program();
    break;

  case 'V':
  case 'v':
    verify_program();
    break;

  case 'D':
  case 'd':
    memory_dump();
    break;

  case 'E':
  case 'e':
    erase_chip();
    break;

  case 'X':
  case 'x':
    leave_prog_mode();
    break;

  case 'F':
  case 'f':
    show_fuse();
    break;

  case 'S':
  case 's':
    set_fuse_cfg(FUSE_SET);
    break;

  case 'C':
  case 'c':
    set_fuse_cfg(FUSE_CLR);
    break;
  
  default:
    Serial.println(F("Received unknown command"));
    break;
  }

  while (Serial.available() > 0)
  {
    (void) Serial.read();
  }

  Serial.flush();
}

void action_menu(void)
{
  Serial.println();
  Serial.println(F("--------------------------------------------"));
  Serial.println(F("Actions menu"));
  Serial.println();
  Serial.println(F("| 'W' | Enter programming mode"));
  Serial.println(F("| 'I' | Read device ID"));
  Serial.println(F("| 'R' | Read program via UART"));
  Serial.println(F("| 'O' | Show received program"));
  Serial.println(F("| 'P' | Program device"));
  Serial.println(F("| 'V' | Verify flash"));
  Serial.println(F("| 'D' | Dump memory"));
  Serial.println(F("| 'E' | Erase chip (done automatically by 'P')"));
  Serial.println(F("| 'X' | Exit programming mode"));
  Serial.println(F("| 'F' | Show fuse"));
  Serial.println(F("| 'S' | Set fuse menu"));
  Serial.println(F("| 'C' | Clear fuse menu"));
  Serial.println();
  Serial.println(F("Waiting for command"));

  return;
}

void show_fuse_cfg_menu(void)
{
  Serial.println(F("--------------------------------------------"));
  Serial.println(F("Fuse setting menu"));
  Serial.println();
  Serial.println(F("| 'C' | System clock output"));
  Serial.println(F("| 'W' | Watchdog timer"));
  Serial.println(F("| 'R' | Reset bit"));
  Serial.println(F("| 'X' | Cancel"));
  Serial.println();
  Serial.println(F("Waiting for command"));

  return;
}

void get_menu_command(uint8_t* command)
{
  while (Serial.available() < 1)
  {
    asm("NOP");
    delay(10);
  }

  *command = Serial.read();

  Serial.print(F("Received '"));
  Serial.print((char) *command);
  Serial.println(F("' command"));

  while (Serial.available() > 0)
  {
    (void) Serial.read();
  }

  Serial.println(F("--------------------------------------------"));
  Serial.println();

  return;
}

void enter_prog_mode(void)
{
  if (prog_mode_enabled == PROG_MODE_EN)
  {
    Serial.println(F("Device is already in programming mode"));
    return;
  }

  Serial.println(F("Entering into programming mode"));
  Serial.println(F("Check wirings if you are stuck into this function"));

  // enter TPI programming mode
  digitalWrite(SS, LOW);  // assert RESET on tiny
  delay(1);               // t_RST min = 400 ns @ Vcc = 5 V

  SPI.transfer(0xff);     // activate TPI by emitting
  SPI.transfer(0xff);     // 16 or more pulses on TPICLK
  SPI.transfer(0xff);     // while holding TPIDATA to "1"

  write_CSS(0x02, 0x04);  // TPIPCR, guard time = 8bits (default=128)

  send_skey(NVM_PROGRAM_ENABLE); // enable NVM interface

  // wait for NVM to be enabled
  while ((read_CSS(0x00) & 0x02) < 1)
  {
    delay(5);
    asm("NOP");
  }

  Serial.println(F("NVM enabled"));
  Serial.println(F("Device under programming mode"));

  // initialize memory pointer register
  set_pointer(0x0000);

  prog_mode_enabled = PROG_MODE_EN;

  return;
}

void leave_prog_mode(void)
{
  Serial.println(F("Leaving programming mode"));
  Serial.println(F("Resetting device"));

  digitalWrite(SS, LOW);   // acquire RESET
  delay(1);                // t_RST min = 400 ns @ Vcc = 5 V

  write_CSS(0x00, 0x00);
  SPI.transfer(0xff);
  SPI.transfer(0xff);

  digitalWrite(SS, HIGH); // release RESET
  delay(1);               // t_RST min = 400 ns @ Vcc = 5 V

  memset(program, 0x00, PROGRAM_MAX_LEN);
  program_size  = 0;

  device_id_valid   = INVALID_DEVICE_ID;
  program_valid     = INVALID_PROGRAM;
  prog_mode_enabled = PROG_MODE_NEN;

  Serial.println(F("Device is free"));

  return;
}

void check_device_id(void)
{
  if (prog_mode_enabled == PROG_MODE_NEN)
  {
    Serial.println(F("Enter first in programming mode"));
    return;
  }

  device_id = 0x00;

  set_pointer(MEMMAP_DEVICE_ID);

  tpi_send_byte(SLDp);
  device_id |= ((uint32_t) tpi_receive_byte() << 16);

  tpi_send_byte(SLDp);
  device_id |= ((uint32_t) tpi_receive_byte() << 8);

  tpi_send_byte(SLDp);
  device_id |= ((uint32_t) tpi_receive_byte() << 0);

  Serial.print(F("Read ID: 0x"));
  Serial.println(device_id, HEX);

  device_id_valid = VALID_DEVICE_ID;

  switch (device_id)
  {
  case ATTINY4_DEVID:
    Serial.print(F("ATtiny4 device found, ID: 0x"));
    Serial.println(ATTINY4_DEVID, HEX);
    break;

  case ATTINY5_DEVID:
    Serial.print(F("ATtiny5 device found, ID: 0x"));
    Serial.println(ATTINY5_DEVID, HEX);
    break;

  case ATTINY9_DEVID:
    Serial.print(F("ATtiny9 device found, ID: 0x"));
    Serial.println(ATTINY9_DEVID, HEX);
    break;

  case ATTINY10_DEVID:
    Serial.print(F("ATtiny10 device found, ID: 0x"));
    Serial.println(ATTINY10_DEVID, HEX);
    break;
  
  default:
    Serial.println(F("Device ID not valid!"));
    Serial.println(F("Expected IDs:"));

    Serial.print(F("ATtiny4:  0x"));
    Serial.println(ATTINY4_DEVID,  HEX);

    Serial.print(F("ATtiny5:  0x"));
    Serial.println(ATTINY5_DEVID,  HEX);

    Serial.print(F("ATtiny9:  0x"));
    Serial.println(ATTINY9_DEVID,  HEX);

    Serial.print(F("ATtiny10: 0x"));
    Serial.println(ATTINY10_DEVID, HEX);

    Serial.println();

    leave_prog_mode();
    
    device_id_valid = INVALID_DEVICE_ID;
    break;
  }

  return;
}

void memory_dump(void)
{
  uint8_t b;
  uint32_t address = 0x00;

  if (prog_mode_enabled == PROG_MODE_NEN)
  {
    Serial.println(F("Enter first in programming mode"));
    return;
  }

  if (device_id_valid == INVALID_DEVICE_ID)
  {
    Serial.println(F("Read device ID first or invalid device ID"));
    return;
  }

  Serial.println(F("AVR Memories"));

  // ---------- REGS -------------
  Serial.println(F("I/O SPACE"));
  set_pointer(IO_PINB);
  tpi_send_byte(SLD);
  b = tpi_receive_byte();
  Serial.print(F("PINB:   0x"));
  print_hex2(b);
  Serial.println();

  set_pointer(IO_DDRB);
  tpi_send_byte(SLD);
  b = tpi_receive_byte();
  Serial.print(F("DDRB:   0x"));
  print_hex2(b);
  Serial.println();

  set_pointer(IO_PORTB);
  tpi_send_byte(SLD);
  b = tpi_receive_byte();
  Serial.print(F("PORTB:  0x"));
  print_hex2(b);
  Serial.println();

  set_pointer(IO_PUEB);
  tpi_send_byte(SLD);
  b = tpi_receive_byte();
  Serial.print(F("PUEB:   0x"));
  print_hex2(b);
  Serial.println();

  set_pointer(IO_PORTCR);
  tpi_send_byte(SLD);
  b = tpi_receive_byte();
  Serial.print(F("PORTCR: 0x"));
  print_hex2(b);
  Serial.println();

  set_pointer(IO_PCMSK);
  tpi_send_byte(SLD);
  b = tpi_receive_byte();
  Serial.print(F("PCMSK:  0x"));
  print_hex2(b);
  Serial.println();

  set_pointer(IO_PCIFR);
  tpi_send_byte(SLD);
  b = tpi_receive_byte();
  Serial.print(F("PCIFR:  0x"));
  print_hex2(b);
  Serial.println();

  set_pointer(IO_PCICR);
  tpi_send_byte(SLD);
  b = tpi_receive_byte();
  Serial.print(F("PCICR:  0x"));
  print_hex2(b);
  Serial.println();

  set_pointer(IO_EIMSK);
  tpi_send_byte(SLD);
  b = tpi_receive_byte();
  Serial.print(F("EIMSK:  0x"));
  print_hex2(b);
  Serial.println();

  set_pointer(IO_EIFR);
  tpi_send_byte(SLD);
  b = tpi_receive_byte();
  Serial.print(F("EIFR:   0x"));
  print_hex2(b);
  Serial.println();

  set_pointer(IO_EICRA);
  tpi_send_byte(SLD);
  b = tpi_receive_byte();
  Serial.print(F("EICRA:  0x"));
  print_hex2(b);
  Serial.println();

  set_pointer(IO_DIDR0);
  tpi_send_byte(SLD);
  b = tpi_receive_byte();
  Serial.print(F("DIDR0:  0x"));
  print_hex2(b);
  Serial.println();

  set_pointer(IO_ADCL);
  tpi_send_byte(SLD);
  b = tpi_receive_byte();
  Serial.print(F("ADCL:   0x"));
  print_hex2(b);
  Serial.println();

  set_pointer(IO_ADMUX);
  tpi_send_byte(SLD);
  b = tpi_receive_byte();
  Serial.print(F("ADMUX:  0x"));
  print_hex2(b);
  Serial.println();

  set_pointer(IO_ADCSRB);
  tpi_send_byte(SLD);
  b = tpi_receive_byte();
  Serial.print(F("ADCSRB: 0x"));
  print_hex2(b);
  Serial.println();

  set_pointer(IO_ADCSRA);
  tpi_send_byte(SLD);
  b = tpi_receive_byte();
  Serial.print(F("ADCSRA: 0x"));
  print_hex2(b);
  Serial.println();

  set_pointer(IO_ACSR);
  tpi_send_byte(SLD);
  b = tpi_receive_byte();
  Serial.print(F("ACSR:   0x"));
  print_hex2(b);
  Serial.println();

  set_pointer(IO_ICR0L);
  tpi_send_byte(SLD);
  b = tpi_receive_byte();
  Serial.print(F("ICR0L:  0x"));
  print_hex2(b);
  Serial.println();

  set_pointer(IO_ICR0H);
  tpi_send_byte(SLD);
  b = tpi_receive_byte();
  Serial.print(F("ICR0H:  0x"));
  print_hex2(b);
  Serial.println();

  set_pointer(IO_OCR0BL);
  tpi_send_byte(SLD);
  b = tpi_receive_byte();
  Serial.print(F("OCR0BL: 0x"));
  print_hex2(b);
  Serial.println();

  set_pointer(IO_OCR0BH);
  tpi_send_byte(SLD);
  b = tpi_receive_byte();
  Serial.print(F("OCR0BL: 0x"));
  print_hex2(b);
  Serial.println();

  set_pointer(IO_OCR0AL);
  tpi_send_byte(SLD);
  b = tpi_receive_byte();
  Serial.print(F("OCR0AL: 0x"));
  print_hex2(b);
  Serial.println();

  set_pointer(IO_OCR0AH);
  tpi_send_byte(SLD);
  b = tpi_receive_byte();
  Serial.print(F("OCR0AL: 0x"));
  print_hex2(b);
  Serial.println();

  set_pointer(IO_TCNT0L);
  tpi_send_byte(SLD);
  b = tpi_receive_byte();
  Serial.print(F("TCNT0L: 0x"));
  print_hex2(b);
  Serial.println();

  set_pointer(IO_TCNT0H);
  tpi_send_byte(SLD);
  b = tpi_receive_byte();
  Serial.print(F("TCNT0H: 0x"));
  print_hex2(b);
  Serial.println();

  set_pointer(IO_TIFR0);
  tpi_send_byte(SLD);
  b = tpi_receive_byte();
  Serial.print(F("TIFR0:  0x"));
  print_hex2(b);
  Serial.println();

  set_pointer(IO_TIMSK0);
  tpi_send_byte(SLD);
  b = tpi_receive_byte();
  Serial.print(F("TIMSK0: 0x"));
  print_hex2(b);
  Serial.println();

  set_pointer(IO_TCCR0C);
  tpi_send_byte(SLD);
  b = tpi_receive_byte();
  Serial.print(F("TCCR0C: 0x"));
  print_hex2(b);
  Serial.println();

  set_pointer(IO_TCCR0B);
  tpi_send_byte(SLD);
  b = tpi_receive_byte();
  Serial.print(F("TCCR0B: 0x"));
  print_hex2(b);
  Serial.println();

  set_pointer(IO_TCCR0A);
  tpi_send_byte(SLD);
  b = tpi_receive_byte();
  Serial.print(F("TCCR0A: 0x"));
  print_hex2(b);
  Serial.println();

  set_pointer(IO_GTCCR);
  tpi_send_byte(SLD);
  b = tpi_receive_byte();
  Serial.print(F("GTCCR:  0x"));
  print_hex2(b);
  Serial.println();

  set_pointer(IO_WDTCSR);
  tpi_send_byte(SLD);
  b = tpi_receive_byte();
  Serial.print(F("WDTCSR: 0x"));
  print_hex2(b);
  Serial.println();

  set_pointer(IO_NVMCSR);
  tpi_send_byte(SLD);
  b = tpi_receive_byte();
  Serial.print(F("NVMCSR: 0x"));
  print_hex2(b);
  Serial.println();

  set_pointer(IO_NVMCMD);
  tpi_send_byte(SLD);
  b = tpi_receive_byte();
  Serial.print(F("NVMCMD: 0x"));
  print_hex2(b);
  Serial.println();

  set_pointer(IO_VLMCSR);
  tpi_send_byte(SLD);
  b = tpi_receive_byte();
  Serial.print(F("VLMCSR: 0x"));
  print_hex2(b);
  Serial.println();

  set_pointer(IO_PRR);
  tpi_send_byte(SLD);
  b = tpi_receive_byte();
  Serial.print(F("PRR:    0x"));
  print_hex2(b);
  Serial.println();

  set_pointer(IO_CLKPSR);
  tpi_send_byte(SLD);
  b = tpi_receive_byte();
  Serial.print(F("CLKPSR: 0x"));
  print_hex2(b);
  Serial.println();

  set_pointer(IO_CLKMSR);
  tpi_send_byte(SLD);
  b = tpi_receive_byte();
  Serial.print(F("CLKMSR: 0x"));
  print_hex2(b);
  Serial.println();

  set_pointer(IO_OSCCAL);
  tpi_send_byte(SLD);
  b = tpi_receive_byte();
  Serial.print(F("OSCCAL: 0x"));
  print_hex2(b);
  Serial.println();

  set_pointer(IO_SMCR);
  tpi_send_byte(SLD);
  b = tpi_receive_byte();
  Serial.print(F("SMCR:   0x"));
  print_hex2(b);
  Serial.println();

  set_pointer(IO_RSTFLR);
  tpi_send_byte(SLD);
  b = tpi_receive_byte();
  Serial.print(F("RSTFLR: 0x"));
  print_hex2(b);
  Serial.println();

  set_pointer(IO_CCP);
  tpi_send_byte(SLD);
  b = tpi_receive_byte();
  Serial.print(F("CCP:    0x"));
  print_hex2(b);
  Serial.println();

  set_pointer(IO_SPL);
  tpi_send_byte(SLD);
  b = tpi_receive_byte();
  Serial.print(F("SPL:    0x"));
  print_hex2(b);
  Serial.println();

  set_pointer(IO_SPH);
  tpi_send_byte(SLD);
  b = tpi_receive_byte();
  Serial.print(F("SPH:    0x"));
  print_hex2(b);
  Serial.println();

  set_pointer(IO_SREG);
  tpi_send_byte(SLD);
  b = tpi_receive_byte();
  Serial.print(F("SREG:   0x"));
  print_hex2(b);
  Serial.println();

  // ---------- NVM LOCK -------------
  set_pointer(MEMMAP_LOCK_BITS);

  tpi_send_byte(SLD);
  b = tpi_receive_byte();

  Serial.println();
  Serial.println();
  Serial.print(F("NVM LOCK:    0x"));
  print_hex2(b);
  Serial.println();

  // ---------- fuse -------------
  set_pointer(MEMMAP_FUSE);
  tpi_send_byte(SLD);
  b = tpi_receive_byte();

  Serial.print(F("FUSE:        0x"));
  print_hex2(b);
  Serial.println();

  // ---------- calibration -------------
  set_pointer(MEMMAP_CALIBRATION);
  tpi_send_byte(SLD);
  b = tpi_receive_byte();

  Serial.print(F("CALIBRATION: 0x"));
  print_hex2(b);
  Serial.println();

  // =========== device id -------------
  set_pointer(MEMMAP_DEVICE_ID);

  Serial.print(F("DEVICE ID:   0x"));

  tpi_send_byte(SLDp);
  b = tpi_receive_byte();
  print_hex2(b);

  tpi_send_byte(SLDp);
  b = tpi_receive_byte();
  print_hex2(b);

  tpi_send_byte(SLDp);
  b = tpi_receive_byte();
  print_hex2(b);

  Serial.println();

  // ---------- SRAM -------------
  set_pointer(MEMMAP_SRAM_BEGIN);
  address = MEMMAP_SRAM_BEGIN;

  Serial.println();
  Serial.println();
  Serial.println(F("SRAM DATA MEMORY"));
  Serial.print("     ");
        
  for (uint8_t i = 0; i < 16; i++)
  {
    Serial.print(" 0");
    print_hex1(i);
  }

  Serial.println();
  Serial.print(F("      -----------------------------------------------"));

  while (address <= MEMMAP_SRAM_END)
  {
    if (0 == (0x000f & address))
    {
      Serial.println();
      print_hex4(address);
      Serial.print(':');
    }

    tpi_send_byte(SLDp);
    b = tpi_receive_byte();

    Serial.print(' '); // delimiter
    print_hex2(b);     // print data in hex 2 digits

    address++;
  }


  // -------------- flash ------------------
  set_pointer(MEMMAP_FLASH_BEGIN);
  address = MEMMAP_FLASH_BEGIN;

  Serial.println();
  Serial.println();
  Serial.println();
  Serial.println(F("FLASH PROGRAM MEMORY"));
  Serial.print("     ");
        
  for (uint8_t i = 0; i < 16; i++)
  {
    Serial.print(" 0");
    print_hex1(i);
  }

  Serial.println();
  Serial.print(F("      -----------------------------------------------"));

  while (address < MEMMAP_FLASH_END)
  {
    if (0 == (0x000f & address))
    {
      Serial.println();
      print_hex4(address);
      Serial.print(':');
    }

    tpi_send_byte(SLDp);
    b = tpi_receive_byte();

    Serial.print(' '); // delimiter
    print_hex2(b);     // print data in hex 2 digits

    address++;
  }

  Serial.println();

  return;
}

void receive_program(void)
{
  char c1, c2;

  uint16_t addr        = 0x0000;
  uint8_t  record_type = 0x00;
  uint8_t  byte_count  = 0x00;
  uint8_t  exit        = 0x00;
  uint8_t  record_crc  = 0x00;
  uint8_t  current_crc = 0x0000;
  uint8_t  temp_byte   = 0x00;

  unsigned long btime        = 0;
  unsigned long recv_timeout = 5000;

  memset(program, 0x00, PROGRAM_MAX_LEN);
  program_valid = INVALID_PROGRAM;
  program_size  = 0;
  
  Serial.println(F("Waiting for hex stream..."));

  while (Serial.available() == 0)
  {
    delayMicroseconds(50);
  }

  // read in the data and
  while (exit == 0x00)
  {
    current_crc = 0x0000;
    btime = millis();

    while (Serial.available() < 10)
    {
      if (millis() - btime > recv_timeout)
      {
        Serial.println(F("Couldn't receive data: Timed out"));
        return;
      }
    }

    if (Serial.read() != ':')
    { 
      // maybe it was a newline
      if (Serial.read() != ':')
      {
        Serial.println(F("Couldn't receive data: hex file format error"));
        return;
      }
    }

    // read record data length
    c1 = Serial.read();
    c2 = Serial.read();
    temp_byte  = byteval(c1, c2);
    byte_count = temp_byte;
    current_crc += temp_byte;

    // read record address 
    // if "0000" program_size = 0
    c1    = Serial.read();
    c2    = Serial.read();
    temp_byte    = byteval(c1, c2);
    current_crc += temp_byte;
    addr |= temp_byte << 8;

    c1    = Serial.read();
    c2    = Serial.read();
    temp_byte    = byteval(c1, c2);
    current_crc += temp_byte;
    addr |= temp_byte << 0;

    if (byte_count != 0x00 && addr == 0x0000)
    {
      program_size = 0;
    }

    // read type thingy. "01" means end of file
    c1 = Serial.read();
    c2 = Serial.read();
    temp_byte    = byteval(c1, c2);
    current_crc += temp_byte;
    record_type  = temp_byte;

    if (record_type == 0x01)
    {
      exit = 0x01;
    }

    // read in the data
    for (int k = 0; k < byte_count; k++)
    {
      if (program_size == PROGRAM_MAX_LEN)
      {
        Serial.println(F("Couldn't receive data: program is too large"));
        return;
      }

      btime = millis();

      while (Serial.available() < 2)
      {
        if (millis() - btime > recv_timeout)
        {
          Serial.println(F("Couldn't receive data: Timed out"));
          return;
        }
      }

      c1 = Serial.read();
      c2 = Serial.read();
      temp_byte    = byteval(c1, c2);
      current_crc += temp_byte;
      program[program_size] = temp_byte;
      program_size++;
    }

    // read in the checksum.
    btime = millis();
    while (Serial.available() < 2)
    {
      if (millis() - btime > recv_timeout)
      {
        Serial.println(F("Couldn't receive data: Timed out"));
        return;
      }
    }

    c1  = Serial.read();
    c2  = Serial.read();
    record_crc = byteval(c1, c2);

    current_crc = (~current_crc) + 1;

    if (record_crc != current_crc)
    {
      while (Serial.available() > 0)
      {
        (void) Serial.read();
      }

      Serial.flush();

      Serial.println(F("CRC not match"));

      Serial.print(F("Expected: 0x"));
      print_hex2(record_crc);
      Serial.println();

      Serial.print(F("Computed: 0x"));
      print_hex2(current_crc);
      Serial.println();

      return;
    }
  }

  // we need an even number of bytes for word writing
  if (program_size & 0x0001)
  {
    program[program_size] = 0x00;
    program_size++;
  }

  // the program was successfully read
  Serial.println(F("Program received!"));
  Serial.print(F("Program size: "));
  Serial.print(program_size, DEC);
  Serial.println(F(" bytes"));

  program_valid = VALID_PROGRAM;
  return;
}

void show_program(void)
{
  uint8_t i;

  if (program_valid == INVALID_PROGRAM)
  {
    Serial.println(F("No program available"));
    return;
  }

  Serial.println();
  Serial.println(F("HEX PROGRAM DUMP"));
  Serial.print("     ");
        
  for (uint8_t i = 0; i < 16; i++)
  {
    Serial.print(" 0");
    print_hex1(i);
  }

  Serial.println();
  Serial.print(F("      -----------------------------------------------"));

  Serial.println();
  print_hex4(0X00);
  Serial.print(':');

  for (i = 0; i < program_size; i++)
  {  
    if (i != 0 && i % 0x10 == 0)
    {
      Serial.println();
      print_hex4(i);
      Serial.print(':');
    }

    Serial.print(' ');
    print_hex2(program[i]);    
  }  

  Serial.println();

  return;
}

void write_program(void)
{
  if (prog_mode_enabled == PROG_MODE_NEN)
  {
    Serial.println(F("Enter first in programming mode"));
    return;
  }

  if (device_id_valid == INVALID_DEVICE_ID)
  {
    Serial.println(F("Read device ID first or invalid device ID"));
    return;
  }

  if (program_valid == INVALID_PROGRAM)
  {
    Serial.println(F("No program available"));
    return;
  }

  // erase the chip
  erase_chip();

  Serial.println(F("Writing program"));

  set_pointer(MEMMAP_FLASH_BEGIN);

  write_IO(NVMCMD, NVM_WORD_WRITE);

  // now write all the bytes to program memory
  // write two bytes at a time (a word) and wait


  for (unsigned int k = 0; k < program_size - 1; k = k + 2)
  {
    tpi_send_byte(SSTp);
    tpi_send_byte(program[k]);     // LSB first
    tpi_send_byte(SSTp);
    tpi_send_byte(program[k + 1]); // then MSB

    while ((read_IO(NVMCSR) & (1 << 7)) != 0x00)
    {
      delay(5);
    }

  }

  write_IO(NVMCMD, NVM_NOP);
  SPI.transfer(0xff);
  SPI.transfer(0xff);

  Serial.print(F("Wrote program: "));
  Serial.print(program_size, DEC);
  Serial.println(F(" of 1024 bytes"));
  Serial.println();

  verify_program();

  Serial.println(F("Exit programming mode to reset the target"));
  Serial.println();

  return;
}

void erase_chip(void)
{
  if (prog_mode_enabled == PROG_MODE_NEN)
  {
    Serial.println(F("Enter first in programming mode"));
    return;
  }

  if (device_id_valid == INVALID_DEVICE_ID)
  {
    Serial.println(F("Read device ID first or invalid device ID"));
    return;
  }

  Serial.println(F("Erasing chip"));

  // initialize memory pointer register
  set_pointer(MEMMAP_FLASH_BEGIN + 1); // need the +1 for chip erase

  // erase the chip
  write_IO(NVMCMD, NVM_CHIP_ERASE);
  tpi_send_byte(SSTp);
  tpi_send_byte(0xAA);
  tpi_send_byte(SSTp);
  tpi_send_byte(0xAA);
  tpi_send_byte(SSTp);
  tpi_send_byte(0xAA);
  tpi_send_byte(SSTp);
  tpi_send_byte(0xAA);

  while ((read_IO(NVMCSR) & (1 << 7)) != 0x00)
  {
    delay(5);
  }

  Serial.println(F("Chip erased"));
  Serial.println();

  return;
}

void verify_program(void)
{
  uint8_t        b;
  boolean        correct = true;
  unsigned short ind     = 0;

  if (prog_mode_enabled == PROG_MODE_NEN)
  {
    Serial.println(F("Enter first in programming mode"));
    return;
  }

  if (device_id_valid == INVALID_DEVICE_ID)
  {
    Serial.println(F("Read device ID first or invalid device ID"));
    return;
  }

  if (program_valid == INVALID_PROGRAM)
  {
    Serial.println(F("No program available"));
    return;
  }

  Serial.println(F("Verifing program"));

  set_pointer(MEMMAP_FLASH_BEGIN);

  while (ind < program_size)
  {
    tpi_send_byte(SLDp);
    b = tpi_receive_byte(); // get data byte

    if (b != program[ind])
    {
      correct = false;
      Serial.println(F("Program error:"));
      Serial.print(F("Byte "));
      print_hex4(ind);

      Serial.print(F(" expected "));
      print_hex2(program[ind]);

      Serial.print(F(" read "));
      print_hex2(b);
      Serial.println();
    }

    ind++;
  }

  if (correct)
  {
    Serial.println(F("Program verified successfully"));
  }

  Serial.println();

  return;
}

void show_fuse(void)
{
  uint8_t fuse;

  if (prog_mode_enabled == PROG_MODE_NEN)
  {
    Serial.println(F("Enter first in programming mode"));
    return;
  }

  if (device_id_valid == INVALID_DEVICE_ID)
  {
    Serial.println(F("Read device ID first or invalid device ID"));
    return;
  }

  set_pointer(MEMMAP_FUSE);
  tpi_send_byte(SLD);
  fuse = tpi_receive_byte();

  Serial.print(F("Fuse: 0x"));
  print_hex2(fuse);
  Serial.println();

  return;
}

void set_fuse_cfg(uint8_t action)
{
  uint8_t current_fuse;

  if (prog_mode_enabled == PROG_MODE_NEN)
  {
    Serial.println(F("Enter first in programming mode"));
    return;
  }

  if (device_id_valid == INVALID_DEVICE_ID)
  {
    Serial.println(F("Read device ID first or invalid device ID"));
    return;
  }

  // get current config byte
  set_pointer(MEMMAP_FUSE);
  tpi_send_byte(SLD);
  current_fuse = tpi_receive_byte();

  show_fuse_cfg_menu();
  get_menu_command(&fuse_cmd);

  set_pointer(MEMMAP_FUSE);
  write_IO(NVMCMD, (action == FUSE_SET ? NVM_WORD_WRITE : NVM_SECTION_ERASE));

  switch (fuse_cmd)
  {
    case 'C':
    case 'c':
      tpi_send_byte(SSTp);

      (action == FUSE_SET) ? tpi_send_byte(current_fuse & 0b11111011) : tpi_send_byte(current_fuse | 0x04);

      tpi_send_byte(SSTp);
      tpi_send_byte(0xFF);
      break;

    case 'W':
    case 'w':
      tpi_send_byte(SSTp);

      (action == FUSE_SET) ? tpi_send_byte(current_fuse & 0b11111101) : tpi_send_byte(current_fuse | 0x02);

      tpi_send_byte(SSTp);
      tpi_send_byte(0xFF);
      break;

    case 'R':
    case 'r':
      tpi_send_byte(SSTp);

      (action == FUSE_SET) ? tpi_send_byte(current_fuse & 0b11111110) : tpi_send_byte(current_fuse | 0x01);

      tpi_send_byte(SSTp);
      tpi_send_byte(0xFF);
      break;

    case 'X':
    case 'x':
      Serial.println(F("Received cancel command, exit"));
      break;

    default:
      Serial.println(F("Received unknown command, exit"));
      break;
  }

  while ((read_IO(NVMCSR) & (1 << 7)) != 0x00)
  {
    delay(5);
  }

  write_IO(NVMCMD, NVM_NOP);

  SPI.transfer(0xff);
  SPI.transfer(0xff);

  return;
}


/*
 * send a byte in one TPI frame (12 bits)
 * (1 start + 8 data + 1 parity + 2 stop)
 * using 2 SPI data bytes (2 x 8 = 16 clocks)
 * (with 4 extra idle bits)
 */
void tpi_send_byte(uint8_t data)
{
  // compute partiy bit
  uint8_t par = data;
  par ^= (par >> 4); // b[7:4] (+) b[3:0]
  par ^= (par >> 2); // b[3:2] (+) b[1:0]
  par ^= (par >> 1); // b[1] (+) b[0]

  // REMEMBER: this is in LSBfirst mode and idle is high
  // (2 idle) + (1 start bit) + (data[4:0])
  SPI.transfer(0x03 | (data << 3));
  // (data[7:5]) + (1 parity) + (2 stop bits) + (2 idle)
  SPI.transfer(0xf0 | (par << 3) | (data >> 5));

  return;
}

/*
 * receive TPI 12-bit format byte data
 * via SPI 2 bytes (16 clocks) or 3 bytes (24 clocks)
 */
uint8_t tpi_receive_byte(void)
{
  uint8_t b1, b2, b3;
  
  //  keep transmitting high(idle) while waiting for a start bit
  do
  {
    b1 = SPI.transfer(0xff);
  } while (0xff == b1);
  
  // get (partial) data bits
  b2 = SPI.transfer(0xff);
  
  // if the first byte(b1) contains less than 4 data bits
  // we need to get a third byte to get the parity and stop bits
  if (0x0f == (0x0f & b1))
  {
    b3 = SPI.transfer(0xff);
  }

  // now shift the bits into the right positions
  // b1 should hold only idle and start bits = 0b01111111
  while (0x7f != b1)
  {            // data not aligned
    b2 <<= 1; // shift left data bits

    if (0x80 & b1)
    {             // carry from 1st byte
      b2 |= 1; // set bit
    }

    b1 <<= 1;
    b1 |= 0x01; // fill with idle bit (1)
  }
  
  // now the data byte is stored in b2
  return (b2);
}

// send the 64 bit NVM key
void send_skey(uint64_t nvm_key)
{
  tpi_send_byte(SKEY);
  
  while (nvm_key)
  {
    tpi_send_byte(nvm_key & 0xFF);
    nvm_key >>= 8;
  }

  return;
}

// sets the pointer address
void set_pointer(unsigned short address)
{
  tpi_send_byte(SSTPRL);
  tpi_send_byte(address & 0xff);
  tpi_send_byte(SSTPRH);
  tpi_send_byte((address >> 8) & 0xff);

  return;
}

// writes using SOUT
void write_IO(uint8_t address, uint8_t value)
{
  //  SOUT 0b1aa1aaaa replace a with 6 address bits
  tpi_send_byte(0x90 | (address & 0x0F) | ((address & 0x30) << 1));
  tpi_send_byte(value);

  return;
}

// reads using SIN
uint8_t read_IO(uint8_t address)
{
  //  SIN 0b0aa1aaaa replace a with 6 address bits
  tpi_send_byte(0x10 | (address & 0x0F) | ((address & 0x30) << 1));

  return tpi_receive_byte();
}

// writes to CSS
void write_CSS(uint8_t address, uint8_t value)
{
  tpi_send_byte(0xC0 | address);
  tpi_send_byte(value);

  return;
}

// reads from CSS
uint8_t read_CSS(uint8_t address)
{
  tpi_send_byte(0x80 | address);

  return tpi_receive_byte();
}

// converts two chars to one byte
// c1 is MS, c2 is LS
uint8_t byteval(char c1, char c2)
{
  uint8_t by;
  
  if (c1 <= '9')
  {
    by = c1 - '0';
  }
  else
  {
    by = c1 - 'A' + 10;
  }

  by = by << 4;

  if (c2 <= '9')
  {
    by += c2 - '0';
  }
  else
  {
    by += c2 - 'A' + 10;
  }

  return by;
}

void print_hex1(uint8_t n)
{
  Serial.print(0x0f & n, HEX);
}

void print_hex2(uint8_t n)
{
  print_hex1(n >> 4);
  print_hex1(n);
}

void print_hex4(uint16_t n)
{
  print_hex2(n >> 8);
  print_hex2(n);
}