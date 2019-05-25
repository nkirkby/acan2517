//——————————————————————————————————————————————————————————————————————————————
//  ACAN2517 Demo in loopback mode, for ESP32
//——————————————————————————————————————————————————————————————————————————————

#ifndef ARDUINO_ARCH_ESP32
  #error "Select an ESP32 board"
#endif

//——————————————————————————————————————————————————————————————————————————————

#include <ACAN2517.h>
#include <SPI.h>
#include "driver/ledc.h"
#define LED_BUILTIN 12

//——————————————————————————————————————————————————————————————————————————————
//  For using SPI on ESP32, see demo sketch SPI_Multiple_Buses
//  Two SPI busses are available in Arduino, HSPI and VSPI.
//  By default, Arduino SPI use VSPI, leaving HSPI unused.
//  Default VSPI pins are: SCK=18, MISO=19, MOSI=23.
//  You can change the default pin with additional begin arguments
//    SPI.begin (MCP2517_SCK, MCP2517_MISO, MCP2517_MOSI)
//  CS input of MCP2517 should be connected to a digital output port
//  INT output of MCP2517 should be connected to a digital input port, with interrupt capability
//  Notes:
//    - GPIOs 34 to 39 are GPIs – input only pins. These pins don’t have internal pull-ups or
//      pull-down resistors. They can’t be used as outputs.
//    - some pins do not support INPUT_PULLUP (see https://www.esp32.com/viewtopic.php?t=439)
//    - All GPIOs can be configured as interrupts
// See https://randomnerdtutorials.com/esp32-pinout-reference-gpios/
//——————————————————————————————————————————————————————————————————————————————

static const byte MCP2517_SCK  = 18 ; // SCK input of MCP2517
static const byte MCP2517_MOSI = 23 ; // SDI input of MCP2517
static const byte MCP2517_MISO = 19 ; // SDO output of MCP2517

// static const byte MCP2517_CS  = 5 ; // CS input of MCP2517
// static const byte MCP2517_INT = 27 ; // INT output of MCP2517
static const byte MCP2517_CS  = 32 ; // CS input of MCP2517
static const byte MCP2517_INT = 36 ; // INT output of MCP2517

//——————————————————————————————————————————————————————————————————————————————
//  ACAN2517 Driver object
//——————————————————————————————————————————————————————————————————————————————

ACAN2517 can (MCP2517_CS, SPI, MCP2517_INT) ;
ACAN2517 can2 (5, SPI, 27);

//——————————————————————————————————————————————————————————————————————————————
//   SETUP
//——————————————————————————————————————————————————————————————————————————————

void configure_MCP_oscillator(void)
{
    ledc_timer_config_t ledc_timer;
    ledc_channel_config_t ledc_channel;

    ledc_timer.duty_resolution = LEDC_TIMER_1_BIT; // resolution of PWM duty
    ledc_timer.freq_hz = 40000000;                 // frequency of PWM signal
    ledc_timer.speed_mode = LEDC_HIGH_SPEED_MODE;  // timer mode
    ledc_timer.timer_num = LEDC_TIMER_0;           // timer index

    ledc_channel.channel    = LEDC_CHANNEL_0;
    ledc_channel.duty       = 0;
    ledc_channel.gpio_num   = 0;
    ledc_channel.speed_mode = ledc_timer.speed_mode;
    ledc_channel.hpoint     = 0;
    ledc_channel.timer_sel  = ledc_timer.timer_num;

    // Set configuration of timer0 for high speed channels
    ledc_timer_config(&ledc_timer);
    ledc_channel_config(&ledc_channel);
    ledc_set_duty(ledc_timer.speed_mode, ledc_channel.channel, 1);
}

void setup () {
  configure_MCP_oscillator();
//--- Switch on builtin led
  pinMode (LED_BUILTIN, OUTPUT) ;
  digitalWrite (LED_BUILTIN, HIGH) ;
//--- Start serial
  Serial.begin (115200) ;
//--- Wait for serial (blink led at 10 Hz during waiting)
  while (!Serial) {
    delay (50) ;
    digitalWrite (LED_BUILTIN, !digitalRead (LED_BUILTIN)) ;
  }
//----------------------------------- Begin SPI
  SPI.begin (MCP2517_SCK, MCP2517_MISO, MCP2517_MOSI) ;
//--- Configure ACAN2517
  Serial.print ("sizeof (ACAN2517Settings): ") ;
  Serial.print (sizeof (ACAN2517Settings)) ;
  Serial.println (" bytes") ;
  Serial.println ("Configure ACAN2517") ;
  ACAN2517Settings settings (ACAN2517Settings::OSC_40MHz, 500 * 1000) ; // CAN bit rate 125 kb/s
  settings.mRequestedMode = ACAN2517Settings::InternalLoopBack ; // Select loopback mode
  const uint32_t errorCode = can.begin (settings, [] { can.isr () ; }) ;
  const uint32_t err2 = can2.begin( settings, [] {can2.isr() ; }) ;
  if (errorCode == 0 && err2 == 0) {
    Serial.print ("Bit Rate prescaler: ") ;
    Serial.println (settings.mBitRatePrescaler) ;
    Serial.print ("Phase segment 1: ") ;
    Serial.println (settings.mPhaseSegment1) ;
    Serial.print ("Phase segment 2: ") ;
    Serial.println (settings.mPhaseSegment2) ;
    Serial.print ("SJW:") ;
    Serial.println (settings.mSJW) ;
    Serial.print ("Actual bit rate: ") ;
    Serial.print (settings.actualBitRate ()) ;
    Serial.println (" bit/s") ;
    Serial.print ("Exact bit rate ? ") ;
    Serial.println (settings.exactBitRate () ? "yes" : "no") ;
    Serial.print ("Sample point: ") ;
    Serial.print (settings.samplePointFromBitStart ()) ;
    Serial.println ("%") ;
  }else{
    Serial.print ("Configuration error 0x") ;
    Serial.println (errorCode, HEX) ;
  }
}

//——————————————————————————————————————————————————————————————————————————————
//   LOOP
//——————————————————————————————————————————————————————————————————————————————

static uint32_t gBlinkLedDate = 0 ;
static uint32_t gReceivedFrameCount = 0 ;
static uint32_t gReceivedFrameCount2 = 0 ;
static uint32_t gSentFrameCount = 0 ;

//——————————————————————————————————————————————————————————————————————————————

void loop () {
  CANMessage frame ;
  if (gBlinkLedDate < millis ()) {
    gBlinkLedDate += 1;
    digitalWrite (LED_BUILTIN, !digitalRead (LED_BUILTIN)) ;
    const bool ok = can.tryToSend (frame) ;
    const bool ok2 = can2.tryToSend (frame) ;
    if (ok && ok2) {
      gSentFrameCount += 1 ;
      Serial.print ("both Sent: ") ;
      Serial.println (gSentFrameCount) ;
    }else{
      Serial.println ("Send failure") ;
    }
  }
  if (can.available ()) {
    can.receive (frame) ;
    gReceivedFrameCount ++ ;
    Serial.print ("1 Received: ") ;
    Serial.println (gReceivedFrameCount) ;
  }
  if (can2.available ()) {
    can2.receive (frame) ;
    gReceivedFrameCount2 ++ ;
    Serial.print ("2 Received: ") ;
    Serial.println (gReceivedFrameCount2) ;
  }
}

//——————————————————————————————————————————————————————————————————————————————
