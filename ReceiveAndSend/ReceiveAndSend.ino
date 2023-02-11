/*
 * ReceiveAndSend.cpp
 *
 * Record and play back last received IR signal at button press.
 * The logic is:
 * If the button is pressed, send the IR code.
 * If an IR code is received, record it.
 *
 * An example for simultaneous receiving and sending is in the UnitTest example.
 *
 * An IR detector/demodulator must be connected to the input IR_RECEIVE_PIN.
 *
 * A button must be connected between the input SEND_BUTTON_PIN and ground.
 * A visible LED can be connected to STATUS_PIN to provide status.
 *
 *
 * Initially coded 2009 Ken Shirriff http://www.righto.com
 *
 *  This file is part of Arduino-IRremote https://github.com/Arduino-IRremote/Arduino-IRremote.
 *
 ************************************************************************************
 * MIT License
 *
 * Copyright (c) 2009-2021 Ken Shirriff, Armin Joachimsmeyer
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is furnished
 * to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED,
 * INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A
 * PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
 * HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF
 * CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE
 * OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 ************************************************************************************
 */
#include <Arduino.h>

#include "PinDefinitionsAndMore.h" // Define macros for input and output pin etc.

/*
 * Specify which protocol(s) should be used for decoding.
 * If no protocol is defined, all protocols (except Bang&Olufsen) are active.
 * This must be done before the #include <IRremote.hpp>
 */
//#define DECODE_LG
//#define DECODE_NEC
//#define DECODE_DISTANCE_WIDTH // Universal decoder for pulse distance width protocols
// etc. see IRremote.hpp
//
#if !defined(RAW_BUFFER_LENGTH)
#  if RAMEND <= 0x4FF || RAMSIZE < 0x4FF
#define RAW_BUFFER_LENGTH  120
#  elif RAMEND <= 0xAFF || RAMSIZE < 0xAFF // 0xAFF for LEONARDO
#define RAW_BUFFER_LENGTH  400 // 600 is too much here, because we have additional uint8_t rawCode[RAW_BUFFER_LENGTH];
#  else
#define RAW_BUFFER_LENGTH  750
#  endif
#endif

//#define NO_LED_FEEDBACK_CODE // saves 92 bytes program memory
//#define EXCLUDE_UNIVERSAL_PROTOCOLS // Saves up to 1000 bytes program memory.
//#define EXCLUDE_EXOTIC_PROTOCOLS // saves around 650 bytes program memory if all other protocols are active

// MARK_EXCESS_MICROS is subtracted from all marks and added to all spaces before decoding,
// to compensate for the signal forming of different IR receiver modules. See also IRremote.hpp line 142.
#define MARK_EXCESS_MICROS    20    // Adapt it to your IR receiver module. 20 is recommended for the cheap VS1838 modules.

//#define RECORD_GAP_MICROS 12000 // Activate it for some LG air conditioner protocols

//#define DEBUG // Activate this for lots of lovely debug output from the decoders.

#include <IRremote.hpp>

uint16_t rawDataPower[67] = {4430,4570, 480,670, 480,1720, 530,1720, 530,1720, 530,620, 480,620, 530,620, 480,620, 530,620, 480,1770, 480,1770, 480,1770, 480,620, 480,670, 480,620, 480,670, 480,620, 480,670, 480,1770, 480,1770, 480,620, 480,670, 480,620, 480,670, 480,1770, 480,1770, 480,620, 480,670, 480,1720, 530,1720, 530,1720, 480,1770, 480};
uint16_t rawDataSoundadd[67] = {4430,4620, 480,620, 480,1770, 480,1770, 480,1770, 480,620, 530,620, 480,620, 530,620, 480,620, 530,1720, 530,1720, 530,1720, 530,620, 480,620, 530,620, 480,620, 530,620, 480,620, 530,1720, 530,620, 480,1770, 480,620, 530,620, 480,620, 530,1720, 480,1770, 480,670, 480,1770, 480,620, 480,1770, 480,1770, 480,1770, 480};
uint16_t rawDataSoundres[67] = {4480,4570, 480,620, 530,1720, 480,1770, 480,1770, 480,670, 480,620, 480,670, 480,620, 480,670, 480,1770, 480,1770, 480,1770, 480,620, 480,670, 480,620, 480,670, 480,1770, 480,620, 480,1770, 480,670, 480,1720, 530,620, 480,670, 480,620, 480,670, 480,1720, 530,620, 480,1770, 480,620, 530,1720, 530,1720, 480,1770, 480};
uint16_t rawDataSoundoff[67] = {4480,4570, 480,620, 530,1720, 530,1720, 480,1770, 480,670, 480,620, 480,670, 480,620, 480,670, 480,1770, 480,1770, 480,1770, 480,620, 480,670, 480,620, 480,670, 480,1770, 480,620, 480,1770, 480,1770, 480,620, 530,620, 480,620, 530,620, 480,670, 480,1720, 530,620, 480,620, 530,1720, 530,1720, 480,1770, 480,1770, 480};
uint16_t rawDataSingle[67] = {4480,4570, 480,620, 480,1770, 480,1770, 480,1770, 480,670, 480,620, 480,670, 480,620, 480,670, 480,1770, 480,1770, 480,1770, 480,620, 480,670, 480,620, 480,670, 480,1720, 530,1720, 530,1720, 530,1720, 480,670, 480,620, 530,620, 480,620, 530,620, 480,620, 530,620, 480,620, 530,1720, 480,1770, 480,1770, 480,1770, 480};

int SEND_BUTTON_PIN = APPLICATION_PIN;
int STATUS_PIN = 16;

int DELAY_BETWEEN_REPEAT = 50;
int DEFAULT_NUMBER_OF_REPEATS_TO_SEND = 3;

// Storage for the recorded code
struct storedIRDataStruct {
    IRData receivedIRData;
    // extensions for sendRaw
    uint8_t rawCode[RAW_BUFFER_LENGTH]; // The durations if raw
    uint8_t rawCodeLength; // The length of the code
} sStoredIRData;

int lastButtonState;

void storeCode(IRData *aIRReceivedData);
void sendCode(storedIRDataStruct *aIRDataToSend);

void setup() {
    Serial.begin(115200);
#if defined(__AVR_ATmega32U4__) || defined(SERIAL_PORT_USBVIRTUAL) || defined(SERIAL_USB) /*stm32duino*/|| defined(USBCON) /*STM32_stm32*/|| defined(SERIALUSB_PID) || defined(ARDUINO_attiny3217)
    delay(4000); // To be able to connect Serial monitor after reset or power up and before first print out. Do not wait for an attached Serial Monitor!
#endif
    // Just to know which program is running on my Arduino
    Serial.println(F("START " __FILE__ " from " __DATE__ "\r\nUsing library version " VERSION_IRREMOTE));

    // Start the receiver and if not 3. parameter specified, take LED_BUILTIN pin from the internal boards definition as default feedback LED
    IrReceiver.begin(IR_RECEIVE_PIN, ENABLE_LED_FEEDBACK);

    Serial.print(F("Ready to receive IR signals of protocols: "));
    printActiveIRProtocols(&Serial);
    Serial.println(F("at pin " STR(IR_RECEIVE_PIN)));

#if defined(IR_SEND_PIN)
    IrSender.begin(); // Start with IR_SEND_PIN as send pin and enable feedback LED at default feedback LED pin
    Serial.print(F("Ready to send IR signals at pin " STR(IR_SEND_PIN) " on press of button at pin "));
#else
    IrSender.begin(15, ENABLE_LED_FEEDBACK, 16); // Specify send pin and enable feedback LED at default feedback LED pin
    Serial.print(F("Ready to send IR signals at pin 0 on press of button at pin "));
#endif
    Serial.println(SEND_BUTTON_PIN);

    
}

void loop() {  
  int PowerState = 0;
  int SoundaddState = 0;
  int SoundresState = 0;
  int SoundoffState = 0;
  int SingleState = 0;

  if (IrReceiver.decode()) {

    IrReceiver.printIRResultShort(&Serial);
  
    sStoredIRData.receivedIRData.flags = 0; // clear flags -esp. repeat- for later sending
    Serial.println();
    
    switch(IrReceiver.decodedIRData.command){
      case 0x15:
        PowerState = 1;
        break;
      case 0x12:
        SoundaddState = 1;
        break;
      case 0x13:
        SoundresState = 1;
        break;
      case 0x14:
        SoundoffState = 1;
        break;
      case 0x25:
        SingleState = 1;
        break;
    }
    IrReceiver.resume(); // resume receiver
  }
  if(PowerState || SoundaddState || SoundresState || SoundoffState || SingleState){
    IrReceiver.stop();
    if(PowerState){
      Serial.println(F("Power Button pressed, now sending"));
      Serial.println();
      IrSender.sendRaw(rawDataPower, sizeof(rawDataPower) / sizeof(rawDataPower[0]), NEC_KHZ); 
      delay(DELAY_BETWEEN_REPEAT); // Wait a bit between retransmissions
    }
    if(SoundaddState){
      Serial.println(F("Soundadd Button pressed, now sending"));
      Serial.println();
      IrSender.sendRaw(rawDataSoundadd, sizeof(rawDataSoundadd) / sizeof(rawDataSoundadd[0]), NEC_KHZ); 
      delay(DELAY_BETWEEN_REPEAT); // Wait a bit between retransmissions
    }
    if(SoundresState){
      Serial.println(F("Soundres Button pressed, now sending"));
      Serial.println();
      IrSender.sendRaw(rawDataSoundres, sizeof(rawDataSoundres) / sizeof(rawDataSoundres[0]), NEC_KHZ); 
      delay(DELAY_BETWEEN_REPEAT); // Wait a bit between retransmissions
    }
    if(SoundoffState){
      Serial.println(F("Soundoff Button pressed, now sending"));
      Serial.println();
      IrSender.sendRaw(rawDataSoundoff, sizeof(rawDataSoundoff) / sizeof(rawDataSoundoff[0]), NEC_KHZ); 
      delay(DELAY_BETWEEN_REPEAT); // Wait a bit between retransmissions
    }
    if(SingleState){
      Serial.println(F("Single Button pressed, now sending"));
      Serial.println();
     // IrSender.sendRaw(rawDataSingle, sizeof(rawDataSingle) / sizeof(rawDataSingle[0]), NEC_KHZ); 
      delay(DELAY_BETWEEN_REPEAT); // Wait a bit between retransmissions
    }

    IrReceiver.start();

  }
  
        
  
     
  
}

