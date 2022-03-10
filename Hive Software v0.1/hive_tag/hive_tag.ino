/*
 * Copyright (c) 2015 by Thomas Trojer <thomas@trojer.net>
 * Decawave DW1000 library for arduino.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 * @file RangingTag.ino
 * Use this to test two-way ranging functionality with two DW1000. This is
 * the tag component's code which polls for range computation. Addressing and
 * frame filtering is currently done in a custom way, as no MAC features are
 * implemented yet.
 *
 * Complements the "RangingAnchor" example sketch.
 *
 * @todo
 *  - use enum instead of define
 *  - move strings to flash (less RAM consumption)
 */

///////////////////////////////////////////////////////////////////////////////
// LIBRARIES

#include <SPI.h>
//#include <FastLED.h>
#include <DW1000.h>

///////////////////////////////////////////////////////////////////////////////
// I/O PINS

//#define PIN_LED 7 // WS2812B LED data pin
#define PIN_LED 2 // PCB v0.1


//const uint8_t PIN_RST = 8; // reset pin
//const uint8_t PIN_IRQ = 9; // irq pin
//const uint8_t PIN_SS = 10; // spi select pin

// for PCB v0.1
const uint8_t PIN_RST = 9; // reset pin
const uint8_t PIN_IRQ = 8; // irq pin
const uint8_t PIN_SS = 10; // spi select pin

///////////////////////////////////////////////////////////////////////////////
// LAMP AND TAG CONSTRUCT VALUES

#define NUM_LEDS 1
#define CHIPSET WS2812B
#define COLOR_ORDER GRB

struct TagVars {
    byte tagID;
    byte rangeOfActivation;
    byte brightness;
    byte redChannel;
    byte greenChannel;
    byte blueChannel;
    byte fadeOnRangeOnly;
    byte cutOffMaxBrightness;
    byte cutOffMinBrightness;
    short turnOnDelay;
    short shutOffDelay;
    short fadeInRate;
    short fadeOutRate;
};

struct TagControl {
    byte authLevel;
    byte collisionMode;
};

enum tagAuthLevels {
    ENV_MASTER,
    ADMIN,
    GENERAL
};

enum collisionHeuristic {
    PREEMINENCE,
    BLENDING,
    FIRST_COME_FIRST_SERVE
};

enum LEDModes {
    BRIGHTER_WHEN_CLOSER,
    DIMMER_WHEN_CLOSER,
    CONSTANT_BRIGHTNESS_WITHIN_RANGE,
    CONSTANT_BRIGHTNESS_OUTSIDE_RANGE,
    N_CLOSEST_LIGHTS,
    N_FARTHEST_LIGHTS
};

byte LEDBrightnessTable[256] = {
    0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
    1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
    1, 2, 2, 2, 2, 2, 2, 2, 2, 3, 3, 3, 3, 3, 4, 4,
    4, 4, 4, 5, 5, 5, 5, 6, 6, 6, 6, 7, 7, 7, 7, 8,
    8, 8, 9, 9, 9, 10, 10, 10, 11, 11, 12, 12, 12, 13, 13, 14,
    14, 15, 15, 15, 16, 16, 17, 17, 18, 18, 19, 19, 20, 20, 21, 22,
    22, 23, 23, 24, 25, 25, 26, 26, 27, 28, 28, 29, 30, 30, 31, 32,
    33, 33, 34, 35, 36, 36, 37, 38, 39, 40, 40, 41, 42, 43, 44, 45,
    46, 46, 47, 48, 49, 50, 51, 52, 53, 54, 55, 56, 57, 58, 59, 60,
    61, 62, 63, 64, 65, 67, 68, 69, 70, 71, 72, 73, 75, 76, 77, 78,
    80, 81, 82, 83, 85, 86, 87, 89, 90, 91, 93, 94, 95, 97, 98, 99,
    101, 102, 104, 105, 107, 108, 110, 111, 113, 114, 116, 117, 119, 121, 122, 124,
    125, 127, 129, 130, 132, 134, 135, 137, 139, 141, 142, 144, 146, 148, 150, 151,
    153, 155, 157, 159, 161, 163, 165, 166, 168, 170, 172, 174, 176, 178, 180, 182,
    184, 186, 189, 191, 193, 195, 197, 199, 201, 204, 206, 208, 210, 212, 215, 217,
    219, 221, 224, 226, 228, 231, 233, 235, 238, 240, 243, 245, 248, 250, 253, 255};

struct TagVars lightSettings = {5, 100, 256, 256, 256, 1, 0, 100, 0, 0, 0, 0};
struct TagControl tagCommands = {ENV_MASTER, PREEMINENCE};

///////////////////////////////////////////////////////////////////////////////
// DWM1000 UWB MODULE VARIABLES

enum UWBComCodes {
    POLL,
    POLL_ACK,
    RANGE,
    RANGE_REPORT,
    UPDATE_LIGHT_STATE,
    UPDATE_TAG_CONTROL_STATE,
    RANGE_FAILED
};

// message flow state
volatile byte expectedMsgId = POLL_ACK;
// message sent/received state
volatile boolean sentAck = false;
volatile boolean receivedAck = false;
// timestamps to remember
DW1000Time timePollSent;
DW1000Time timePollAckReceived;
DW1000Time timeRangeSent;
// data buffer
#define LEN_DATA 20
byte data[LEN_DATA];
// watchdog and reset period
uint32_t lastActivity;
uint32_t resetPeriod = 250;
// reply times (same on both sides for symm. ranging)
uint16_t replyDelayTimeUS = 3000;

///////////////////////////////////////////////////////////////////////////////
// CONTROL INTERFACE LOGIC


///////////////////////////////////////////////////////////////////////////////
// SSD1306 DISPLAY LOGIC

///////////////////////////////////////////////////////////////////////////////
// DWM1000 UWB COMMUNICATION LOGIC

// update activity timestamp, so that we do not reach "resetPeriod"
void noteActivity() {
    lastActivity = millis();
}

void resetInactive() {
    // tag sends POLL and listens for POLL_ACK
    expectedMsgId = POLL_ACK;
    transmitPoll();
    noteActivity();
}

// Handler function for when a communication is successfully sent by DWM1000 module.
void handleSent() {
    // status change on sent success
    sentAck = true;
}

// Handler function for when a communication is successfully received by DWM1000 module.
void handleReceived() {
    receivedAck = true;
}

// Send polling message out into the air to see if any anchors/bulbs are in range to
// reply and start a conversation.
void transmitPoll() {
    DW1000.newTransmit();
    DW1000.setDefaults();
    data[0] = POLL;
    DW1000.setData(data, LEN_DATA);
    DW1000.startTransmit();
//    Serial.println(F("Transmit Routine Started"));
}

// Send four things:  Tag ID, timePollSent, timePollAckReceived, timeRangeSent
void transmitRange() {
    DW1000.newTransmit();
    DW1000.setDefaults();
    data[0] = RANGE;
    // delay sending the message for the sake of precision and remember expected future sent timestamp
    DW1000Time deltaTime = DW1000Time(replyDelayTimeUS, DW1000Time::MICROSECONDS);
    timeRangeSent = DW1000.setDelay(deltaTime);
    timePollSent.getTimestamp(data + 1);
    timePollAckReceived.getTimestamp(data + 6);
    timeRangeSent.getTimestamp(data + 11);
    DW1000.setData(data, LEN_DATA);
    DW1000.startTransmit();
    Serial.print("Expect RANGE to be sent @ "); Serial.println(timeRangeSent.getAsFloat());
}

void transmitLightState() {
    DW1000.newTransmit();
    DW1000.setDefaults();
    data[0] = UPDATE_LIGHT_STATE;

    DW1000.setData(data, LEN_DATA);
    DW1000.startTransmit();
}

// Send:  
void transmitTagControlState() {
    DW1000.newTransmit();
    DW1000.setDefaults();
    data[0] = UPDATE_TAG_CONTROL_STATE;

    DW1000.setData(data, LEN_DATA);
    DW1000.startTransmit();
}

void receiver() {
    DW1000.newReceive();
    DW1000.setDefaults();
    // so we don't need to restart the receiver manually
    DW1000.receivePermanently(true);
    DW1000.startReceive();
//    Serial.println(F("Receiver Routine Started"));
}

///////////////////////////////////////////////////////////////////////////////
// SETUP EXECUTION ROUTINE

void initializeSerialOutput() {
    Serial.begin(115200);
    delay(8000);
    Serial.println(F("RUN ### DW1000-arduino-ranging-tag ###"));
//    delay(14000);
}

void initializeUWBModule() {
    Serial.println(F("### DW1000-arduino-ranging-tag ###"));
    // initialize the driver
    DW1000.begin(PIN_IRQ, PIN_RST);
    DW1000.select(PIN_SS);
    Serial.println("DW1000 initialized ...");
    // general configuration
    DW1000.newConfiguration();
    DW1000.setDefaults();
    DW1000.setDeviceAddress(4);
    DW1000.setNetworkId(10);
    DW1000.enableMode(DW1000.MODE_LONGDATA_RANGE_LOWPOWER);
    DW1000.commitConfiguration();
    Serial.println(F("Committed configuration ..."));
    // DEBUG chip info and registers pretty printed
    char msg[128];
    DW1000.getPrintableDeviceIdentifier(msg);
    Serial.print("Device ID: "); Serial.println(msg);
    DW1000.getPrintableExtendedUniqueIdentifier(msg);
    Serial.print("Unique ID: "); Serial.println(msg);
    DW1000.getPrintableNetworkIdAndShortAddress(msg);
    Serial.print("Network ID & Device Address: "); Serial.println(msg);
    DW1000.getPrintableDeviceMode(msg);
    Serial.print("Device mode: "); Serial.println(msg);
    // attach callback for (successfully) sent and received messages
    DW1000.attachSentHandler(handleSent);
    DW1000.attachReceivedHandler(handleReceived);
    // anchor starts by transmitting a POLL message
    receiver();
    transmitPoll();
    noteActivity();
}

void setup() {
    initializeSerialOutput();
//    initializeUWBModule();
}

///////////////////////////////////////////////////////////////////////////////
// LOOPING EXECUTION ROUTINE

void loop() {
//    Serial.println("E");
//    if (!sentAck && !receivedAck) {
//        // check if inactive
//        if (millis() - lastActivity > resetPeriod) {
//            resetInactive();
//        }
//        return;
//    }
//
//    Serial.print("\tSent Ack: ");
//    Serial.print(sentAck);
//    Serial.print("\tReceived Ack: ");
//    Serial.print(receivedAck);
//    Serial.println();
//    
//    // continue on any success confirmation
//    if (sentAck) {
//        sentAck = false;
//        byte msgId = data[0];
//        if (msgId == POLL) {
//            DW1000.getTransmitTimestamp(timePollSent);
//            Serial.print("Sent POLL @ "); Serial.println(timePollSent.getAsFloat());
//        } else if (msgId == RANGE) {
//            DW1000.getTransmitTimestamp(timeRangeSent);
//            noteActivity();
//        }
//        else if (msgId == UPDATE_LIGHT_STATE) {
//            transmitLightState();
//        }
//        else if (msgId == UPDATE_TAG_CONTROL_STATE) {
//            transmitTagControlState();
//        }
//    }
//    if (receivedAck) {
//        receivedAck = false;
//        // get message and parse
//        DW1000.getData(data, LEN_DATA);
//        byte msgId = data[0];
//        if (msgId != expectedMsgId) {
//            // unexpected message, start over again beginning with poll
//            Serial.print("Received wrong message # "); Serial.println(msgId);
//            expectedMsgId = POLL_ACK;
//            transmitPoll();
//            return;
//        }
//        if (msgId == POLL_ACK) {
//            DW1000.getReceiveTimestamp(timePollAckReceived);
//            expectedMsgId = RANGE_REPORT;
//            transmitRange();
//            noteActivity();
//        } else if (msgId == RANGE_REPORT) {
//            expectedMsgId = POLL_ACK;
//            float curRange;
//            memcpy(&curRange, data + 1, 4);
//            transmitPoll();
//            noteActivity();
//        } else if (msgId == RANGE_FAILED) {
//            expectedMsgId = POLL_ACK;
//            transmitPoll();
//            noteActivity();
//        }
//    }
}
