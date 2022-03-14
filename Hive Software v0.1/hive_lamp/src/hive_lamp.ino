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
 * @file RangingAnchor.ino
 * Use this to test two-way ranging functionality with two
 * DW1000. This is the anchor component's code which computes range after
 * exchanging some messages. Addressing and frame filtering is currently done
 * in a custom way, as no MAC features are implemented yet.
 *
 * Complements the "RangingTag" example sketch.
 *
 * @todo
 *  - weighted average of ranging results based on signal quality
 *  - use enum instead of define
 *  - move strings to flash (less RAM consumption)
 */

///////////////////////////////////////////////////////////////////////////////
// LIBRARIES

#include <SPI.h>
//#include <FastLED.h>
#include <Adafruit_NeoPixel.h>
#include <DW1000.h>

///////////////////////////////////////////////////////////////////////////////
// I/O PINS

//#define LED_PIN 7 // WS2812B LED data pin
#define LED_PIN 2 // PCB v0.1


//const uint8_t PIN_RST = 8; // reset pin
//const uint8_t PIN_IRQ = 9; // irq pin
//const uint8_t PIN_SS = 10; // spi select pin

// for PCB v0.1
const uint8_t PIN_RST = 9; // reset pin
const uint8_t PIN_IRQ = 8; // irq pin
const uint8_t PIN_SS = 10; // spi select pin

///////////////////////////////////////////////////////////////////////////////
// LAMP AND TAG CONSTRUCT VALUES

#define NUM_LEDS 16
#define CHIPSET WS2812B
#define COLOR_ORDER GRB

struct bulbProps {
    int bulbID;
    int fixtureBrightness;
    int brightnessCurve[256];
};

struct lightConfig {
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

struct tagControl {
    byte tagID;
    byte authLevel;
    byte lightMode;
    byte collisionMode;
};

struct rangePacket {
    byte tagID;
    
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

Adafruit_NeoPixel pixels(NUM_LEDS, LED_PIN, NEO_GRB + NEO_KHZ800);
//CRGB leds[NUM_LEDS];
struct lightConfig lightSettings = {5, 100, 255, 255, 255, 1, 0, 100, 0, 0, 0, 0};
struct tagControl tagsInRange[5];
float distance;

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
volatile byte expectedMsgId = POLL;
// message sent/received state
volatile boolean sentAck = false;
volatile boolean receivedAck = false;
// protocol error state
boolean protocolFailed = false; // may be read externally
// timestamps to remember
DW1000Time timePollSent;
DW1000Time timePollReceived;
DW1000Time timePollAckSent;
DW1000Time timePollAckReceived;
DW1000Time timeRangeSent;
DW1000Time timeRangeReceived;
// last computed range/time
DW1000Time timeComputedRange;
// data buffer
#define LEN_DATA 20
byte data[LEN_DATA];
// watchdog and reset period (in milliseconds)
uint32_t lastActivity;
uint32_t resetPeriod = 250;
// reply times (same on both sides for symm. ranging)
uint16_t replyDelayTimeUS = 3000;
// ranging counter (per second)
uint16_t successRangingCount = 0;
uint32_t rangingCountPeriod = 0;
float samplingRate = 0;

///////////////////////////////////////////////////////////////////////////////
// DWM1000 UWB COMMUNICATION LOGIC

// update activity timestamp, so that we do not reach "resetPeriod"
void noteActivity() {
    lastActivity = millis();
}

void resetInactive() {
    // anchor listens for POLL
    expectedMsgId = POLL;
    receiver();
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

void transmitPollAck() {
    DW1000.newTransmit();
    DW1000.setDefaults();
    data[0] = POLL_ACK;
    // delay the same amount as ranging tag
    DW1000Time deltaTime = DW1000Time(replyDelayTimeUS, DW1000Time::MICROSECONDS);
    DW1000.setDelay(deltaTime);
    DW1000.setData(data, LEN_DATA);
    DW1000.startTransmit();
}

void transmitRangeReport(float curRange) {
    DW1000.newTransmit();
    DW1000.setDefaults();
    data[0] = RANGE_REPORT;
    // write final ranging result
    memcpy(data + 1, &curRange, 4);
    DW1000.setData(data, LEN_DATA);
    DW1000.startTransmit();
}

void transmitRangeFailed() {
    DW1000.newTransmit();
    DW1000.setDefaults();
    data[0] = RANGE_FAILED;
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
// DWM1000 UWB COMPUTATIONS

/*
 * RANGING ALGORITHMS
 * ------------------
 * Either of the below functions can be used for range computation (see line "CHOSEN
 * RANGING ALGORITHM" in the code).
 * - Asymmetric is more computation intense but least error prone
 * - Symmetric is less computation intense but more error prone to clock drifts
 *
 * The anchors and tags of this reference example use the same reply delay times, hence
 * are capable of symmetric ranging (and of asymmetric ranging anyway).
 */

void computeRangeAsymmetric() {
    // asymmetric two-way ranging (more computation intense, less error prone)
    DW1000Time round1 = (timePollAckReceived - timePollSent).wrap();
    DW1000Time reply1 = (timePollAckSent - timePollReceived).wrap();
    DW1000Time round2 = (timeRangeReceived - timePollAckSent).wrap();
    DW1000Time reply2 = (timeRangeSent - timePollAckReceived).wrap();
    DW1000Time tof = (round1 * round2 - reply1 * reply2) / (round1 + round2 + reply1 + reply2);
    // set tof timestamp
    timeComputedRange.setTimestamp(tof);
}

void computeRangeSymmetric() {
    // symmetric two-way ranging (less computation intense, more error prone on clock drift)
    DW1000Time tof = ((timePollAckReceived - timePollSent) - (timePollAckSent - timePollReceived) +
                      (timeRangeReceived - timePollAckSent) - (timeRangeSent - timePollAckReceived)) * 0.25f;
    // set tof timestamp
    timeComputedRange.setTimestamp(tof);
}

/*
 * END RANGING ALGORITHMS
 * ----------------------
 */

///////////////////////////////////////////////////////////////////////////////
// LED STATE CHANGE

// What kind of behavioral mode do we want the lights to be in?
// VARIABLES:
// - Range of activation                 - meters or centimeters
// - Brightness (of each color channel)  - percent
// - Time delay for shutoff              - seconds or milliseconds
// - Time delay for turn on              - seconds or milliseconds
// - Fade-in rate                        - seconds or milliseconds
// - Fade-out rate                       - seconds or milliseconds
// - Fade on range only                  - boolean
// - Set color                           - set list of colors or RGB plot
// - Specific brightness of each fixture - lumens
// - Cutoff max brightness               - percent or lumens
// - Cutoff min brightness               - percent or lumens
//
// MODES:
// 1 - Brighter when closer
// 2 - Dimmer when closer
// 3 - Constant brightness within range threshold
// 4 - Constant brightness outside of range threshold
// 5 - X closest lights
// 6 - X farthest lights

void toggleLEDMode(int ledModes, int setTo) {
//    switch(ledMode) {
//        case -1:
//          if 
//    }
}

// How do we determine how the light changes in correspondence to its defined
// behavior?
int calculateLEDChange() {
    return 0;
}

// In acknowledgement of the character of the hardware (LEDs) we are using,
// what do we need to keep in mind?
//
// WS2812B brightness curves


///////////////////////////////////////////////////////////////////////////////
// SETUP EXECUTION ROUTINE

void initializeSerialOutput() {
    Serial.begin(9600);
    delay(8000);
}

void initializeLEDModule() {
//    FastLED.addLeds<CHIPSET, PIN_LED, COLOR_ORDER>(leds, NUM_LEDS);
    pixels.begin();
    pixels.clear();
    pixels.setPixelColor(0, pixels.Color(0, 15, 15));
    pixels.show();
    pixels.setPixelColor(1, pixels.Color(0, 15, 15));
    pixels.show();
    pixels.setPixelColor(2, pixels.Color(0, 15, 15));
    pixels.show();
    pixels.setPixelColor(3, pixels.Color(0, 15, 15));
    pixels.show();
    pixels.setPixelColor(4, pixels.Color(0, 15, 15));
    pixels.show();
    pixels.setPixelColor(5, pixels.Color(0, 15, 15));
    pixels.show();
    pixels.setPixelColor(6, pixels.Color(0, 15, 15));
    pixels.show();
    pixels.setPixelColor(7, pixels.Color(0, 15, 15));
    pixels.show();
    pixels.setPixelColor(8, pixels.Color(0, 15, 15));
    pixels.show();
    pixels.setPixelColor(9, pixels.Color(0, 15, 15));
    pixels.show();
    pixels.setPixelColor(10, pixels.Color(0, 15, 15));
    pixels.show();
    pixels.setPixelColor(11, pixels.Color(0, 15, 15));
    pixels.show();
}

void initializeUWBModule() {
    // initialize ultra-wideband hardware (DWM1000)
    Serial.println(F("### DW1000-arduino-ranging-anchor ###"));
    // initialize the driver
    DW1000.begin(PIN_IRQ, PIN_RST);
    DW1000.select(PIN_SS);
    Serial.println(F("DW1000 initialized ..."));
    // general configuration
    DW1000.newConfiguration();
    DW1000.setDefaults();
    DW1000.setDeviceAddress(3);
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
    // anchor starts in receiving mode, awaiting a ranging poll message
    receiver();
    noteActivity();
    // for first time ranging frequency computation
    rangingCountPeriod = millis();
}

void setup() {
    initializeSerialOutput();
    initializeLEDModule();
    initializeUWBModule();
}

///////////////////////////////////////////////////////////////////////////////
// LOOPING EXECUTION ROUTINE


void uponPollMsg() {
    // on POLL we (re-)start, so no protocol failure
//    Serial.println("POLL");
    protocolFailed = false;
    DW1000.getReceiveTimestamp(timePollReceived);
    expectedMsgId = RANGE;
    transmitPollAck();
    noteActivity();    
}

// Upon receiving timing info required for range calculation, execute range
// calculation and update range calculation.  Get 
void uponRangeMsg(int32_t curMillis) {
    DW1000.getReceiveTimestamp(timeRangeReceived);
    expectedMsgId = POLL;
    if (!protocolFailed) {
        timePollSent.setTimestamp(data + 1);
        timePollAckReceived.setTimestamp(data + 6);
        timeRangeSent.setTimestamp(data + 11);
        // (re-)compute range as two-way ranging is done
        computeRangeAsymmetric(); // CHOSEN RANGING ALGORITHM
        transmitRangeReport(timeComputedRange.getAsMicroSeconds());
        distance = timeComputedRange.getAsMeters();
        Serial.print("Range: "); Serial.print(distance); Serial.print(" m");
//        leds[0] = CHSV(distance * 100, distance * 100, distance * 100); FastLED.show(); delay(30);
        pixels.setPixelColor(0, pixels.Color(distance * 100, distance * 100, distance * 100));
//        pixels.clear();
//        pixels.setPixelColor(0, pixels.Color(0, 150, 150));
        pixels.show();
//                leds[0] = CRGB::Black; FastLED.show(); delay(30);
        Serial.print("\t RX power: "); Serial.print(DW1000.getReceivePower()); Serial.print(" dBm");
        Serial.print("\t Sampling: "); Serial.print(samplingRate); Serial.println(" Hz");
        //Serial.print("FP power is [dBm]: "); Serial.print(DW1000.getFirstPathPower());
        //Serial.print("RX power is [dBm]: "); Serial.println(DW1000.getReceivePower());
        //Serial.print("Receive quality: "); Serial.println(DW1000.getReceiveQuality());
        // update sampling rate (each second)
        successRangingCount++;
        if (curMillis - rangingCountPeriod > 1000) {
            samplingRate = (1000.0f * successRangingCount) / (curMillis - rangingCountPeriod);
            rangingCountPeriod = curMillis;
            successRangingCount = 0;
        }
    }
    else {
        transmitRangeFailed();
    }

    noteActivity();
}

void uponLightStateMsg() {
    return;
}

void uponTagControlStateMsg() {
    return;
}

int counter = 0;

void loop() {
    int32_t curMillis = millis();
    Serial.print("Cycle Start Time: ");
    Serial.println(curMillis);
    Serial.print("Counter: ");
    Serial.print(counter);
    Serial.print("\tSent Ack: ");
    Serial.print(sentAck);
    Serial.print("\tReceived Ack: ");
    Serial.print(receivedAck);
    Serial.println();
    
    // Appears to be for if there is no activity and the hardware times out,
    // then reset it so it (receiver mechanism) will listen for polling requests again.
    if (!sentAck && !receivedAck) {
        // check if inactive
        if (curMillis - lastActivity > resetPeriod) {
            resetInactive();
        }
        return;
    }
    
//    
//    // If message sent successfully, then get DW1000 precision timestamp
//    // for when it was sent.
//    Serial.println("E");
    if (sentAck) {
        counter++;
//        Serial.println("SENT");
        sentAck = false;
        byte msgId = data[0];
        if (msgId == POLL_ACK) {
            DW1000.getTransmitTimestamp(timePollAckSent);
            noteActivity();
        }
    }
//
//    // Upon receipt of message, 
    if (receivedAck) {
        counter++;
        receivedAck = false;
        // get incoming message and parse
        DW1000.getData(data, LEN_DATA);
//        Serial.print(" ");
//        Serial.println(data[0]);
        byte msgId = data[0];
        
        if (msgId != expectedMsgId) { //
            // unexpected message, start over again (except if already POLL)
            Serial.println("PROTOCOL FAILED");
            protocolFailed = true;
        }
        
        if (msgId == POLL) {
            Serial.println("POLL RESPONSE START");
            uponPollMsg();
            Serial.println("POLL RESPONSE FINISHED");
        }
        else if (msgId == RANGE) {
            Serial.println("RANGE RESPONSE START");
            uponRangeMsg(curMillis);
            Serial.println("RANGE RESPONSE FINISHED");
        }
        else if (msgId == UPDATE_LIGHT_STATE) {
            uponLightStateMsg();
        }
        else if (msgId == UPDATE_TAG_CONTROL_STATE) {
            uponTagControlStateMsg();
        }
    }
}
