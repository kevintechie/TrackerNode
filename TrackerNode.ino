/*
   Copyright (c) 2016 Kevin Coleman

   Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation
   files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy,
   modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the
   Software is furnished to do so, subject to the following conditions:

   The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

   THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
   OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS
   BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT
   OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#include <RFM69.h>
#include <SPI.h>
#include <TinyGPS++.h>
#include <SoftwareSerial.h>
#include <Adafruit_GFX.h>
#include <Adafruit_PCD8544.h>
#include <TimeLib.h>

#define NODEID      2
#define NETWORKID   100
#define GATEWAYID   1
#define FREQUENCY   RF69_915MHZ
#define KEY         "ChangeMeChangeMe"
#define IS_RFM69HCW true
#define LED         8
#define SERIAL_BAUD 115200
#define GPS_BAUD    9600
#define RFM69_CS    10
#define RFM69_IRQ   2
#define RFM69_IRQN  0  // Pin 2 is IRQ 0!
#define RFM69_RST   9
#define SPEAKER     A1

int TRANSMITPERIOD = 3000; //transmit a packet to gateway so often (in ms)
byte sendSize = 0;
int oldSeconds = 99;

enum PktType {
  GPS,
  MSG,
  BEEP
};

enum BeepType {
  normal,
  emergency
};

typedef struct {
  PktType       pktType;
  int           nodeId; //store this nodeId
  float         latitude;
  float         longitude;
  unsigned long gpsDate;
  unsigned long gpsTime;
} GpsStruct;
GpsStruct gpsStruct;

typedef struct {
  PktType pktType;
  int     nodeId;
  char  msg[];
} MsgStruct;
MsgStruct msgStruct;

typedef struct {
  PktType  pktType;
  BeepType beepType;
} BeepStruct;
BeepStruct beepStruct;

static const int RxPin = 3, TxPin = 4;

SoftwareSerial gpsSerial(RxPin, TxPin);

TinyGPSPlus gps;

RFM69 radio = RFM69(RFM69_CS, RFM69_IRQ, IS_RFM69HCW, RFM69_IRQN);

Adafruit_PCD8544 display = Adafruit_PCD8544(5, 7, 6);

void setup() {
  pinMode(SPEAKER, OUTPUT);
  SPI.usingInterrupt(RFM69_IRQN);
  while (!Serial); // wait until serial console is open, remove if not tethered to computer
  Serial.begin(SERIAL_BAUD);

  gpsSerial.begin(GPS_BAUD);

  display.begin();
  display.setContrast(50);
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(BLACK);
  display.setCursor(0, 0);

  Serial.println("TrackerNode");

  // Hard Reset the RFM module
  pinMode(RFM69_RST, OUTPUT);
  digitalWrite(RFM69_RST, HIGH);
  delay(100);
  digitalWrite(RFM69_RST, LOW);
  delay(100);

  radio.initialize(FREQUENCY, NODEID, NETWORKID);
  radio.setHighPower();
  radio.setPowerLevel(31);
  radio.encrypt(KEY);
  char buff[50];
  sprintf(buff, "\nTransmitting at %d Mhz...", FREQUENCY == RF69_433MHZ ? 433 : FREQUENCY == RF69_868MHZ ? 868 : 915);
  Serial.println(buff);

  // tone(SPEAKER, 2800, 100);
}

long lastPeriod = -1;
void loop() {
  //process any serial input
  if (Serial.available() > 0) {
    char input = Serial.read();
    if (input >= 48 && input <= 57) { //[0,9]
      TRANSMITPERIOD = 1000 * (input - 48);
      if (TRANSMITPERIOD == 0) TRANSMITPERIOD = 10000;
      Serial.print("\nChanging delay to ");
      Serial.print(TRANSMITPERIOD);
      Serial.println("ms\n");
    }
  }

  //check for any received packets
  if (radio.receiveDone())
  {
    Serial.print('['); Serial.print(radio.SENDERID, DEC); Serial.print("] ");
    Serial.print("  [RX_RSSI:"); Serial.print(radio.RSSI); Serial.print("]");
    for (byte i = 0; i < radio.DATALEN; i++) {
      Serial.print((char)radio.DATA[i]);
    }

    if (radio.ACKRequested())
    {
      radio.sendACK();
      Serial.print(" - ACK sent");
    }
    if (radio.DATA[0] == 2) {
      blink(LED, 200);
      /*
        tone(SPEAKER, 2800, 200);
        delay(200);
        tone(SPEAKER, 2500, 200);
        delay(200);
      */
    } else {
      blink(LED, 5);
      Serial.println();
    }
  }

  while (gpsSerial.available() > 0) {
    if (gps.encode(gpsSerial.read())) {
      unsigned long age;
      int Year;
      byte Month, Day, Hour, Minute, Second;
      Year = gps.date.year();
      Month = gps.date.month();
      Day = gps.date.day();
      Hour = gps.time.hour();
      Minute = gps.time.minute();
      Second = gps.time.second();
      age = gps.time.age();
      if (age < 500) {
        // set the Time to the latest GPS reading
        setTime(Hour, Minute, Second, Day, Month, Year);
        //        adjustTime(offset * SECS_PER_HOUR);;
      }
    }
  }


  display.clearDisplay();
  gps.encode(gpsSerial.read());
  display.setCursor(0, 0);

  if (gps.location.isValid() && gps.location.age() < 5000 ) {
    display.println(gps.location.lat(), 6);
    display.println(gps.location.lng(), 6);
  } else {
    display.println(("NO GPS LOCK"));
  }

  drawGpsTime();
  gps.encode(gpsSerial.read());

  int currPeriod = millis() / TRANSMITPERIOD;
  if (currPeriod != lastPeriod) {
    //fill in the struct with new values
    gpsStruct.pktType = GPS;
    gpsStruct.nodeId = NODEID;
    gpsStruct.latitude = gps.location.lat();
    gpsStruct.longitude = gps.location.lng();
    gpsStruct.gpsDate = gps.date.value();
    gpsStruct.gpsTime = gps.time.value();

    Serial.print("Sending GpsStruct (");
    Serial.print(sizeof(gpsStruct));
    Serial.print(" bytes) ... ");
    if (radio.sendWithRetry(GATEWAYID, (const void*)(&gpsStruct), sizeof(gpsStruct)))
      Serial.print(" ok!");
    else Serial.print(" nothing...");
    Serial.println();
    blink(LED, 3);
    lastPeriod = currPeriod;
  }
  gps.encode(gpsSerial.read());
}

void blink(byte PIN, int delayMs) {
  pinMode(PIN, OUTPUT);
  digitalWrite(PIN, HIGH);
  gps.encode(gpsSerial.read());
  delay(delayMs);
  gps.encode(gpsSerial.read());
  digitalWrite(PIN, LOW);
}

void drawGpsTime() {
  gps.encode(gpsSerial.read());
  if (oldSeconds != second()) {
    oldSeconds = second();
    if (gps.time.isValid()) {
      display.setCursor(0, 16);
      display.print("        ");
      display.setCursor(0, 16);
      display.print(hour());
      printDigits(minute());
      printDigits(second());
    }
    if (gps.date.isValid()) {
      display.setCursor(0, 24);
      display.print("          ");
      display.setCursor(0, 24);
      display.print(day());
      display.print("-");
      display.print(month());
      display.print("-");
      display.print(year());
    }
    display.display();
    gps.encode(gpsSerial.read());
  }
}

void printDigits(int digits) {
  // utility function for digital clock display: prints preceding colon and leading 0
  display.print(":");
  if (digits < 10)
    display.print('0');
  display.print(digits);
  gps.encode(gpsSerial.read());
}

