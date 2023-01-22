#include <SPI.h>
#include <SD.h>

/*
 * Ryan Cooper
 * Santa Clara University
 * 
 * Date Published: 5/15/2016
 * Version Number: 2.6
 * File: LidarDataLogger.ino
 * 
 * Intended to be run on Arduino MEGA 2560 r3
  */


// change this to match your SD shield or module;
// Arduino Ethernet shield: pin 4
// Adafruit SD shields and modules: pin 10
// Sparkfun SD shield: pin 8
const int chipSelect = 53;
File dataFile;

void initializeDataLogger() {
//  Serial.print("Initializing SD card...");
  if (!SD.begin(chipSelect)) {
//    Serial.println("Card failed, or not present");
    return;
  }
//  Serial.println("card initialized.");
  int i = 0;
  String file(i);
  while(SD.exists(file)) {
    String temp(++i);
    file = temp;
  }
//  Serial.println(file);
  dataFile = SD.open(file, FILE_WRITE);
}

bool logData(String data) {
  if (dataFile) {
    dataFile.print(data);
//    Serial.println(data);
    return true;
  } else {
//    Serial.println("error during data logging");
    return false;
  }
}

void logFloat(float f, String descriptor) { 
  char c[10];
  dtostrf(f, 9, 4, c);
  String temp(c);
  descriptor += temp;
//  Serial.println(descriptor);
  logData(descriptor);
}


bool logData(char data) {
  if (dataFile) {
    dataFile.print(data);
//    Serial.println(data);
    return true;
  } else {
//    Serial.println("error during data logging");
    return false;
  }
}

void stopDataLogging() {
  if (dataFile) {
//    Serial.println("done logging");
    dataFile.close();
  } else {
//    Serial.println("Failed to log data");
    return false;
  }
}
