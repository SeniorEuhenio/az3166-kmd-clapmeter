// Copyright (c) Microsoft. All rights reserved.
// Licensed under the MIT license. 

#ifndef UTILITY_H
#define UTILITY_H

#define RGB_LED_BRIGHTNESS 32

void parseTwinMessage(DEVICE_TWIN_UPDATE_STATE, const char *);
bool readMessage(int, char *);

void SensorInit(void);

void blinkLED(int red = RGB_LED_BRIGHTNESS, int green = RGB_LED_BRIGHTNESS, int blue = RGB_LED_BRIGHTNESS, int len = 100);
void blinkSendConfirmation(void);
int getInterval(void);

float readPressure();
float readTemperature();
float readHumidity();

float calcLoudness16LE(char *buf, uint16_t len);
void ledOn(int red, int green, int blue);
void ledOff();

#endif /* UTILITY_H */