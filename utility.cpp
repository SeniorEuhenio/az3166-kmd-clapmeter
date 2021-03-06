// Copyright (c) Microsoft. All rights reserved.
// Licensed under the MIT license. 

#include "HTS221Sensor.h"
#include "AzureIotHub.h"
#include "Arduino.h"
#include "parson.h"
#include "config.h"
#include "RGB_LED.h"
#include "sensor.h"
#include "utility.h"

DevI2C *i2c;
HTS221Sensor *sensor;
LPS22HBSensor *pressureSensor;
static RGB_LED rgbLed;
static int interval = INTERVAL;
static float humidity;
static float temperature;

int getInterval()
{
    return interval;
}

void ledOn(int red, int green, int blue) {
    rgbLed.turnOff();
    rgbLed.setColor(red, green, blue);
}

void ledOff() {
    rgbLed.turnOff();
}

void blinkLED(int red, int green, int blue, int len)
{
    rgbLed.turnOff();
    rgbLed.setColor(red, green, blue);
    delay(len);
    rgbLed.turnOff();
}

void blinkSendConfirmation()
{
    blinkLED(0, 0, RGB_LED_BRIGHTNESS);
}

void parseTwinMessage(DEVICE_TWIN_UPDATE_STATE updateState, const char *message)
{
    JSON_Value *root_value;
    root_value = json_parse_string(message);
    if (json_value_get_type(root_value) != JSONObject)
    {
        if (root_value != NULL)
        {
            json_value_free(root_value);
        }
        LogError("parse %s failed", message);
        return;
    }
    JSON_Object *root_object = json_value_get_object(root_value);

    double val = 0;
    if (updateState == DEVICE_TWIN_UPDATE_COMPLETE)
    {
        JSON_Object *desired_object = json_object_get_object(root_object, "desired");
        if (desired_object != NULL)
        {
            val = json_object_get_number(desired_object, "interval");
        }
    }
    else
    {
        val = json_object_get_number(root_object, "interval");
    }
    if (val > 500)
    {
        interval = (int)val;
        LogInfo(">>>Device twin updated: set interval to %d", interval);
    }
    json_value_free(root_value);
}

void SensorInit()
{
    i2c = new DevI2C(D14, D15);
    sensor = new HTS221Sensor(*i2c);
    sensor->init(NULL);

    pressureSensor = new LPS22HBSensor(*i2c);
    pressureSensor->init(NULL);

    humidity = -1;
    temperature = -1000;
}

float readPressure() {
    float value = -1.0;
    pressureSensor->getPressure(&value);
    return value;
}

float readTemperature()
{
    sensor->reset();

    float temperature = 0;
    sensor->getTemperature(&temperature);

    return temperature;
}

float readHumidity()
{
    sensor->reset();

    float humidity = 0;
    sensor->getHumidity(&humidity);

    return humidity;
}

bool readMessage(int messageId, char *payload)
{
    JSON_Value *root_value = json_value_init_object();
    JSON_Object *root_object = json_value_get_object(root_value);
    char *serialized_string = NULL;

    json_object_set_number(root_object, "messageId", messageId);

    float t = readTemperature();
    float h = readHumidity();
    bool temperatureAlert = false;
    if(t != temperature)
    {
        temperature = t;
        json_object_set_number(root_object, "temperature", temperature);
    }
    if(temperature > TEMPERATURE_ALERT)
    {
        temperatureAlert = true;
    }
    
    if(h != humidity)
    {
        humidity = h;
        json_object_set_number(root_object, "humidity", humidity);
    }
    serialized_string = json_serialize_to_string_pretty(root_value);

    snprintf(payload, MESSAGE_MAX_LEN, "%s", serialized_string);
    json_free_serialized_string(serialized_string);
    json_value_free(root_value);
    return temperatureAlert;
}

float calcGain16LE(char *buf, uint16_t len) {
    uint64_t sum = 0;
    uint16_t samples = 0;
    uint16_t idx = 0x44;
    len -= 0x44;
    while (idx < len) {
        int16_t partialValue = abs((int16_t)(buf[idx] + buf[idx + 1] << 8));
        if (partialValue > 0) {     
            sum += partialValue;
            samples++;
        }
        idx += 4; // monochannel scan
    }
    if (samples > 0) {
        float res = 20 * log10((sum/samples)/32767.0);
        return isnan(res) ? -20.1 : res;
    } else {
        return -21.0;
    }
}

float calcRMS16LE(char *buf, uint16_t len) {
    uint64_t sum = 0;
    uint16_t samples = 0;
    uint16_t idx = 0x44;
    len -= 0x44;
    while (idx < len) {
        uint16_t partialValue = (uint64_t)abs((int16_t)(buf[idx] + buf[idx + 1] << 8));
        if (partialValue > 0) {     
            sum += partialValue * partialValue;
            samples++;
        }
        idx += 4; // monochannel scan
    }
    if (samples > 0) {
        float res = sqrt(sum/samples);
        return isnan(res) ? 0.0 : res;
    } else {
        return 0.0;
    }
}

