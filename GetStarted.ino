// Copyright (c) Microsoft. All rights reserved.
// Licensed under the MIT license. 
// To get started please visit https://microsoft.github.io/azure-iot-developer-kit/docs/projects/connect-iot-hub?utm_source=ArduinoExtension&utm_medium=ReleaseNote&utm_campaign=VSCode
#include "AZ3166WiFi.h"
#include "AzureIotHub.h"
#include "DevKitMQTTClient.h"
#include "mbed.h"
#include "Arduino.h"
#include "AudioClassV2.h"

#include "config.h"
#include "utility.h"
#include "SystemTickCounter.h"


bool hasWifi = false;
bool allowComm = true;
uint64_t tick;
int lastButtonAState;
int buttonAState;
int lastButtonBState;
int buttonBState;

#define UPLOAD_INTERVAL 1000
#define SAMPLE_RATE 8000 // 8khz
#define WAVE_HEADER_LEN 45
#define BITS_PER_SAMPLE 16
#define BYTES_PER_SAMPLE BITS_PER_SAMPLE / 8
#define AUDIO_LEN_SECS 1
const int AUDIO_LEN_MSECS = AUDIO_LEN_SECS * 1000;
const int AUDIO_SIZE = AUDIO_LEN_SECS * (SAMPLE_RATE * BYTES_PER_SAMPLE) + WAVE_HEADER_LEN; // one sec more

typedef enum ClapMode {
  CLAPMODE_NOP = 0,
  CLAPMODE_SAMPLING,
  CLAPMODE_CALCING,
  CLAPMODE_UPLOADING
} CLAPMODE_TYPEDEF;

AudioClass& Audio = AudioClass::getInstance();
char *audioBuffer;
int audioInBuffer = 0;
int clapState = CLAPMODE_NOP;

bool allowRecord = false;

float maxGain = -20.0;
float maxRMS = 0;

#define DEVICE_NAME "CLAP_0x"
char clapDeviceId[16];

//////////////////////////////////////////////////////////////////////////////////////////////////////////
// Utilities
static void InitWifi()
{
  Screen.print(2, "Connecting...");
  
  if (WiFi.begin() == WL_CONNECTED)
  {
    IPAddress ip = WiFi.localIP();
    Screen.print(1, ip.get_address());
    hasWifi = true;
    Screen.print(2, "Booting... \r\n");
  }
  else
  {
    hasWifi = false;
    Screen.print(1, "No Wi-Fi\r\n ");
  }
}

static void SendConfirmationCallback(IOTHUB_CLIENT_CONFIRMATION_RESULT result)
{
  if (result == IOTHUB_CLIENT_CONFIRMATION_OK)
  {
    blinkSendConfirmation();
  }
}

static void MessageCallback(const char* payLoad, int size)
{
  blinkLED();
  Screen.print(1, payLoad, true);
}

static void DeviceTwinCallback(DEVICE_TWIN_UPDATE_STATE updateState, const unsigned char *payLoad, int size)
{
  char *temp = (char *)malloc(size + 1);
  if (temp == NULL)
  {
    return;
  }
  memcpy(temp, payLoad, size);
  temp[size] = '\0';
  parseTwinMessage(updateState, temp);
  free(temp);
}

static int  DeviceMethodCallback(const char *methodName, const unsigned char *payload, int size, unsigned char **response, int *response_size)
{
  LogInfo("Try to invoke method %s", methodName);
  const char *responseMessage = "\"Successfully invoke device method\"";
  int result = 200;

  if (strcmp(methodName, "start") == 0)
  {
    LogInfo("Start sending temperature and humidity data");
    allowComm = true;
  }
  else if (strcmp(methodName, "stop") == 0)
  {
    LogInfo("Stop sending temperature and humidity data");
    allowComm = false;
  }
  else
  {
    LogInfo("No method %s found", methodName);
    responseMessage = "\"No method found\"";
    result = 404;
  }

  *response_size = strlen(responseMessage) + 1;
  *response = (unsigned char *)strdup(responseMessage);

  return result;
}


void setState(int newState){
  if (newState != clapState) {
    clapState = newState;
    char buf[16];

    switch (clapState) {
      case CLAPMODE_NOP:
        snprintf(buf, 16, "> Idling...");
        ledOff();
        break;
      case CLAPMODE_SAMPLING:
        ledOn(RGB_LED_BRIGHTNESS, 0, 0);
        snprintf(buf, 16, "> Sampling...");
        break;        
      case CLAPMODE_CALCING:
        ledOn(RGB_LED_BRIGHTNESS, RGB_LED_BRIGHTNESS, 0);
        snprintf(buf, 16, "> Calculating");
        break;
      case CLAPMODE_UPLOADING:
        ledOn(0, 0, RGB_LED_BRIGHTNESS);
        snprintf(buf, 16, "> Uploading...");
        break;
      default:
        ledOn(RGB_LED_BRIGHTNESS, 0, RGB_LED_BRIGHTNESS);
        snprintf(buf, 16, "X-(");
        break;
    }
    Screen.print(3, buf);
  }
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////
// Arduino sketch
void setup()
{
  Screen.init();
  Screen.print(0, "KMD ClapMeter");
  Screen.print(2, "Initializing...");
  
  Screen.print(3, " > Serial");
  Serial.begin(115200);

  // Initialize the WiFi module
  Screen.print(3, " > WiFi");
  hasWifi = false;
  InitWifi();
  if (!hasWifi)
  {
    return;
  }

  LogTrace("HappyPathSetup", NULL);

  Screen.print(3, " > Sensors");
  SensorInit();

  Screen.print(3, " > IoT Hub");
  DevKitMQTTClient_SetOption(OPTION_MINI_SOLUTION_NAME, "DevKit-KmdClapMeter");
  DevKitMQTTClient_Init(true, true);

  DevKitMQTTClient_SetSendConfirmationCallback(SendConfirmationCallback);
  DevKitMQTTClient_SetMessageCallback(MessageCallback);
  DevKitMQTTClient_SetDeviceTwinCallback(DeviceTwinCallback);
  DevKitMQTTClient_SetDeviceMethodCallback(DeviceMethodCallback);

  snprintf(clapDeviceId, 50, "%s%lx", DEVICE_NAME, random());

  Screen.print(1, clapDeviceId);

  // Setup your local audio buffer
  audioBuffer = (char *)malloc(AUDIO_SIZE + 1);

  // initialize the button pin as a input
  pinMode(USER_BUTTON_A, INPUT);
  lastButtonAState = digitalRead(USER_BUTTON_A);
  pinMode(USER_BUTTON_B, INPUT);
  lastButtonBState = digitalRead(USER_BUTTON_B);

  pauseResumeRecording(false);

  tick = millis();
}

int record()
{
  memset(audioBuffer, 0x0, AUDIO_SIZE);

  // Re-config the audio data format
  Audio.format(SAMPLE_RATE, BITS_PER_SAMPLE);

  // Start to record audio data
  int status = Audio.startRecord(audioBuffer, AUDIO_SIZE);
  if (status != AUDIO_STATE_IDLE) {
    Serial.println("RecStartStatus="+ String(status));
  }

  uint32_t started = millis();
  // Check whether the audio record is completed.
  while (millis() - started < AUDIO_LEN_SECS * 1000)
  {
    status = Audio.getAudioState();
    
    if (status != AUDIO_STATE_RECORDING) {
      if (status != AUDIO_STATE_RECORDING_FINISH) {
        Serial.println("WhileRecStartStatus="+ String(status));
      }
      break;
    }
    delay(10);
  }
  Audio.stop();

  return Audio.getCurrentSize();
}

void resetLevel() {
  maxGain = -20.0;
  maxRMS = 0;
}

void processSoundMonitoring() {
  setState(CLAPMODE_SAMPLING);
  audioInBuffer = record();
  setState(CLAPMODE_NOP);
}

void processSoundCalculations(float* gain, float* rms) {
  setState(CLAPMODE_CALCING);

  const byte BUF_LEN = 50;
  char buf[BUF_LEN];

  *gain = calcGain16LE(audioBuffer, audioInBuffer);
  *rms = calcRMS16LE(audioBuffer, audioInBuffer);

  audioInBuffer = 0;

  maxGain = max(*gain, maxGain);
  maxRMS = max(*rms, maxRMS);
  snprintf(buf, BUF_LEN, "%.2fdB/%.0f", maxGain, maxRMS);
  Screen.print(2, buf);
}

bool sendDeviceToCloudMessage(float gain, float rms) {
  char messagePayload[MESSAGE_MAX_LEN];

  EVENT_INSTANCE* message = DevKitMQTTClient_Event_Generate(messagePayload, MESSAGE);

  DevKitMQTTClient_Event_AddProp(message, "DID", clapDeviceId);

  const byte BUF_LEN = 50;
  char buf[BUF_LEN];

  snprintf(buf, BUF_LEN, "%.2f", gain);
  DevKitMQTTClient_Event_AddProp(message, "SV", buf);

  snprintf(buf, BUF_LEN, "%.2f", rms);
  DevKitMQTTClient_Event_AddProp(message, "RMS", buf);

  snprintf(buf, BUF_LEN, "%.2f", readTemperature());   
  DevKitMQTTClient_Event_AddProp(message, "TV", buf);

  snprintf(buf, BUF_LEN, "%.2f", readHumidity());
  DevKitMQTTClient_Event_AddProp(message, "HV", buf);

  snprintf(buf, BUF_LEN, "%.2f", readPressure());
  DevKitMQTTClient_Event_AddProp(message, "PV", buf);

    time_t t = time(NULL);
    struct tm *tm_info = gmtime(&t);
    char dts[26];
    strftime(dts, 26, "%Y-%m-%d %H:%M:%S", tm_info);

  DevKitMQTTClient_Event_AddProp(message, "DTS", dts);

  setState(CLAPMODE_UPLOADING);

  if (DevKitMQTTClient_SendEventInstance(message)) {
    LogTrace("CLAP", "EventSent OK");
    return true;
  } else {
    LogTrace("CLAP", "EventSend failed");
    return false;
  } 
}

void pauseResumeRecording(bool change) {
  if (change) {
    allowRecord = !allowRecord;
  }

  if (!allowRecord) {
      Screen.print(2, ">> PAUSED <<");
      Screen.print(3, "A=Meter B=Comm");
  } else {
      resetLevel();
      Screen.print(2, "Levels Reset");
      Screen.print(3, "(wait 3 secs)");
      delay(3000);  
    }
}

void pauseResumeUploading(bool change) {
  if (change) {
    allowComm = !allowComm;
  }

  if (allowComm) {
    Screen.print(2, "Upload resumed");
  } else {
    Screen.print(2, "Upload suspended");
  }
  Screen.print(3, "(wait 1 sec)");
  delay(1000);
}

void loop()
{
  buttonAState = digitalRead(USER_BUTTON_A);
  buttonBState = digitalRead(USER_BUTTON_B);

  if (buttonBState == LOW && lastButtonBState == HIGH) {
    pauseResumeUploading(true);
  } else if (buttonAState == LOW && lastButtonAState == HIGH) {
    pauseResumeRecording(true);
  } 
  else {
    if (allowRecord) {
      processSoundMonitoring();
    } else {
      pauseResumeRecording(false); // refresh screen
    }

    if (hasWifi)
    {
      if (audioInBuffer) 
      {
        float gain, rms;

        processSoundCalculations(&gain, &rms);

        if (allowComm)
        {
          if (!sendDeviceToCloudMessage(gain, rms)) {
            Serial.println("ERR: Upload failed");
          }

          tick = millis();
        }
      }
    }
    setState(CLAPMODE_NOP);
  }

  DevKitMQTTClient_Check();

  if (!allowRecord && !allowComm) {
    delay(50);
  }

  lastButtonAState = buttonAState;
  lastButtonBState = buttonBState;
}
