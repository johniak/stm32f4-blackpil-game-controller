#include <Arduino.h>
#include <ezButton.h>
#include "hid/usbd_hid.h"
#include "usbd_desc.h"
#include <EEPROM.h>

#define PIN_PITCH_ANALOG 1
#define PIN_ROLL_ANALOG 2
#define PIN_LED PC13
#define PIN_BUTTON PA0
#define MODE_CONTROLLER 0
#define MODE_SETTINGS 1
#define BLINK_INTERVAL 100

#define MIN_MAX_ADDR 0
struct MinMaxStruct {
  int minPitchValue;
  int maxPitchValue;
  int minRollValue;
  int maxRollValue;
};

int mode = MODE_CONTROLLER;
int old_mode = MODE_CONTROLLER;
ezButton button(PIN_BUTTON);
unsigned long previousBlinkMillis = 0;
int ledState = LOW;
int minPitchValue = 4096;
int maxPitchValue = 0;

int minRollValue = 4096;
int maxRollValue = 0;
USBD_HandleTypeDef hUsbDeviceHS;

uint8_t HID_Buffer[3];

void controllerLoop();
void settingsLoop();
void settingSave();
void blinkLed();

void getPointerData(uint8_t *pbuf, int8_t x, int8_t y)
{
  pbuf[0] = 0;
  pbuf[1] = x;
  pbuf[2] = y;
}
void moveAxis(int8_t x, int8_t y)
{
  getPointerData(HID_Buffer, x, y);
  USBD_HID_SendReport(&hUsbDeviceHS, HID_Buffer, 3);
}

void MX_USB_Device_Init(void)
{
  /* USER CODE BEGIN USB_Device_Init_PreTreatment */
  /* USER CODE END USB_Device_Init_PreTreatment */

  /* Init Device Library, add supported class and start the library. */
  if (USBD_Init(&hUsbDeviceHS, &USBD_Desc, 0) != USBD_OK)
  {
    Error_Handler();
  }
  if (USBD_RegisterClass(&hUsbDeviceHS, &USBD_HID) != USBD_OK)
  {
    Error_Handler();
  }
  if (USBD_Start(&hUsbDeviceHS) != USBD_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USB_Device_Init_PostTreatment */

  /* USER CODE END USB_Device_Init_PostTreatment */
}

void setup()
{
  pinMode(PIN_PITCH_ANALOG, INPUT_ANALOG);
  pinMode(PIN_LED, OUTPUT);
  Serial1.setRx(PB7);
  Serial1.setTx(PB6);
  Serial1.begin(9600);
  analogReadResolution(12);
  MX_USB_Device_Init();
  MinMaxStruct minMaxVar;
  EEPROM.get(MIN_MAX_ADDR, minMaxVar);
  minPitchValue = minMaxVar.minPitchValue;
  maxPitchValue = minMaxVar.maxPitchValue;
  minRollValue = minMaxVar.minRollValue;
  maxRollValue = minMaxVar.maxRollValue;
}  

void loop()
{
  button.loop();
  bool buttonClicked = button.isReleased();
  if (buttonClicked)
  {
    Serial1.println("Button Clicked");

    switch (mode)
    {
    case MODE_SETTINGS:
      mode = MODE_CONTROLLER;
      break;
    default:
      mode = MODE_SETTINGS;
      break;
    }
  }
  switch (mode)
  {
  case MODE_SETTINGS:
    settingsLoop();
    break;
  default:
    controllerLoop();
    break;
  }
  if (mode == MODE_CONTROLLER && old_mode == MODE_SETTINGS)
  {
    settingSave();
  }
  old_mode = mode;
}

void controllerLoop()
{
  int pitchCurrent = analogRead(PIN_PITCH_ANALOG);
  int rollCurrent = analogRead(PIN_ROLL_ANALOG);
  int pitchMaped = map(pitchCurrent, minPitchValue, maxPitchValue, -127, 127);
  int rollMaped = map(rollCurrent, minRollValue, maxRollValue, -127, 127);
  Serial.printf("pitchMaped: %3d | minPitchValue: %3d | maxPitchValue: %3d\n", pitchMaped, minPitchValue, maxPitchValue);
  moveAxis(-rollMaped, pitchMaped);
}

void settingsLoop()
{
  blinkLed();
  int pitchCurrent = analogRead(PIN_PITCH_ANALOG);
  int rollCurrent = analogRead(PIN_ROLL_ANALOG);
  minPitchValue = min(pitchCurrent, minPitchValue);
  maxPitchValue = max(pitchCurrent, maxPitchValue);
  minRollValue = min(rollCurrent, minRollValue);
  maxRollValue = max(rollCurrent, maxRollValue);
}

void blinkLed()
{
  unsigned long currentMillis = millis();

  if (currentMillis - previousBlinkMillis >= BLINK_INTERVAL)
  {
    previousBlinkMillis = currentMillis;
    if (ledState == LOW)
    {
      ledState = HIGH;
    }
    else
    {
      ledState = LOW;
    }
    digitalWrite(PIN_LED, ledState);
  }
}

void settingSave()
{
  MinMaxStruct minMaxVar = {
    minPitchValue,
    maxPitchValue,
    minRollValue,
    maxRollValue
  };
  EEPROM.put(MIN_MAX_ADDR, minMaxVar);
}
