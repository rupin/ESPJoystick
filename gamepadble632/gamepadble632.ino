//https://github.com/nkolban/esp32-snippets/issues/632

/*Button and Byte Map

  Byte 1
  LSB A B C X Y Z L1 R1 MSB
*/


#include <Arduino.h>



/**
   Create a new BLE server.
*/
#include "BLEDevice.h"
#include "BLEServer.h"
#include "BLEUtils.h"
#include "BLE2902.h"
#include "BLEHIDDevice.h"

#define ABUTTON 26
#define BBUTTON 25
#define CBUTTON 32
#define XBUTTON 4
#define YBUTTON 15
#define ZBUTTON 27
#define L1_BUTTON 34
#define R1_BUTTON 35
#define ANALOG_X 36
#define ANALOG_Y 39
#define PULSE_PIN 27

#define PULSE_MIN_DURATION 0
#define PULSE_MAX_DURATION 2048

#define JOYSTICK_ANALOG_MAX 255
#define JOYSTICK_ANALOG_MIN 0

#define UNPRESSED_BUTTON_VALUE 0


#define SAMPLE_COUNT 4 //Must be a power of two

portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;

uint32_t connectedDelay = millis();
boolean reconnected = false;


boolean userSignalledConnected = false;
uint32_t lastInterruptTime = 0;
uint32_t pulseDuration = 0;

#define BUTTONCOUNT 8

int buttonArray[BUTTONCOUNT] = {R1_BUTTON, L1_BUTTON, ZBUTTON, YBUTTON, XBUTTON, CBUTTON, BBUTTON, ABUTTON };
byte buttonIndex = 0;

static BLEHIDDevice* hid;


BLECharacteristic* device1;
BLECharacteristic* device1o;

BLECharacteristic* device2;
BLECharacteristic* device2o;
boolean device_connected = false;


/*class MyCallbacks : public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
      Serial.println("connected");
      device_connected = true;

      // workaround after reconnect (see comment below)
      BLEDescriptor *desc1 = device1->getDescriptorByUUID(BLEUUID((uint16_t)0x2902));
      uint8_t val1[] = {0x01, 0x00};
      desc1->setValue(val1, 2);



    }

    void onDisconnect(BLEServer* pServer) {
      Serial.println("disconnected");

    }
  };*/

const uint8_t reportMap[] = {
  0x05, 0x01,  // USAGE_PAGE (Generic Desktop)
  0x09, 0x04,  // USAGE (Game Pad)
  0xa1, 0x01,  // COLLECTION (Application)

  0x85, 0x01,  //     REPORT_ID (1)
  0x05, 0x09,  //     USAGE_PAGE (Button)
  0x19, 0x01,  //     USAGE_MINIMUM (Button 1)
  0x29, 0x10,  //     USAGE_MAXIMUM (Button 16)
  0x15, 0x00,  //     LOGICAL_MINIMUM (0)
  0x25, 0x01,  //     LOGICAL_MAXIMUM (1)
  0x95, 0x08,  //     REPORT_COUNT (8)
  0x75, 0x02,  //     REPORT_SIZE (2)
  0x81, 0x02,  //     INPUT (Data,Var,Abs)
  //0xa1, 0x00,  //     COLLECTION (Physical)
  0x05, 0x01,  //       USAGE_PAGE (Generic Desktop)
  0x09, 0x30,  //       USAGE (X)
  0x09, 0x31,  //       USAGE (Y)
  0x15, 0x00,  //       LOGICAL_MINIMUM (0)
  0x25, 0xFF,  //       LOGICAL_MAXIMUM (100)
  0x75, 0x08,  //       REPORT_SIZE (8)
  0x95, 0x02,  //       REPORT_COUNT (2)
  0x81, 0x02,  //       INPUT (Data,Var,Abs)
  // 0xc0,        //     END_COLLECTION

  0xc0,         // END_COLLECTION

  0x05, 0x01,  // USAGE_PAGE (Generic Desktop)
  0x09, 0x04,  // USAGE (Game Pad)
  0xa1, 0x01,  // COLLECTION (Application)

  0x85, 0x02,  //     REPORT_ID (2)
  0x05, 0x09,  //     USAGE_PAGE (Button)
  0x19, 0x01,  //     USAGE_MINIMUM (Button 1)
  0x29, 0x10,  //     USAGE_MAXIMUM (Button 16)
  0x15, 0x00,  //     LOGICAL_MINIMUM (0)
  0x25, 0x01,  //     LOGICAL_MAXIMUM (1)
  0x95, 0x08,  //     REPORT_COUNT (8)
  0x75, 0x02,  //     REPORT_SIZE (2)
  0x81, 0x02,  //     INPUT (Data,Var,Abs)
  //0xa1, 0x00,  //     COLLECTION (Physical)
  0x05, 0x01,  //       USAGE_PAGE (Generic Desktop)
  0x09, 0x33,  //       USAGE (X)
  0x09, 0x34,  //       USAGE (Y)
  0x15, 0x00,  //       LOGICAL_MINIMUM (0)
  0x25, 0xFF,  //       LOGICAL_MAXIMUM (100)
  0x75, 0x08,  //       REPORT_SIZE (8)
  0x95, 0x02,  //       REPORT_COUNT (2)
  0x81, 0x02,  //       INPUT (Data,Var,Abs)
  // 0xc0,        //     END_COLLECTION

  0xc0         // END_COLLECTION


};

class MyCallbacks : public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
      device_connected = true;
      BLE2902* desc1 = (BLE2902*)device1->getDescriptorByUUID(BLEUUID((uint16_t)0x2902));
      desc1->setNotifications(true);

      BLE2902* desc2 = (BLE2902*)device2->getDescriptorByUUID(BLEUUID((uint16_t)0x2902));
      desc2->setNotifications(true);
    }

    void onDisconnect(BLEServer* pServer) {
      //connected = false;
      BLE2902* desc1 = (BLE2902*)device1->getDescriptorByUUID(BLEUUID((uint16_t)0x2902));
      desc1->setNotifications(false);

      BLE2902* desc2 = (BLE2902*)device2->getDescriptorByUUID(BLEUUID((uint16_t)0x2902));
      desc2->setNotifications(false);

      device_connected = false;
      userSignalledConnected = false;
    }
};


void taskServer() {
  BLEDevice::init("ESP32");
  BLEServer *pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyCallbacks());

  hid = new BLEHIDDevice(pServer);
  device1 = hid->inputReport(1); // <-- input REPORTID from report map
  device1o = hid->outputReport(1); // <-- output REPORTID from report map


  device2 = hid->inputReport(2); // <-- input REPORTID from report map
  device2o = hid->outputReport(2); // <-- output REPORTID from report map

  std::string name = "ElectronicCats";
  hid->manufacturer()->setValue(name);

  hid->pnp(0x02, 0x0810, 0xe501, 0x0106);
  hid->hidInfo(0x00, 0x02);

  hid->reportMap((uint8_t*)reportMap, sizeof(reportMap));
  hid->startServices();


  BLESecurity *pSecurity = new BLESecurity();
  //  pSecurity->setKeySize();
  pSecurity->setAuthenticationMode(ESP_LE_AUTH_BOND);


  BLEAdvertising *pAdvertising = pServer->getAdvertising();
  pAdvertising->setAppearance(HID_GAMEPAD);
  pAdvertising->addServiceUUID(hid->hidService()->getUUID());
  pAdvertising->start();
  hid->setBatteryLevel(7);

  // delay(portMAX_DELAY);
}

void IRAM_ATTR handleInterrupt();
void setup() {
  //
  Serial.begin(115200);
  Serial.println("Starting");
  for (buttonIndex = 0; buttonIndex < BUTTONCOUNT; buttonIndex++)
  {
    pinMode(buttonArray[buttonIndex], INPUT);
  }
  pinMode(PULSE_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(PULSE_PIN), handleInterrupt, RISING);


  //xTaskCreate(taskServer, "server", 20000, NULL, 5, NULL);
  taskServer();
  Serial.println("starting");
}

uint8_t lButtonState;
byte lastButtonState;
byte X_Joystick;
byte Y_Joystick;
uint8_t userSignal;

void loop() {

  if (!userSignalledConnected) //User has not as yet signalled that pairing is completed on phone
  {
    userSignal = getButtonState();
    if (userSignal != UNPRESSED_BUTTON_VALUE)
    {
      userSignalledConnected = true;
    }
  }


  if (userSignalledConnected) //Only when the user signals, do we start sending data
  {
    lButtonState = getButtonState();
    X_Joystick = getAnalogChannelValue(ANALOG_X);
    Y_Joystick = getAnalogChannelValue(ANALOG_Y);
    uint8_t a[] = {lButtonState, 0x00, X_Joystick, Y_Joystick};
    device1->setValue(a, sizeof(a));
    device1->notify();

    uint8_t b[] = {0x00, lButtonState, X_Joystick, Y_Joystick};
    device2->setValue(b, sizeof(b));
    device2->notify();


    lastButtonState = lButtonState;

    //delay(1);

  }

  delay(1);
}

uint8_t getButtonState()
{
  byte buttonState = 0;
  byte currentButtonPressed;
  for (buttonIndex = 0; buttonIndex < BUTTONCOUNT; buttonIndex++)
  {
    currentButtonPressed = digitalRead(buttonArray[buttonIndex]);
    buttonState = buttonState | currentButtonPressed;
    buttonState = buttonState << 1; // Push left by 1 bit
  }
  buttonState = buttonState >> 1;

  return buttonState;

}

byte getAnalogChannelValue(const int analogpin)
{
  uint8_t i;
  uint32_t joystickValue = 0;
  for (i = 0; i < SAMPLE_COUNT; i++)
  {
    joystickValue = joystickValue + analogRead(analogpin);

  }
  if (SAMPLE_COUNT == 2)
  {
    joystickValue = joystickValue >> 1; //shifting by 1 means division by 2
  }
  if (SAMPLE_COUNT == 4)
  {
    joystickValue = joystickValue >> 2; //shifting by 2 means division by 4
  }
  //We have now averaged the value of the joystick which is in range 0-4095(12 bits), we have to bring it in range 0-255(8 bits)
  //Hence shift by 4 places
  joystickValue = joystickValue >> 4;
  return (byte)joystickValue;

}

byte getFrequency()
{
  if (millis() - lastInterruptTime > PULSE_MAX_DURATION)
  {
    return 127;// Midpoint value for a Joystick With full range 0-255
  }
  else
  {
    // PULSE_MAX_DURATION is 2048 milliseconds/2.048 seconds. To bring it to range 0-128, we need to shift it by 4 bits.
    //This is faster than the map function.
    return pulseDuration >> 4;
  }

}

void IRAM_ATTR handleInterrupt()
{
  portENTER_CRITICAL_ISR(&mux);
  pulseDuration = millis() - lastInterruptTime;
  lastInterruptTime = millis();
  portEXIT_CRITICAL_ISR(&mux);
}
