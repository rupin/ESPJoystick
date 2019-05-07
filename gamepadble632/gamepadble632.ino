//https://github.com/nkolban/esp32-snippets/issues/632

/*Button and Byte Map

  Byte 1
  LSB A B C X Y Z L1 R1 MSB
*/


#include <Arduino.h>
#include <SparkFunMPU9250-DMP.h>
#include<Math.h>
#define MSB 1
#define LSB 0

MPU9250_DMP imu;





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


boolean device_connected = false;


uint16_t orientationData[6];

const uint8_t reportMap[] = {
  0x05, 0x01, // USAGE_PAGE (Generic Desktop)
  0x09, 0x04,  // USAGE (Joystick)
  0xa1, 0x01, // COLLECTION (Application)

  0x85, 0x01,  //     REPORT_ID (1)
  0x05, 0x09,  //     USAGE_PAGE (Button)
  0x19, 0x01,  //     USAGE_MINIMUM (Button 1)
  0x29, 0x10,  //     USAGE_MAXIMUM (Button 16)
  0x15, 0x00,  //     LOGICAL_MINIMUM (0)
  0x25, 0x01,  //     LOGICAL_MAXIMUM (1)
  0x95, 0x08,  //     REPORT_COUNT (16)
  0x75, 0x02,  //     REPORT_SIZE (1)
  0x81, 0x02,  //     INPUT (Data,Var,Abs)

  0x05, 0x01,  //     USAGE_PAGE (Generic Desktop)
  0x09, 0x30,  //     USAGE (X)
  0x09, 0x31,  //     USAGE (Y)
  0x15, 0x00,  //     LOGICAL_MINIMUM (0)
  0x26, 0xFF,0x00,  //     LOGICAL_MAXIMUM (100)
  0x75, 0x08,  //     REPORT_SIZE (8)
  0x95, 0x02,  //     REPORT_COUNT (2)
  0x81, 0x02,  //     INPUT (Data,Var,Abs)
  

  0x05, 0x01,  //     USAGE_PAGE (Generic Desktop)
  0x09, 0x33,  //     USAGE (RX)
  0x09, 0x34,  //     USAGE (RY)
  0x09, 0x35,  //     USAGE (RZ)
  0x15, 0x00, //     LOGICAL_MINIMUM (0) //See https://www.microchip.com/forums/m413261.aspx
  0x26, 0x01, 0x68,// LOGICAL_MAXIMUM (360) //Sequence of Bytes here (MSB, LSB) should be same sequence in sending data.
 
  0x75, 0x10,  //     REPORT_SIZE (16)
  0x95, 0x03,  //     REPORT_COUNT (3)
  0x81, 0x02,  //     INPUT (Data,Var,Abs)


  0xc0       //       END COLLECTION (Application)




};

class MyCallbacks : public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {

      BLE2902* desc1 = (BLE2902*)device1->getDescriptorByUUID(BLEUUID((uint16_t)0x2902));
      desc1->setNotifications(true);

      device_connected = true;


    }

    void onDisconnect(BLEServer* pServer) {
      //connected = false;
      BLE2902* desc1 = (BLE2902*)device1->getDescriptorByUUID(BLEUUID((uint16_t)0x2902));
      desc1->setNotifications(false);

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


  // device2 = hid->inputReport(2); // <-- input REPORTID from report map
  //device2o = hid->outputReport(2); // <-- output REPORTID from report map

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


  // IMU Code
  if (imu.begin() != INV_SUCCESS)
  {
    while (1)


    {
      Serial.println("Unable to communicate with MPU-9250");
      Serial.println("Check connections, and try again.");
      Serial.println();
      delay(5000);
    }
  }

  imu.dmpBegin(DMP_FEATURE_6X_LP_QUAT | // Enable 6-axis quat
               DMP_FEATURE_GYRO_CAL, // Use gyro calibration
               100); // Set DMP FIFO rate to 10 Hz
  // DMP_FEATURE_LP_QUAT can also be used. It uses the
  // accelerometer in low-power mode to estimate quat's.
  // DMP_FEATURE_LP_QUAT and 6X_LP_QUAT are mutually exclusive

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
  updateIMUData();
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
    uint8_t a[] = {lButtonState,
                   0x00,
                   X_Joystick,
                   Y_Joystick,
                   orientationData[0],
                   orientationData[1],
                   orientationData[2],
                   orientationData[3],
                   orientationData[4],
                   orientationData[5]
                  };
     

              
    device1->setValue(a, sizeof(a));
    device1->notify();

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

void updateIMUData()
{
  // Check for new data in the FIFO
  if ( imu.fifoAvailable() )
  {
    // Use dmpUpdateFifo to update the ax, gx, mx, etc. values
    if ( imu.dmpUpdateFifo() == INV_SUCCESS)
    {
      // computeEulerAngles can be used -- after updating the
      // quaternion values -- to estimate roll, pitch, and yaw
      imu.computeEulerAngles();

      uint16_t pitchData = (uint16_t)imu.pitch;
      uint16_t rollData = (uint16_t)imu.roll;
      uint16_t yawData = (uint16_t)imu.yaw;

     // uint16_t pitchData = 45;
     // uint16_t rollData = 35;
     // uint16_t yawData = 22;

      //Serial.println(pitchData);

      orientationData[0] = getByte(pitchData, MSB);
      orientationData[1] = getByte(pitchData, LSB);

      orientationData[2] = getByte(rollData, MSB);
      orientationData[3] = getByte(rollData, LSB);

      orientationData[4] = getByte(yawData, MSB);
      orientationData[5] = getByte(yawData, LSB);

      for (byte i = 0; i < 6; i++)
      {
        Serial.print(orientationData[i]);
        Serial.print("\t");
      }
      Serial.println();




    }
  }
}
uint8_t getByte(uint16_t sentData, uint8_t whichByte)
{
  uint8_t retVal = 0;
  if (whichByte == MSB)
  {
    retVal = (sentData & 0xFF00) >> 8;
    return retVal;
  }

  if (whichByte == LSB)
  {
    retVal = sentData & 0x00FF;
    return retVal;
  }

}
