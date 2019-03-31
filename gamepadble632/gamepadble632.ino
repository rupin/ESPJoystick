//https://github.com/nkolban/esp32-snippets/issues/632

/*Button and Byte Map

Byte 1
LSB A B C X Y Z L1 R1 MSB
*/


#include <Arduino.h>



/**
 * Create a new BLE server.
 */
#include "BLEDevice.h"
#include "BLEServer.h"
#include "BLEUtils.h"
#include "BLE2902.h"
#include "BLEHIDDevice.h"

#define ABUTTON 35
#define BBUTTON 34
#define CBUTTON 32
#define XBUTTON 36
#define YBUTTON 39
#define ZBUTTON 33
#define L1_BUTTON 4
#define R1_BUTTON 2
#define ANALOG_X 27



#define BUTTONCOUNT 8

int buttonArray[BUTTONCOUNT]={R1_BUTTON,L1_BUTTON,ZBUTTON,YBUTTON,XBUTTON,CBUTTON,BBUTTON,ABUTTON };
byte buttonIndex=0;

static BLEHIDDevice* hid;
BLECharacteristic* input;
BLECharacteristic* output;

class MyCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer* pServer){
    Serial.println("connected");

                // workaround after reconnect (see comment below) 
                BLEDescriptor *desc = input->getDescriptorByUUID(BLEUUID((uint16_t)0x2902));
                uint8_t val[] = {0x01, 0x00};
                desc->setValue(val, 2);
  }

  void onDisconnect(BLEServer* pServer){
        Serial.println("disconnected");
  }
};

void setup() {
    //
    Serial.begin(115200);
    Serial.println("Starting");
    for(buttonIndex=0;buttonIndex<BUTTONCOUNT; buttonIndex++)
    {
     pinMode(buttonArray[buttonIndex], INPUT);
    }

    //
    BLEDevice::init("ESP32");

    BLEServer *pServer = BLEDevice::createServer();
    pServer->setCallbacks(new MyCallbacks());

    /*
     * Instantiate hid device
     */
    hid = new BLEHIDDevice(pServer);

    input = hid->inputReport(1); // <-- input REPORTID from report map
    output = hid->outputReport(1); // <-- output REPORTID from report map

    /*
     * Set manufacturer name (OPTIONAL)
     * https://www.bluetooth.com/specifications/gatt/viewer?attributeXmlFile=org.bluetooth.characteristic.manufacturer_name_string.xml
     */
    std::string name = "esp-community";
    hid->manufacturer()->setValue(name);

    /*
     * Set pnp parameters (MANDATORY)
     * https://www.bluetooth.com/specifications/gatt/viewer?attributeXmlFile=org.bluetooth.characteristic.pnp_id.xml
     */
    hid->pnp(0x02, 0x0810, 0xe501, 0x0106);

    /*
     * Set hid informations (MANDATORY)
     * https://www.bluetooth.com/specifications/gatt/viewer?attributeXmlFile=org.bluetooth.characteristic.hid_information.xml
     */
    hid->hidInfo(0x00,0x01);

    /*
     * Gamepad
     */
    const uint8_t reportMap[] = {
        0x05, 0x01,  /* USAGE_PAGE (Generic Desktop)       */
        0x09, 0x05,  /* USAGE (Game Pad)                   */
        0xa1, 0x01,  /* COLLECTION (Application)           */
        0xa1, 0x03,  /*   COLLECTION (Report)              */
        0x85, 0x01,  /*     REPORT_ID (1)                  */
        0x05, 0x09,  /*     USAGE_PAGE (Button)            */
        0x19, 0x01,  /*     USAGE_MINIMUM (Button 1)       */
        0x29, 0x10,  /*     USAGE_MAXIMUM (Button 16)      */
        0x15, 0x00,  /*     LOGICAL_MINIMUM (0)            */
        0x25, 0x01,  /*     LOGICAL_MAXIMUM (1)            */
        0x95, 0x10,  /*     REPORT_COUNT (8)              */
        0x75, 0x01,  /*     REPORT_SIZE (2)                */
        0x81, 0x02,  /*     INPUT (Data,Var,Abs)           */        
        0xa1, 0x00,  /*     COLLECTION (Physical)          */
        0x05, 0x01,  /*       USAGE_PAGE (Generic Desktop) */
        0x09, 0x30,  /*       USAGE (X)                    */
        0x09, 0x31,  /*       USAGE (Y)                    */
        0x15, 0x00,  /*       LOGICAL_MINIMUM (0)          */
        0x25, 0x64,  /*       LOGICAL_MAXIMUM (100)        */
        0x75, 0x08,  /*       REPORT_SIZE (8)              */
        0x95, 0x02,  /*       REPORT_COUNT (2)             */
        0x81, 0x02,  /*       INPUT (Data,Var,Abs)         */
        0xc0,        /*     END_COLLECTION                 */
        0xc0,        /*   END_COLLECTION                   */
        0xc0         /* END_COLLECTION                     */
    };

    /*
     * Set report map (here is initialized device driver on client side) (MANDATORY)
     * https://www.bluetooth.com/specifications/gatt/viewer?attributeXmlFile=org.bluetooth.characteristic.report_map.xml
     */
    hid->reportMap((uint8_t*)reportMap, sizeof(reportMap));

    /*
     * We are prepared to start hid device services. Before this point we can change all values and/or set parameters we need.
     * Also before we start, if we want to provide battery info, we need to prepare battery service.
     * We can setup characteristics authorization
     */
    hid->startServices();

    /*
     * Its good to setup advertising by providing appearance and advertised service. This will let clients find our device by type
     */
    BLEAdvertising *pAdvertising = pServer->getAdvertising();
    pAdvertising->setAppearance(HID_GAMEPAD);    // <-- optional
    pAdvertising->addServiceUUID(hid->hidService()->getUUID());
    pAdvertising->start();

    BLESecurity *pSecurity = new BLESecurity();
    pSecurity->setAuthenticationMode(ESP_LE_AUTH_BOND);

    //Serial.println("waiting 5sec");
    delay(5000);
    Serial.println("starting");
}

 byte lButtonState;


void loop() {   
   
 
    lButtonState=getButtonState();
    uint8_t a[] = {lButtonState,0x00,0x55, 0x55};
    input->setValue(a, sizeof(a));
    input->notify();
    /*delay(100);
    
    uint8_t b[] = {0x00, 0x00, 0x00, 0x00};
    input->setValue(b, sizeof(b));
    input->notify();
    delay(100);
   */
    
}

 byte getButtonState()
{
  byte buttonState=0;
  byte currentButtonPressed;
   for(buttonIndex=0;buttonIndex<BUTTONCOUNT; buttonIndex++)
  {
    currentButtonPressed=digitalRead(buttonArray[buttonIndex]);
    buttonState=buttonState | currentButtonPressed;
    buttonState=buttonState<<1;// Push left by 1 bit
  }
  buttonState=buttonState>>1;

  return buttonState;
  
}