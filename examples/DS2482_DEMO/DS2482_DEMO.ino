#include <DS18B20_DS2482.h>
#include <stdlib.h>
#include "Arduino_DebugUtils.h"


#define MAX_NUMBER_OF_SENSORS   16    // maximum number of Dallas sensors
#define MAX_NUMBER_OF_CHANNELS  8     // 1=DS2482-100, 8=DS2482-800

struct DSSensorStruct
{
  uint8_t Channel;
  uint8_t Addr[8];
  float Val;
};

// ADD SENSORS ONLY TO THE END OF LIST
// ORDER IS USED IN VISUINO ANALOG ARRAY OUTPUT
const char* DSSensors[MAX_NUMBER_OF_SENSORS][8] = {
    {0x28, 0x56, 0xe1, 0x30, 0x00, 0x00, 0x00, 0xdd}, // T0
    {0x28, 0xff, 0xe2, 0x47, 0x04, 0x00, 0x00, 0xe9}  // T1
};

DSSensorStruct Sensors[MAX_NUMBER_OF_SENSORS];
DS2482 OneWireDemux(0);
DS18B20_DS2482 DSTemperature(&OneWireDemux);
uint8_t OneWireDemuxCount = 0;
uint8_t DSCount = 0;

// --------------------------------------------------------------------
void setup() 
{
  Serial.begin(9600);  
  Debug.setDebugLevel(DBG_DEBUG);
  Debug.print(DBG_INFO, "\n--------- DS2482-x00 Test ----------------");
  
  // I2C
  Debug.print(DBG_INFO, "Starting I2C");
  OneWireDemux.Setup(2);
  bool res = OneWireDemux.reset();
  Debug.print(DBG_DEBUG, "DS2482 reset: %s", res ? "Ok" : "Error");

  //search for devices
  OneWireDemuxCount = OneWireDemux.devicesCount(true);
  Debug.print(DBG_INFO, "DS2482 devices: %d", OneWireDemuxCount);
  
  for (uint8_t i=0; i<MAX_NUMBER_OF_CHANNELS; i++)
  {
    DSCount += ScanChannel(i, DSCount);
  }

  DSTemperature.setResolution(10);
}

// --------------------------------------------------------------------
uint8_t ScanChannel(uint8_t channel, uint8_t sensoroffset)
{
  bool res = OneWireDemux.selectChannel(channel);
  Debug.print(DBG_DEBUG, "Channel%d selected: %s", channel, res ? "Ok" : "Error");
  //delay(1000);

  // 1-Wire
  DSTemperature.begin();
  uint8_t c = DSTemperature.getDeviceCount();
  //Serial.println("Channel" + String(channel)+" DS18b20 devices: " + String(c));

  for (uint8_t i=0; i<c; i++)
  {
    uint8_t index = sensoroffset + i;
    DSTemperature.getAddress(Sensors[index].Addr, i);

    Sensors[index].Channel = channel;
    Debug.print(DBG_INFO, "Channel%d Sensor%d address: %s", channel, index, FormatAddress(index).c_str());
  }

  return c;
}

// --------------------------------------------------------------------
String FormatAddress(uint8_t index)
{
    String addr = "";
    for (uint8_t j=0; j<8; j++)
    {
      if (Sensors[index].Addr[j] < 0x10) addr += "0";
      addr += String(Sensors[index].Addr[j], HEX);
      if (j<7) addr += "-";
    }
    return addr;
}

// --------------------------------------------------------------------
void loop() 
{
  // loop throught detected sensors
  for (uint8_t i = 0; i < DSCount; i++)
  {
    OneWireDemux.selectChannel(Sensors[i].Channel);
    DSTemperature.requestTemperaturesByAddress(Sensors[i].Addr);
    Sensors[i].Val = DSTemperature.getTempC(Sensors[i].Addr);
    if (Sensors[i].Val < -55.0 || Sensors[i].Val > 125.0) Sensors[i].Val = 255;
    uint8_t j;
    bool found = false;
    
    // locate sensor in declared sensors array (we know that for example index0=T_1, index1=T_2)
    for (j = 0; j < MAX_NUMBER_OF_SENSORS; j++)
    {
        found = Sensors[i].Addr[0] == DSSensors[j][0] 
            & Sensors[i].Addr[1] == DSSensors[j][1]
            & Sensors[i].Addr[2] == DSSensors[j][2]
            & Sensors[i].Addr[3] == DSSensors[j][3]
            & Sensors[i].Addr[4] == DSSensors[j][4]
            & Sensors[i].Addr[5] == DSSensors[j][5]
            & Sensors[i].Addr[6] == DSSensors[j][6]
            & Sensors[i].Addr[7] == DSSensors[j][7];
        if (found) break;        
    }

    if (!found) Debug.print(DBG_DEBUG, "Channel%d, %s: %s", Sensors[i].Channel, FormatAddress(i).c_str(), String(Sensors[i].Val, 3).c_str());
    else Debug.print(DBG_DEBUG, "Channel%d, Sensor%d: %s", Sensors[i].Channel, j, String(Sensors[i].Val, 3).c_str());
  }

  delay(1000);
}
