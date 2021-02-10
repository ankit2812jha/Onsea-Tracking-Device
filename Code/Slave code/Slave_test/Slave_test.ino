#include <ESP8266WiFi.h>
#include <espnow.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>

#define SEALEVELPRESSURE_HPA (1013.25)
#define NUMSLAVES 20
#define CHANNEL 1
#define PRINTSCANRESULTS 0

Adafruit_BME280 bme;

bool status;
int SlaveCnt = 0;

typedef struct esp_now_peer_info 
{
  u8 peer_addr[6];
  uint8_t channel;
  uint8_t encrypt;
}esp_now_peer_info_t;


typedef struct message 
{
  float temperature;
  float humidity;
  float pressure;
  String device;
};

struct message myMessage;

esp_now_peer_info_t slaves[NUMSLAVES] = {};

void InitESPNow() 
{
  WiFi.disconnect();
  if (esp_now_init() == 0) 
  { Serial.println("ESPNow Init Success"); }
  
  else 
  {
    Serial.println("ESPNow Init Failed");
    ESP.restart();
  }
  esp_now_set_self_role(ESP_NOW_ROLE_CONTROLLER);
}


void ScanForSlave() {
  int8_t scanResults = WiFi.scanNetworks();
  memset(slaves, 0, sizeof(slaves));
  SlaveCnt = 0;
  Serial.println("");
  
  if (scanResults == 0) 
  {
    Serial.println("No WiFi devices in AP Mode found");
  } 
  
  else 
  {
    Serial.print("Found "); 
    Serial.print(scanResults); 
    Serial.println(" devices ");
    for (int i = 0; i < scanResults; ++i) 
    {
      String SSID = WiFi.SSID(i);
      int32_t RSSI = WiFi.RSSI(i);
      String BSSIDstr = WiFi.BSSIDstr(i);

      if (PRINTSCANRESULTS) 
      {
        Serial.print(i + 1); Serial.print(": "); Serial.print(SSID); Serial.print(" ["); Serial.print(BSSIDstr); Serial.print("]"); Serial.print(" ("); Serial.print(RSSI); Serial.print(")"); Serial.println("");
      }
      delay(10);
      
      if (SSID.indexOf("Slave") == 0) 
      {
        Serial.print(i + 1); Serial.print(": "); Serial.print(SSID); Serial.print(" ["); Serial.print(BSSIDstr); Serial.print("]"); Serial.print(" ("); Serial.print(RSSI); Serial.print(")"); Serial.println("");
        int mac[6];

        if ( 6 == sscanf(BSSIDstr.c_str(), "%x:%x:%x:%x:%x:%x",  &mac[0], &mac[1], &mac[2], &mac[3], &mac[4], &mac[5] ) ) 
        {
          for (int ii = 0; ii < 6; ++ii ) 
          {
            slaves[SlaveCnt].peer_addr[ii] = (uint8_t) mac[ii];
          }
        }
        slaves[SlaveCnt].channel = CHANNEL; 
        slaves[SlaveCnt].encrypt = 0; 
        SlaveCnt++;
      }
    }
  }

  if (SlaveCnt > 0) 
  {
    Serial.print(SlaveCnt); Serial.println(" Slave(s) found, processing..");
  } 
  else 
  {
    Serial.println("No Slave Found, trying again.");
  }

  WiFi.scanDelete();
}


void manageSlave() 
{
  if (SlaveCnt > 0) 
  {
    for (int i = 0; i < SlaveCnt; i++) 
    {
      const esp_now_peer_info_t *peer = &slaves[i];
      u8 *peer_addr = slaves[i].peer_addr;
      
      Serial.print("Processing: ");
      for (int ii = 0; ii < 6; ++ii ) 
      {
        Serial.print((uint8_t) slaves[i].peer_addr[ii], HEX);
        if (ii != 5) Serial.print(":");
      }
      Serial.print(" Status: ");
      bool exists = esp_now_is_peer_exist((u8*)peer_addr);
      
      if (exists) 
      { Serial.println("Already Paired");}
       
      else 
      {
        int addStatus = esp_now_add_peer((u8*)peer_addr, ESP_NOW_ROLE_CONTROLLER, CHANNEL, NULL, 0);
        if (addStatus == 0) 
        { Serial.println("Pair success"); }
         
        else 
        { Serial.println("Pair failed");}
        delay(100);
      }
    }
  }
   
  else 
  { Serial.println("No Slave found to process");}
}


void sendData() 
{
  
  myMessage.temperature = bme.readTemperature();
  myMessage.pressure = bme.readPressure()/100.0F;
  myMessage.humidity = bme.readHumidity();
  myMessage.device = "Device A";
    
  for (int i = 0; i < SlaveCnt; i++) 
  {
    u8 *peer_addr = slaves[i].peer_addr;
    if (i == 0) 
    { 
      Serial.print("Sending data");
    }

    int result = esp_now_send(peer_addr, (uint8_t *)&myMessage, sizeof(myMessage));
    Serial.print("Send Status: ");
    if (result == 0) 
    {
      Serial.println("Success");
    } 
    else 
    {
      Serial.println("Failed");
    }
    delay(100);
  }
}

esp_now_send_cb_t OnDataSent([](uint8_t *mac_addr, uint8_t status) 
{
  char macStr[18];
  snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x",
           mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
  Serial.print("Last Packet Sent to: "); Serial.println(macStr);
  Serial.print("Last Packet Send Status: "); Serial.println(status == 0 ? "Delivery Success" : "Delivery Fail");
});

void setup() 
{
  Serial.begin(115200);
  
  status = bme.begin(0x76);  
  if (!status) {
    Serial.println("Could not find a valid BME280 sensor, check wiring!");
    while (1);
  }
  
  WiFi.mode(WIFI_STA);
  Serial.print("STA MAC: "); Serial.println(WiFi.macAddress());
  InitESPNow();
  esp_now_register_send_cb(OnDataSent);
}

void loop() 
{
  ScanForSlave();
  
  if (SlaveCnt > 0) 
  { 
    manageSlave();
    sendData();
  } 
 else 
 {}

  delay(5000);
}
