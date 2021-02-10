#define TINY_GSM_MODEM_SIM800
#define ARDUINOJSON_USE_LONG_LONG 1
#define uS_TO_S_FACTOR 1000000 
#define TIME_TO_SLEEP 40   //in seconds

#include <ThingsBoard_broker.h>
#include <TinyGsmClient.h>
#include <ArduinoJson.h>
#include <esp_now.h>
#include <WiFi.h>

#define CHANNEL 1

#define MODEM_RST            5
#define MODEM_PWKEY          4
#define MODEM_POWER_ON       23
#define MODEM_TX             27
#define MODEM_RX             26

RTC_DATA_ATTR int bootCount = 0;
bool modemConnected = false;
int totalSlaves=3; // number of slave sending data to thid broker
int sendStatus;
int counter=0;

#define SerialAT Serial1

#define TOKEN               "A123"
#define THINGSBOARD_SERVER  "13.233.208.114"

const char apn[]  = "airtelgprs.com";
const char user[] = "";
const char pass[] = "";

TinyGsm modem(SerialAT);

TinyGsmClient client(modem);
ThingsBoard tb(client);

typedef struct message 
{
  float temperature;
  float humidity;
  float pressure;
  String device;
};
 
struct message myMessage;


void GSM_OFF()
{
  pinMode(MODEM_PWKEY, OUTPUT);
  pinMode(MODEM_POWER_ON, OUTPUT);
  pinMode(MODEM_RST, OUTPUT);

  digitalWrite(MODEM_PWKEY, HIGH);   // turn of modem in case its ON from previous state
  digitalWrite(MODEM_POWER_ON, LOW); // turn of modem psu in case its from previous state
  digitalWrite(MODEM_RST, HIGH);     // Keep IRQ high ? (or not to save power?)
}

void GSM_ON(uint32_t time_delay)
{
  pinMode(MODEM_PWKEY, OUTPUT);
  pinMode(MODEM_POWER_ON, OUTPUT);
  pinMode(MODEM_RST, OUTPUT);

  Serial.println("MODEM_RST & IP5306 IRQ: HIGH"); // IP5306 HIGH
  digitalWrite(MODEM_RST, HIGH);
  delay(time_delay);

  Serial.println("MODEM_PWKEY: HIGH");
  digitalWrite(MODEM_PWKEY, HIGH); // turning modem OFF
  delay(time_delay);

  Serial.println("MODEM_POWER_ON: HIGH");
  digitalWrite(MODEM_POWER_ON, HIGH); //Enabling SY8089 4V4 for SIM800 (crashing when in battery)
  delay(time_delay);

  Serial.println("MODEM_PWKEY: LOW");
  digitalWrite(MODEM_PWKEY, LOW); // turning modem ON
  delay(time_delay);
}

void shutdown()
{
  modemConnected = false;
  Serial.println(F("GPRS disconnect"));
  modem.gprsDisconnect();

  Serial.println("Radio off");
  modem.radioOff();

  Serial.println("GSM power off");
  GSM_OFF();

  Serial.println("Going to sleep now");
  Serial.flush();
  esp_deep_sleep_start();
}

void InitESPNow() 
{
  WiFi.disconnect();
  if (esp_now_init() == ESP_OK) 
  {
    Serial.println("ESPNow Init Success");
  }
  else 
  {
    Serial.println("ESPNow Init Failed");
    ESP.restart();
  }
}


void configDeviceAP() 
{
  String Prefix = "Slave:";
  String Mac = WiFi.macAddress();
  String SSID = Prefix + Mac;
  String Password = "123456789";
  bool result = WiFi.softAP(SSID.c_str(), Password.c_str(), CHANNEL, 0);
  if (!result) 
  {
    Serial.println("AP Config failed.");
  } 
  else 
  {
    Serial.println("AP Config Success. Broadcasting with AP: " + String(SSID)); 
  }
}


void reconnect()
{
  if (!modemConnected || !tb.connected())
  { 
    SerialAT.begin(115200, SERIAL_8N1, MODEM_RX, MODEM_TX);
    delay(300);
    GSM_ON(100);

    String modemInfo = modem.getModemInfo();
    Serial.print(F("Modem: "));
    Serial.println(modemInfo);
    Serial.print(F("Waiting for network..."));
    
    if (!modem.waitForNetwork(240000L))
    { Serial.println(" fail");}
    
    Serial.println(" OK");

    Serial.print("Signal quality:");
    Serial.println(modem.getSignalQuality());

    Serial.print(F("Connecting to "));
    Serial.print(apn);
    
    if (!modem.gprsConnect(apn, user, pass))
    { Serial.println(" fail"); }
    
    modemConnected = true;
    Serial.println(" OK");
  }

  if (modem.isNetworkConnected()) 
  { Serial.println("Network connected"); }

  if (!tb.connected())
  {
    Serial.print("Connecting to: ");
    Serial.print(THINGSBOARD_SERVER);
    Serial.print(" with token ");
    Serial.println(TOKEN);
    
    if (!tb.connect(THINGSBOARD_SERVER, TOKEN))
    { Serial.println("Failed to connect"); }
  }
 }


void sendData(String(Device),float(Temperature),float(Humidity),float(Pressure))
{
  reconnect();
  
  Serial.println("Sending data...");

  DynamicJsonDocument root(1024);
  DynamicJsonDocument boot(1024);

  root["ts"] = 1608146437669;
 
  JsonObject data = root.createNestedObject("values");
  data["temperature"]= Temperature;
  data["humidity"]= Humidity;
  data["pressure"]= Pressure;

  JsonArray shot = boot.createNestedArray(Device);
  shot.add(root);
  
  serializeJsonPretty(boot, Serial);

  char output[100];
  serializeJson(boot, output);

  sendStatus =tb.sendTelemetryJson(output);

 Serial.print("Send Status: ");
 Serial.println(sendStatus);

 if(sendStatus == 1)
 { Serial.println("Sucess to send data to thingsboard");}
 
 else
 { Serial.println("Failed to send data to thingsboard");}
 
 delay(1000);
}


void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len)
 {
  
 counter++;
 Serial.println("Message received.");
 memcpy(&myMessage, incomingData, sizeof(myMessage));
 Serial.println("=== Data ===");
 Serial.print("Mac address: ");
 for (int i = 0; i < 6; i++) 
 {
     Serial.print("0x");
     Serial.print(mac[i], HEX);
     Serial.print(":");
 }
 Serial.print("\nSlave id ");
 Serial.println(myMessage.device);   
 Serial.print("Temperature: ");
 Serial.println(myMessage.temperature);
 Serial.print("Humidity: ");
 Serial.println(myMessage.humidity);
 Serial.print("Presure: ");
 Serial.println(myMessage.pressure);

 sendData(String(myMessage.device),float(myMessage.temperature),float(myMessage.humidity),float(myMessage.pressure));
 
 if(totalSlaves == counter)
 { shutdown(); }  
 
}

void setup() 
{
  Serial.begin(9600);
  delay(100);

  ++bootCount;
  Serial.println("Boot number: " + String(bootCount));

  esp_sleep_enable_timer_wakeup(TIME_TO_SLEEP * uS_TO_S_FACTOR);
  Serial.println("Setup ESP32 to sleep for " + String(TIME_TO_SLEEP) + " Seconds");
  
  reconnect();

  WiFi.mode(WIFI_AP);
  configDeviceAP();
  Serial.print("AP MAC: "); 
  Serial.println(WiFi.softAPmacAddress());
  InitESPNow();
  esp_now_register_recv_cb(OnDataRecv);
}


void loop() 
{
}
