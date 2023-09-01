#include <ArduinoJson.h>
#include <Wire.h>
#include <VL6180X.h>
#include <SPI.h>
#include <MFRC522.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include "driver/ledc.h"

#define VL6180X_ADDRESS_1 0x29       //ToF sensor1 uses default I2C address
#define VL6180X_ADDRESS_2 0x30       //ToF sensor2 has to be changed to this I2C address at boot
#define SHUT_1_PIN GPIO_NUM_32       //SHUT pin to shutdown ToF sensor1, so we can update I2C address of ToF sensor2
#define IRQ_1_PIN GPIO_NUM_34        //IRQ pin ToF sensor1
#define IRQ_2_PIN GPIO_NUM_35        //IRQ pin ToF sensor2
#define LOCK_PIN GPIO_NUM_13         //MOSFET pin door lock
#define RFID_RST_PIN GPIO_NUM_17     //RST pin RFID
#define RFID_SS_PIN GPIO_NUM_5       //SDA pin RFID
#define RFID_IRQ_PIN GPIO_NUM_16     //IRQ pin RFID
#define SW_OUT_DOOR_PIN GPIO_NUM_4   //REED SW pin output door (back)
#define SW_INP_DOOR_PIN GPIO_NUM_2   //REED SW pin input door (front)
#define LED_R_PIN GPIO_NUM_27        //LED red pin
#define LED_G_PIN GPIO_NUM_33        //LED green pin
#define LED_B_PIN GPIO_NUM_26        //LED blue pin
#define LED_W_PIN GPIO_NUM_12        //LED white pin

// Replace the next variables with your SSID/Password combination
const char* ssid = "***";
const char* password = "***";

// Add your MQTT Broker IP address
const char* mqtt_server = "<loxberry-ip>";
const char* mqtt_user = "***";
const char* mqtt_pass = "***";
WiFiClient wifiClient;
PubSubClient MQTTclient(mqtt_server, 1883, wifiClient);

// ToF sensors
VL6180X sensor1;
VL6180X sensor2;

// RFID
MFRC522 rfid(RFID_SS_PIN, RFID_RST_PIN);
MFRC522::MIFARE_Key key;

// IRQ
volatile bool eventToF = false;
volatile bool eventSWOut = false;
volatile bool eventSWIn = false;

// LED
#define REDC   LEDC_CHANNEL_1
#define GREENC LEDC_CHANNEL_2
#define BLUEC  LEDC_CHANNEL_3
#define WHITEC LEDC_CHANNEL_4
uint8_t RGBW[4] = {0,0,0,0};
ledc_channel_t channels[4] = {REDC, GREENC, BLUEC, WHITEC};

// Tasks
SemaphoreHandle_t sema_MQTT_KeepAlive;



/*********************************************************************************
 * WIFI
 *********************************************************************************/

void IRAM_ATTR WiFiEvent(WiFiEvent_t event) {
  switch (event) {
    case SYSTEM_EVENT_STA_CONNECTED:
      Serial.println("Connected to WiFi access point");
      break;
    case SYSTEM_EVENT_STA_DISCONNECTED:
      Serial.println("Disconnected from WiFi access point");
      break;
    case SYSTEM_EVENT_AP_STADISCONNECTED:
      Serial.println("WiFi client disconnected");
      break;
    default: 
      Serial.println("anders");
      break;
  }
}

void connectToWiFi() {
  int TryCount = 0;
  Serial.println("");
  Serial.print("connect to wifi");
  while (WiFi.status() != WL_CONNECTED) {
    TryCount++;
    Serial.printf("  attempt %d...", TryCount);
    Serial.println("");
    WiFi.disconnect();
    WiFi.begin(ssid, password);
    vTaskDelay(4000);
    if (TryCount == 10) {
      ESP.restart();
    }
  }
  WiFi.onEvent(WiFiEvent);
}







/*********************************************************************************
 * MQTT
 *********************************************************************************/

void MQTTkeepalive( void *pvParameters ) {
  sema_MQTT_KeepAlive = xSemaphoreCreateBinary();
  xSemaphoreGive(sema_MQTT_KeepAlive); // found keep alive can mess with a publish, stop keep alive during publish
  MQTTclient.setKeepAlive(90); // setting keep alive to 90 seconds makes for a very reliable connection, must be set before the 1st connection is made.
  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xFrequency = 250; //delay for ms
  for (;;) {
    //check for a is-connected and if the WiFi 'thinks' its connected, found checking on both is more realible than just a single check
    if ((wifiClient.connected()) && (WiFi.status() == WL_CONNECTED)) {
      xSemaphoreTake(sema_MQTT_KeepAlive, portMAX_DELAY); // whiles client.loop() is running no other mqtt operations should be in process
      MQTTclient.loop();
      xSemaphoreGive(sema_MQTT_KeepAlive);
    } else {
      Serial.printf("MQTT keep alive found MQTT status % s WiFi status % s", String(wifiClient.connected()), String(WiFi.status()));
      if (!(wifiClient.connected()) || !(WiFi.status() == WL_CONNECTED)) {
        connectToWiFi();
      }
      connectToMQTT();
    }
    xLastWakeTime = xTaskGetTickCount();
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
  vTaskDelete(NULL);
}

void connectToMQTT() {
  MQTTclient.setKeepAlive(90); // needs be made before connecting
  byte mac[5];
  WiFi.macAddress(mac); // get mac address
  String clientID = String(mac[0]) + String(mac[4]) ; // use mac address to create clientID
  while (!MQTTclient.connected()) {
    MQTTclient.connect(clientID.c_str(), mqtt_user, mqtt_pass, NULL , 1, true, NULL);
    vTaskDelay(250);
  }
  MQTTclient.setCallback(callbackMQTT);
  MQTTclient.subscribe("brievenbus/open");
  Serial.println("MQTT Connected");
}

void callbackMQTT(char* topic, byte* message, unsigned int length) {
  Serial.print("Message arrived on topic: ");
  Serial.print(topic);
  Serial.print(". Message: ");
  String messageTemp;
  
  for (int i = 0; i < length; i++) {
    Serial.print((char)message[i]);
    messageTemp += (char)message[i];
  }
  Serial.println();

  if (!strcmp(topic, "brievenbus/open") && messageTemp.toInt() == 1) {
    unlock();
  }
}

void sendMessageMQTT(const char* topic, const char* msg) {
  while (!MQTTclient.connected()) {
    vTaskDelay(250);
  }
  xSemaphoreTake(sema_MQTT_KeepAlive, portMAX_DELAY);
  MQTTclient.publish(topic, msg);
  xSemaphoreGive(sema_MQTT_KeepAlive);
  
  Serial.print("MQTT message send on topic ");
  Serial.print(topic);
  Serial.print(" => ");
  Serial.println(msg);
}





/*********************************************************************************
 * SENSORS
 *********************************************************************************/

void IRAM_ATTR interruptToF() {
  eventToF = true;
}

void setupSensorsToF() {
  //Init IRQ pins
  pinMode(IRQ_1_PIN, INPUT_PULLUP);
  pinMode(IRQ_2_PIN, INPUT_PULLUP);

  //Start I2C library
  Wire.begin();
  vTaskDelay(250);

  //Check addresses
  byte error, address;
  Wire.beginTransmission(VL6180X_ADDRESS_1);
  error = Wire.endTransmission();
  if (error == 0) {
    Wire.beginTransmission(VL6180X_ADDRESS_2);
    error = Wire.endTransmission();
  } else {
    Serial.println("Error during setup ToF");
  }

  if (error != 0) {
    Serial.println("Sensor2 has same addres of sensor1, changing it now...");
    //Shutdown sensor1
    pinMode(SHUT_1_PIN, OUTPUT);
    digitalWrite(SHUT_1_PIN, LOW);
    vTaskDelay(250);

    //Init sensor2 and change address
    sensor2 = VL6180X(VL6180X_ADDRESS_1, &Wire);
    sensor2.setIICAddr(VL6180X_ADDRESS_2);

    //Start sensor 1
    digitalWrite(SHUT_1_PIN, HIGH);
    vTaskDelay(250);
  }

  //Init sensors
  sensor1 = VL6180X(VL6180X_ADDRESS_1, &Wire);
  if(sensor1.begin() != 1){
    Serial.println("FAILED TO INITALIZE 1"); //Initialize device and check for errors
  };
  vTaskDelay(5);
  sensor2 = VL6180X(VL6180X_ADDRESS_2, &Wire);
  if(sensor2.begin() != 1){
    Serial.println("FAILED TO INITALIZE 2"); //Initialize device and check for errors
  };
  vTaskDelay(5);

  sensor1.setScaling(2);
  sensor2.setScaling(2);

  //Init interrupts
  sensor1.setInterrupt(VL6180X_HIGH_INTERRUPT);
  sensor2.setInterrupt(VL6180X_HIGH_INTERRUPT);
  sensor1.rangeConfigInterrupt(VL6180X_OUT_OF_WINDOW);
  sensor2.rangeConfigInterrupt(VL6180X_OUT_OF_WINDOW);
  sensor1.rangeSetInterMeasurementPeriod(10);
  sensor2.rangeSetInterMeasurementPeriod(10);
  sensor1.setRangeThresholdValue(254,255);
  sensor2.setRangeThresholdValue(254,255);

  attachInterrupt(IRQ_1_PIN, interruptToF, FALLING);
  attachInterrupt(IRQ_2_PIN, interruptToF, FALLING);

  //Read constant and interrupt if reading is detected under 255
  sensor1.rangeStartContinuousMode();
  sensor2.rangeStartContinuousMode();

  eventToF = false;

  Serial.println("ToF sensors initialized");
}

void setupRFID() {
  SPI.begin();
  rfid.PCD_Init();

  //set key (FFFFFFFFFFFFh - this is factory default)
  for (byte i = 0; i < 6; i++) {
    key.keyByte[i] = 0xFF;
  }

  //pinMode(RFID_IRQ_PIN, INPUT_PULLUP);
  //attachInterrupt(digitalPinToInterrupt(RFID_IRQ_PIN), readCard, FALLING);

  Serial.println("RFID initialized");
}

void IRAM_ATTR interruptReedOutput() {
  eventSWOut = true;
}

void IRAM_ATTR interruptReedInput() {
  eventSWIn = true;
}

void setupLock() {
  pinMode(LOCK_PIN, OUTPUT);

  pinMode(SW_OUT_DOOR_PIN, INPUT_PULLUP);
  pinMode(SW_INP_DOOR_PIN, INPUT_PULLUP);

  attachInterrupt(SW_OUT_DOOR_PIN, interruptReedOutput, CHANGE);
  attachInterrupt(SW_INP_DOOR_PIN, interruptReedInput, CHANGE);

  Serial.println("Lock and reeds initialized");
}

void setupLED() {
  ledcSetup(REDC, 1000, 8);  // 5kHz 8 bit
  ledcSetup(GREENC, 1000, 8);
  ledcSetup(BLUEC, 1000, 8);
  ledcSetup(WHITEC, 1000, 8);

  ledcAttachPin(LED_R_PIN, REDC);
  ledcAttachPin(LED_G_PIN, GREENC);
  ledcAttachPin(LED_B_PIN, BLUEC);
  ledcAttachPin(LED_W_PIN, WHITEC);

  ledc_fade_func_install(0);

  Serial.println("RGBW LED initialized");
}

void unlock() {
  digitalWrite(LOCK_PIN, HIGH);
  vTaskDelay(2500);
  digitalWrite(LOCK_PIN, LOW);
}

void setLED(uint r, uint g, uint b, uint w) {
  //Serial.printf("Set RGBW LED to: R%d G%d B%d W%d\n", r, g, b, w);
  RGBW[0] = r;
  RGBW[1] = g;
  RGBW[2] = b;
  RGBW[3] = w;
}









// Setup runs on core 1
void setup() {
  Serial.begin(115200);
  while (!Serial);

  xTaskCreatePinnedToCore(MQTTkeepalive, "MQTTkeepalive", 7000, NULL, 9, NULL, 1);
  xTaskCreatePinnedToCore(updateLEDs, "updateLEDs", 2000, NULL, 2, NULL, 1);
  xTaskCreatePinnedToCore(rfidWatchdog, "rfidWatchdog", 10000, NULL, 9, NULL, 0);
  xTaskCreatePinnedToCore(irqWatchdog, "irqWatchdog", 10000, NULL, 8, NULL, 0);  

  Serial.println("end setup");
}

void rfidWatchdog(void * pvParameters) {
  setupRFID();

  TickType_t xLastWakeTime    = xTaskGetTickCount();
  const TickType_t xFrequency = 75; //delay for mS

  bool send = false;
  StaticJsonDocument<48> doc;

  for(;;) {
    if (rfid.PICC_IsNewCardPresent()) {
      Serial.println("Reading card");

      if (rfid.PICC_ReadCardSerial()) {
        char dest[8];
        for (int i = 0; i < rfid.uid.size; i++) {
          sprintf(&dest[i * 2], "%02X", rfid.uid.uidByte[i]);
        }
        doc["rfidtag"] = dest;
        send = true;

        // Unlock door if uid matches
        if (doc["rfidtag"] == "36B3673B") {
          setLED(0,255,0,0);
          unlock();
        } else {
          setLED(255,0,0,0);
          vTaskDelay(2000);
          setLED(0,0,0,0);
        }

        // Halt PICC
        rfid.PICC_HaltA();
        // Stop encryption on PCD
        rfid.PCD_StopCrypto1();
        Serial.println("Card reading done");
      }
    }
    
    // MQTT send
    if (send) {
      char brievenbus[56];
      serializeJson(doc, brievenbus);
      sendMessageMQTT("brievenbus", brievenbus);
    }

    send = false;
    doc.clear();

    xLastWakeTime = xTaskGetTickCount();
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
  vTaskDelete(NULL);
}

void irqWatchdog(void * pvParameters) {
  setupLock();
  unlock();
  setupSensorsToF();

  TickType_t xLastWakeTime    = xTaskGetTickCount();
  const TickType_t xFrequency = 20; //delay for mS

  long long int timerToF = 0;
  bool send = false;
  bool frontOpen = false;
  StaticJsonDocument<48> doc;

  for(;;) {
    // When letter or package passes the ToF sensor
    if (eventToF) {
      //Serial.println("eventToF");
      setLED(0,0,255,0); //blue 100%
      if (frontOpen) {
        doc["packet"] = true;
      } else {
        doc["letter"] = true;
      }
      eventToF = false;
      sensor1.clearRangeInterrupt();
      sensor2.clearRangeInterrupt();

      if ((millis() - timerToF) > 5000) {
        send = true;
        timerToF = millis();
      }
    }

    // When back door (output) opens or closes
    if (eventSWOut) {
      Serial.println("eventSWOut");
      vTaskDelay(5);
      if (digitalRead(SW_OUT_DOOR_PIN) == 1) {
        setLED(0,0,0,255); //white 100%
        doc["dooropen"] = true;
      } else {
        setLED(0,0,0,0); //off
        doc["letter"] = false;
        doc["packet"] = false;
        doc["dooropen"] = false;
      }
      send = true;
      eventSWOut = false;
    }

    // When front packet door (input) opens or closes
    if (eventSWIn) {
      Serial.println("eventSWIn");
      vTaskDelay(5);
      if (digitalRead(SW_INP_DOOR_PIN) == 1) {
        setLED(0,100,0,50);
        doc["frontopen"] = true;
        frontOpen = true;
      } else {
        doc["frontopen"] = false;
        frontOpen = false;
      }
      send = true;
      eventSWIn = false;
    }

    // MQTT send
    if (send) {
      char brievenbus[56];
      serializeJson(doc, brievenbus);
      sendMessageMQTT("brievenbus", brievenbus);
    }

    send = false;
    doc.clear();

    xLastWakeTime = xTaskGetTickCount();
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
  vTaskDelete(NULL);
}

void updateLEDs(void * pvParameters) {
  setupLED();

  for(;;) {
    for (int ch = 0; ch < 4; ch++) {
      ledc_set_fade_with_time(LEDC_HIGH_SPEED_MODE, channels[ch], RGBW[ch], 500);
      ledc_fade_start(LEDC_HIGH_SPEED_MODE, channels[ch], LEDC_FADE_NO_WAIT);
    }
    vTaskDelay(500);
  }
  vTaskDelete(NULL);
}

// This runs on core 1
void loop() {

}











