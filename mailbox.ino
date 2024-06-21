#include <ArduinoJson.h>
#include <Wire.h>
#include <VL6180X.h>
#include <SPI.h>
#include <MFRC522v2.h>
#include <MFRC522DriverSPI.h>
#include <MFRC522DriverPinSimple.h>
#include <WiFi.h>
#include "driver/ledc.h"
#include "mqtt_client.h"

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
#define SW_INP_DOOR_PIN GPIO_NUM_15  //REED SW pin input door (front)
#define LED_R_PIN GPIO_NUM_27        //LED red pin
#define LED_G_PIN GPIO_NUM_33        //LED green pin
#define LED_B_PIN GPIO_NUM_26        //LED blue pin
#define LED_W_PIN GPIO_NUM_12        //LED white pin

// Replace the next variables with your SSID/Password combination
const char* ssid = "***";
const char* password = "***";

// Add your MQTT Broker details
const char* mqtt_server = "<loxberry-ip>";
const char* mqtt_user = "***";
const char* mqtt_pass = "***";
const char* mqtt_topic_send = "brievenbus";
const char* mqtt_topic_open = "brievenbus/open";
const char* mqtt_topic_reboot = "brievenbus/reboot";

// MQTT
#define NUM_OF_TOPICS (uint8_t)(2)
esp_mqtt_topic_t topics[] = {
  { mqtt_topic_open, 2 },
  { mqtt_topic_reboot, 2 },
};
const esp_mqtt_client_config_t mqtt_cfg = {
    .broker = {
      .address = {
        .hostname = mqtt_server,
        .transport = MQTT_TRANSPORT_OVER_TCP,
        .port = 1883,
      }
    },
    .credentials = {
      .username = mqtt_user,
      .authentication = {
        .password = mqtt_pass,
      }
    }
};
static esp_mqtt_client_handle_t client;

WiFiClient wifiClient;
JsonDocument docRFID;
JsonDocument docIRQ;

// ToF sensors
VL6180X sensor1;
VL6180X sensor2;

// RFID
MFRC522DriverPinSimple ss_pin(RFID_SS_PIN);
MFRC522DriverSPI driver{ss_pin};
MFRC522 rfid{driver};
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






/*********************************************************************************
 * WIFI
 *********************************************************************************/

void IRAM_ATTR WiFiEvent(WiFiEvent_t event) {
  switch (event) {
    case WIFI_EVENT_STA_CONNECTED:
      Serial.println("Connected to WiFi access point");
      break;
    case WIFI_EVENT_STA_DISCONNECTED:
      Serial.println("Disconnected from WiFi access point");
      break;
    case WIFI_EVENT_AP_STADISCONNECTED:
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

static void mqtt_event_handler(void* handler_args, esp_event_base_t base, int32_t event_id, void* event_data) {
	esp_mqtt_event_handle_t event = (esp_mqtt_event_handle_t)event_data;
	esp_mqtt_client_handle_t client = event->client;

  switch(event->event_id) {
    case MQTT_EVENT_CONNECTED: {
      Serial.println("[mqtt] conntected");
      esp_mqtt_client_subscribe_multiple(client, topics, NUM_OF_TOPICS);
      break;
    }

    case MQTT_EVENT_ERROR: {
      Serial.println("[mqtt] error");
      break;
    }

    case MQTT_EVENT_DISCONNECTED: {
      Serial.println("[mqtt] disconnected");
      break;
    }

    case MQTT_EVENT_SUBSCRIBED: {
      Serial.println("[mqtt] subscribed to all topics");
      break;
    }

    case MQTT_EVENT_UNSUBSCRIBED: {
      Serial.println("[mqtt] unsubscribed");
      break;
    }

    case MQTT_EVENT_PUBLISHED: {
      Serial.println("[mqtt] published");
      break;
    }

    case MQTT_EVENT_DATA: {
      char read_data[event->data_len];
      char read_topic[event->topic_len];
      sprintf(read_data, "%.*s", event->data_len, event->data);
      sprintf(read_topic, "%.*s", event->topic_len, event->topic);

      Serial.print("[mqtt] Received data -> ");
      Serial.println(read_data);
      Serial.print("[mqtt] @ topic -> ");
      Serial.println(read_topic);

      if (!strcmp(read_data, "1")) {
        if (!strcmp(read_topic, mqtt_topic_open)) {
          unlock();      //open mailbox
        }
        if (!strcmp(read_topic, mqtt_topic_reboot)) {
          ESP.restart(); //reboot ESP
        }
      }

      break;
    }

    case MQTT_EVENT_BEFORE_CONNECT: {
      Serial.println("[mqtt] before connect");
      break;
    }

    default: {
      Serial.println("[mqtt] default");
      break;
    }
  }
}

void sendMessageMQTT(const char* topic, const char* msg) {
  Serial.print("[mqtt] publish message -> ");
  Serial.println(msg);
  Serial.print("[mqtt] @ topic -> ");
  Serial.println(topic);
  esp_mqtt_client_publish(client, topic, msg, 0, 2, 0);
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
  pinMode(RFID_RST_PIN, OUTPUT);
  digitalWrite(RFID_RST_PIN, LOW);
 	delayMicroseconds(2);   //In order to perform a reset, the signal must be LOW for at least 100 ns => 2us is more than enough
	digitalWrite(RFID_RST_PIN, HIGH);
 	delay(50);
  
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
  ledcAttachChannel(LED_R_PIN, 1000, 8, REDC);
  ledcAttachChannel(LED_G_PIN, 1000, 8, GREENC);
  ledcAttachChannel(LED_B_PIN, 1000, 8, BLUEC);
  ledcAttachChannel(LED_W_PIN, 1000, 8, WHITEC);

  ledc_fade_func_install(0);

  Serial.println("RGBW LED initialized");
}

void unlock() {
  Serial.println("Unlock start");
  digitalWrite(LOCK_PIN, HIGH);
  vTaskDelay(2500);
  digitalWrite(LOCK_PIN, LOW);
  Serial.println("Unlock end");
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

  connectToWiFi();

  client = esp_mqtt_client_init(&mqtt_cfg);
  esp_mqtt_client_register_event(client, MQTT_EVENT_ERROR, mqtt_event_handler, client);
	esp_mqtt_client_register_event(client, MQTT_EVENT_CONNECTED, mqtt_event_handler, client);
	esp_mqtt_client_register_event(client, MQTT_EVENT_DISCONNECTED, mqtt_event_handler, client);
	esp_mqtt_client_register_event(client, MQTT_EVENT_SUBSCRIBED, mqtt_event_handler, client);
	esp_mqtt_client_register_event(client, MQTT_EVENT_UNSUBSCRIBED, mqtt_event_handler, client);
	esp_mqtt_client_register_event(client, MQTT_EVENT_PUBLISHED, mqtt_event_handler, client);
	esp_mqtt_client_register_event(client, MQTT_EVENT_DATA, mqtt_event_handler, client);
	esp_mqtt_client_register_event(client, MQTT_EVENT_BEFORE_CONNECT, mqtt_event_handler, client);
  esp_mqtt_client_start(client);

  xTaskCreatePinnedToCore(rfidWatchdog, "rfidWatchdog", 10000, NULL, 20, NULL, 0);
  xTaskCreatePinnedToCore(irqWatchdog, "irqWatchdog", 10000, NULL, 8, NULL, 0);  
  xTaskCreatePinnedToCore(updateLEDs, "updateLEDs", 2000, NULL, 2, NULL, 1);

  Serial.println("end setup");
}

void rfidWatchdog(void * pvParameters) {
  setupRFID();

  TickType_t xLastWakeTime    = xTaskGetTickCount();
  const TickType_t xFrequency = 100; //delay for mS

  bool send = false;
  docRFID.clear();

  for(;;) {
    if (rfid.PICC_IsNewCardPresent()) {
      Serial.println("Reading card");

      if (rfid.PICC_ReadCardSerial()) {
        char dest[8];
        for (int i = 0; i < rfid.uid.size; i++) {
          sprintf(&dest[i * 2], "%02X", rfid.uid.uidByte[i]);
        }
        Serial.println(dest);
        docRFID["rfidtag"] = dest;
        send = true;

        // Unlock door if uid matches
        if (docRFID["rfidtag"] == "36B3673B") {
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
      serializeJson(docRFID, brievenbus);
      sendMessageMQTT(mqtt_topic_send, brievenbus);
      docRFID.clear();
    }

    send = false;

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
  docIRQ.clear();

  for(;;) {
    // When letter or package passes the ToF sensor
    if (eventToF) {
      //Serial.println("eventToF");
      setLED(0,0,255,0); //blue 100%
      if (frontOpen) {
        docIRQ["packet"] = true;
      } else {
        docIRQ["letter"] = true;
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
        docIRQ["dooropen"] = true;
      } else {
        setLED(0,0,0,0); //off
        docIRQ["letter"] = false;
        docIRQ["packet"] = false;
        docIRQ["dooropen"] = false;
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
        docIRQ["frontopen"] = true;
        frontOpen = true;
      } else {
        docIRQ["frontopen"] = false;
        frontOpen = false;
      }
      send = true;
      eventSWIn = false;
    }

    // MQTT send
    if (send) {
      char brievenbus[56];
      serializeJson(docIRQ, brievenbus);
      sendMessageMQTT(mqtt_topic_send, brievenbus);
      docIRQ.clear();
    }

    send = false;

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
