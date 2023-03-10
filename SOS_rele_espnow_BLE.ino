#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <esp_now.h>
#include <WiFi.h>
#include <LoRa.h>
#include <SPI.h>


#define SERVICE_UUID           "ffe0" // UART service UUID a5f81d42-f76e-11ea-adc1-0242ac120002
#define CHARACTERISTIC_UUID_RX "ffe1"
#define CHARACTERISTIC_UUID_TX "ffe2"

#define ss 5
#define rst 14
#define dio0 2

#define rele 35
int srele;
int spulso = 1;
uint8_t broadcastAddress[] = {0x40, 0x91, 0x51, 0x9B, 0x5B, 0xA8};

// Structure example to send data
// Must match the receiver structure
typedef struct struct_message {
  String senal;
} struct_message;

// Create a struct_message called myData
struct_message myData;

// CALLBACKS ESPNOW

void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  delay(1000);
  Serial.print("\r\nLast Packet Send Status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}



// CALLBACKS BLE

BLECharacteristic *pCharacteristic;
bool deviceConnected = false;

class MyServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
      deviceConnected = true;
    };

    void onDisconnect(BLEServer* pServer) {
      deviceConnected = false;
    }
};

class MyCallbacks: public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *pCharacteristic) {
      std::string rxValue = pCharacteristic->getValue();

      if (rxValue.length() > 0) {
        Serial.println("***");
        Serial.print("Received Value: ");

        for (int i = 0; i < rxValue.length(); i++) {
          Serial.print(rxValue[i]);
        }

        Serial.println();

        if (rxValue.find("1") != -1) {
          Serial.println("ACTIVANDO RELE");
          digitalWrite(rele, HIGH);
          delay(1000);
          digitalWrite(rele, LOW);
          delay(2000);
          Serial.println("ENVIANDO MENSAJE POR ESPNOW");
          envio_espnow();
          ESP.restart();
        }

      }
    }
};




void setup() {
  Serial.begin(115200);
  pinMode(rele, INPUT);
  WiFi.mode(WIFI_AP_STA);
  Serial.print("ESP Board MAC Address:  ");
  Serial.println(WiFi.macAddress());
 
/*  //init LoRa

  while (!Serial);
  Serial.println("LoRa Receiver");

  //setup LoRa transceiver module
  LoRa.setPins(ss, rst, dio0);

  //replace the LoRa.begin(---E-) argument with your location's frequency
  //433E6 for Asia
  //866E6 for Europe
  //915E6 for North America
  while (!LoRa.begin(915E6)) {
//    Serial.println(".");
    delay(500);
  }
  // Change sync word (0xF3) to match the receiver
  // The sync word assures you don't get LoRa messages from other LoRa transceivers
  // ranges from 0-0xFF
  //Direccion address de la oficina ---------------------> 0x42
  //Otro address -------------> 0x11
  LoRa.setSyncWord(0x11);
  Serial.println("LoRa Initializing OK!");*/

  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  // Once ESPNow is successfully Init, we will register for Send CB to
  // get the status of Trasnmitted packet
  esp_now_register_send_cb(OnDataSent);

  // Register peer
  esp_now_peer_info_t peerInfo;
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;

  // Add peer
  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add peer");
    return;
  }



  // Create the BLE Device
  BLEDevice::init("SABANA SV"); // Give it a name

  // Create the BLE Server
  BLEServer *pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  // Create the BLE Service
  BLEService *pService = pServer->createService(SERVICE_UUID);

  // Create a BLE Characteristic
  pCharacteristic = pService->createCharacteristic(
                      CHARACTERISTIC_UUID_TX,
                      BLECharacteristic::PROPERTY_NOTIFY
                    );

  pCharacteristic->addDescriptor(new BLE2902());

  BLECharacteristic *pCharacteristic = pService->createCharacteristic(
                                         CHARACTERISTIC_UUID_RX,
                                         BLECharacteristic::PROPERTY_WRITE
                                       );

  pCharacteristic->setCallbacks(new MyCallbacks());

  // Start the service
  pService->start();

  // Start advertising
  pServer->getAdvertising()->start();
  Serial.println("Waiting a client connection to notify...");
 envio_espnow();
}

void loop() {

  srele = digitalRead(rele);
  Serial.println(srele);
  
  if (srele == 1) {
    envio_espnow();
    ESP.restart();
  }

}

void envio_espnow() {
  myData.senal = spulso;
  Serial.print("Mensaje: ");
  Serial.println(myData.senal);
  // Send message via ESP-NOW
  esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &myData, sizeof(myData));

  if (result == ESP_OK) {
    Serial.println("Sent with success");
  }
  else {
    Serial.println("Error sending the data");
  }
  delay(2000);
}
