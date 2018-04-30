

/*
    Video: https://www.youtube.com/watch?v=oCMOYS71NIU
    Based on Neil Kolban example for IDF: https://github.com/nkolban/esp32-snippets/blob/master/cpp_utils/tests/BLE%20Tests/SampleNotify.cpp
    Ported to Arduino ESP32 by Evandro Copercini

   Create a BLE server that, once we receive a connection, will send periodic notifications.
   The service advertises itself as: 6E400001-B5A3-F393-E0A9-E50E24DCCA9E
   Has a characteristic of: 6E400002-B5A3-F393-E0A9-E50E24DCCA9E - used for receiving data with "WRITE" 
   Has a characteristic of: 6E400003-B5A3-F393-E0A9-E50E24DCCA9E - used to send data with  "NOTIFY"

   The design of creating the BLE server is:
   1. Create a BLE Server
   2. Create a BLE Service
   3. Create a BLE Characteristic on the Service
   4. Create a BLE Descriptor on the characteristic
   5. Start the service.
   6. Start advertising.

   In this example rxValue is the data received (only accessible inside that function).
   And txValue is the data to be sent, in this example just a byte incremented every second. 
*/
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

#include<Wire.h> //biblioteca para se comunicar com outros despositivos

BLECharacteristic *pCharacteristic;
bool deviceConnected = false;
float txValue = 0;
const int readPin = 32; // Use GPIO number. See ESP32 board pinouts
const int LED = 2; // Could be different depending on the dev board. I used the DOIT ESP32 dev board.

//std::string rxValue; // Could also make this a global var to access it in loop()

// See the following for generating UUIDs:
// https://www.uuidgenerator.net/

#define SERVICE_UUID           "6E400001-B5A3-F393-E0A9-E50E24DCCA9E" // UART service UUID
#define CHARACTERISTIC_UUID_RX "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"
#define CHARACTERISTIC_UUID_TX "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"

#define MPU 0x68 //endereço padrão do MPU

int16_t ax, ay, az,tmp; //variáveis para receber as informações brutas do MPU: acelerômetro, temperatura e giroscópio
int16_t gx, gy, gz; 

float ganho_giro = 65.0;//original 131.00 //constantes de sensibiliade do giroscópio e do acelerômetro;  a conta é (327686/250)/1000 --> 16 bits vai de -32785 a 32786, o 250 é tirado do datasheet do MPU
float ganho_acel = 16384.0; //    a conta é (327686/2)/1000 --> 16 bits vai de -32785 a 32786, o 2 é tirado do datasheet do MPU
float cte_filtro =  0.90; //original 0.98 constante do filtro complementar
float angX, angY, angZ, angX1, angY1, angZ1,pitch,roll,yaw, gx_arrumado, gy_arrumado, gz_arrumado; //variáveis para trabalhar com os valores após a aplicação das constantes

long t1; //variáveis para contar tempo
float dt;


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
        Serial.println("*********");
        Serial.print("Received Value: ");

        for (int i = 0; i < rxValue.length(); i++) {
          Serial.print(rxValue[i]);
        }

        Serial.println();

        // Do stuff based on the command received from the app
        if (rxValue.find("A") != -1) { 
          Serial.print("Turning ON!");
          digitalWrite(LED, HIGH);
        }
        else if (rxValue.find("B") != -1) {
          Serial.print("Turning OFF!");
          digitalWrite(LED, LOW);
        }

        Serial.println();
        Serial.println("*********");
      }
    }
};

void setup() {
  Serial.begin(115200);

  pinMode(LED, OUTPUT);
  Wire.begin(21,22); //as portas 21 e 22 do ESP32 são para conexão I2C (caso do MPU) um delas é para clock e a outra é para dados
  Wire.beginTransmission(MPU); //inicia transmissão
  Wire.write(0x6B); // PWR_MGMT_1 register
  Wire.write(0); // set to zero (wakes up the MPU-6050)
  Wire.endTransmission(true);

  // Create the BLE Device
  BLEDevice::init("ESP32 UART Test"); // Give it a name

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
  
}

void loop() {
  if (deviceConnected) {

    Wire.beginTransmission(MPU);
    Wire.write(0x3B); // 0x3B é o endereço no qual ele irá começar a puxar os dados
    Wire.endTransmission(false);
    Wire.requestFrom(MPU,14,true); //após definido o endereço, essa função pega os 14 dados seguintes

    ax=Wire.read()<<8|Wire.read(); // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L) //cada um desses números é o endereço onde o MPU guarda as informações brutas. 
    ay=Wire.read()<<8|Wire.read(); // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L) //A parte do <<8 é porque a informação é guardada em 2 endereços de 8 bits cada (um HIGH e outro LOW)
    az=Wire.read()<<8|Wire.read(); // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L) //Dai a necessidade de definir ax,ay,az e etc. como inteiros de 16 bits
    tmp=Wire.read()<<8|Wire.read(); // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
    gx=Wire.read()<<8|Wire.read(); // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
    gy=Wire.read()<<8|Wire.read(); // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
    gz=Wire.read()<<8|Wire.read(); // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)

  

    dt = (millis() - t1)/1000.0; //guarda qnto tempo se passou entre um loop e outro. A divisão por mil é para passar de milisegundos para segundos
    t1 = millis();

    pitch = (atan2(ax/ganho_acel, sqrt(pow(ay/ganho_acel,2) + pow(az/ganho_acel,2)))*180.0)/3.14; //pegas os valores de aceleração de x, y e z pelo métodos das tangentes (ver um dos links) 
    roll = (atan2(ay/ganho_acel, sqrt(pow(ax/ganho_acel,2) + pow(az/ganho_acel,2)))*180.0)/3.14; // aqui já está em função da constante de sensibilidade da aceleração e depois transforma o valor de radianos para graus
    yaw = (atan2(az/ganho_acel, sqrt(pow(ax/ganho_acel,2) + pow(ay/ganho_acel,2)))*180.0)/3.14;

    gx_arrumado = gx/ganho_giro; //aplica as constantes de sensibilidade do giroscópio
    gy_arrumado = gy/ganho_giro;
    gz_arrumado = gz/ganho_giro;
  
  
    angX = cte_filtro*(angX +(gx_arrumado*dt))+ (1- cte_filtro)*pitch; //aplica o filtro complementar para melhorar a saída
    angY = cte_filtro*(angY +(gy_arrumado*dt))+ (1- cte_filtro)*roll;
    angZ = cte_filtro*(angZ +(gz_arrumado*dt))+ (1- cte_filtro)*yaw;
  
    // Fabricate some arbitrary junk for now...
    txValue = angY;//analogRead(readPin) / 3.456; // This could be an actual sensor reading!

    // Let's convert the value to a char array:
    char txString[8]; // make sure this is big enuffz
    dtostrf(txValue, 1, 2, txString); // float_val, min_width, digits_after_decimal, char_buffer
    
//    pCharacteristic->setValue(&txValue, 1); // To send the integer value
//    pCharacteristic->setValue("Hello!"); // Sending a test message
    pCharacteristic->setValue(txString);
    
    pCharacteristic->notify(); // Send the value to the app!
    Serial.print("*** Sent Value: ");
    Serial.print(txString);
    Serial.println(" ***");

    // You can add the rxValue checks down here instead
    // if you set "rxValue" as a global var at the top!
    // Note you will have to delete "std::string" declaration
    // of "rxValue" in the callback function.
//    if (rxValue.find("A") != -1) { 
//      Serial.println("Turning ON!");
//      digitalWrite(LED, HIGH);
//    }
//    else if (rxValue.find("B") != -1) {
//      Serial.println("Turning OFF!");
//      digitalWrite(LED, LOW);
//    }
  }
  delay(50);
}
