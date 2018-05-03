#include <Wire.h> //Biblioteca para se comunicacao I2C

#define MPU 0x68 //Endereco para comunicacao I2C (Sec 9.2 Datasheet)
#define M_PI 3.1415926 //Definicao de pi

//Variaveis da leitura do acelerometro e do giroscopio para achar os angulos

int angX_int, angY_int;

float angX = 0.0, angY = 0.0, angZ = 0.0; //Angulos em x, y e z respectivamente

int16_t acelX, acelY, acelZ; //Leitura do acelerometro em x, y e z respec.
float gForcaX, gForcaY, gForcaZ; //Forca g sendo aplicada nos eixos x, y e z respec.

int16_t girX, girY, girZ; //Leitura do giroscopio em x, y e z respec.
float rotX, rotY, rotZ; //Rotacao nos eixos x, y e z respec.

int16_t temp; //Leitura da temperatura (nao utilizada)

unsigned long t_antes; //Para determinacao do tempo de leitura dos dados
float dt; //Tempo transcorrido

float off_girX, off_girY, off_girZ, off_acelX, off_acelY, off_acelZ; //Offset inicial do giroscopio e acelerometro

float alpha = 0.94; //Constante do Filtro Complementar determinado por tentativa e erro

//Coneccoes e inicializacoes
void setup() {
  Serial.begin(19200); //Inicia o serial com uma taxa baud de 115200 bits por segundo
  Wire.begin(21, 22); //As portas 21 e 22 sao para comunicacao I2C, sendo a 21 SDA (dados) e a 22 SCL (clock)
  setupMPU(); //Configura o MPU
  offset(); //Calculo do offset do giroscopio e da inclinicao inicial
}

//Loop de calculo dos angulos
void loop() {
  leituraDados(); //Leitura dos dados brutos do acelerometro e giroscopio

  dt = (millis() - t_antes) / 1000.0; //A diferenca do tempo atual e o anterior eh o tempo transcorrido entre os dados. A divisao por 1000 eh para transformar de milisegundos para segundos.
  t_antes = millis(); //Salva o tempo atual para subtracao na proxima interacao

  //Compensacao das leituras do giroscopio e acelerometro com seus respectivos offsets
  girX -= off_girX;
  girY -= off_girY;
  girZ -= off_girZ;
  acelX -= off_acelX;
  acelY -= off_acelY;
  acelZ -= off_acelZ;

  //Transformacao dos dados do giroscopio para rotacao nos eixos
  //Sensibilidade LSB/dps de 131 para o giroscopio na configuracao FS_SEL = 0 (Sec 6.1 Datasheet)
  rotX = girX / 131.0; //roll
  rotY = girY / 131.0; //pitch
  rotZ = girZ / 131.0; //yaw
  
  //Calculo dos angulos a partir de decomposicao do vetor de aceleracao
  //Multiplicacao de 180/M_PI eh para transformar de radianos para graus
  gForcaX = (atan2(acelY , sqrt(pow(acelX, 2) + pow(acelZ, 2))) * 180.0) / M_PI;
  gForcaY = (atan2(-1*acelX , sqrt(pow(acelY, 2) + pow(acelZ, 2))) * 180.0) / M_PI;
  gForcaZ = (atan2(sqrt(pow(acelX, 2) + pow(acelY, 2)) , acelZ) * 180.0) / M_PI;
  
  //Filtro complementar
  angX = alpha * (angX + (rotX * dt)) + (1 - alpha) * gForcaX;
  angY = alpha * (angY + (rotY * dt)) + (1 - alpha) * gForcaY;
  angZ = alpha * (angZ + (rotZ * dt)) + (1 - alpha) * gForcaZ;

  //Imprimi os angulos
  Serial.print("ang X: "); Serial.print(angX);
  Serial.print(" | ang Y: "); Serial.print(-angY);
  Serial.print(" | ang Z: ");Serial.print(angZ);

  angX_int = int(abs(angX) * 10);
  angY_int = int(abs(angY) * 10);

  if(angX_int > 900) { angX_int = 900; }
  if(angY_int > 900) { angY_int = 900; }

  Serial.print(" | angint X: "); Serial.print(angX_int);
  Serial.print(" | angint Y: "); Serial.print(angY_int);

  char txString[8];
  sprintf(txString, "%03i,%03i", angX_int, angY_int);

  Serial.print(" | tx: "); Serial.println(txString);

  //Determinacao do melhor alpha entre 0.91 e 0.98
  /*
  float angY_alpha[8];
  String angY_name[8] = {"91: ", " | 92: ", " | 93: ", " | 94: ", " | 95: ", " | 96: ", " | 97: ", " | 98: "};
  for(int i = 0; i < 8; i++) {
    alpha = 0.91 + (i / 100.0);
    angY_alpha[i] = alpha * (angY_alpha[i] + (rotY * dt)) + (1 - alpha) * gForcaY;
    if(i != 7){
      Serial.print(angY_name[i]); Serial.print(-angY_alpha[i]);
    }
    else{
      Serial.print(angY_name[i]); Serial.println(-angY_alpha[i]);
    }
  } */

  delay(10); //Delay para melhorar a leitura e nao truncar o serial
}

//Setup inicial do MPU
void setupMPU() {
  Wire.beginTransmission(MPU); //Inicia transmissao
  Wire.write(0x6B); //Acessando registro PWR_MGMT_1 (Power Management) (Sec 3 Register Map)
  Wire.write(0b00000000); //Setando para zero o registro SLEEP (ver nota da Sec 4 Register Map)
  Wire.endTransmission(true); //Fim da transmissao
  Wire.beginTransmission(MPU); //Inicia transmissao
  Wire.write(0x1B); //Acessando registro GYRO_CONFIG (Gyroscope Configuration) (Sec 3 Register Map)
  Wire.write(0b00000000); //Setando escala do giroscopio para +/- 250 dps (FS_SEL = 0 | Sec 6.1 Datasheet)
  Wire.endTransmission(true); //Fim da transmissao
  Wire.beginTransmission(MPU); //Inicia transmissao
  Wire.write(0x1C); //Acessando registro ACCEL_CONFIG (Accelerometer Configuration) (Sec 3 Register Map)
  Wire.write(0b00000000); //Setando escala do acelerometro para +/- 2 g (AFS_SEL = 0 | Sec 6.2 Datasheet)
  Wire.endTransmission(true); //Fim da transmissao
}

//Realiza a leitura dos dados brutos do acelerometro e do giroscopio
void leituraDados() {
  Wire.beginTransmission(MPU); //Inicia transmissao
  Wire.write(0x3B); //Comeca leitura dos dados do acelerometro (Sec 3 Register Map)
  Wire.endTransmission(false); //Reinicia a transmissao, mantendo a coneccao ativa
  Wire.requestFrom(MPU, 14, true); //Pede os registros de 3B - 48 (Sec 3 Register Map) e para a transmissao

  //Leitura dos registros de 3B - 40 (Sec 3 Register Map)
  acelX = Wire.read() << 8 | Wire.read(); // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)
  acelY = Wire.read() << 8 | Wire.read(); // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
  acelZ = Wire.read() << 8 | Wire.read(); // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)

  //Leitura dos registros 41 e 42
  temp = Wire.read() << 8 | Wire.read(); // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)

  //Leitura dos registros de 3B - 40 (Sec 3 Register Map)
  girX = Wire.read() << 8 | Wire.read(); // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
  girY = Wire.read() << 8 | Wire.read(); // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
  girZ = Wire.read() << 8 | Wire.read(); // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)
}

//Calcula os offsets do giroscopio e do acelerometro do MPU
void offset() {

  int num_leituras = 100;

  //Inicializacao dos offsets do giroscopio e acelerometro
  off_girX = 0.0;
  off_girY = 0.0;
  off_girZ = 0.0;
  off_acelX = 0.0;
  off_acelY = 0.0;
  off_acelZ = 0.0;

  //Realiza uma leitura e aguarda, pois nas primeiras leituras o MPU ainda esta se adequando
  leituraDados();
  delay(100);

  //Realiza N leituras e as soma para tirar a media depois
  for (int i = 0; i < num_leituras; i++) {
    leituraDados();
    off_girX += girX;
    off_girY += girY;
    off_girZ += girZ;
    off_acelX += acelX;
    off_acelY += acelY;
    off_acelZ += acelZ;
    delay(10);
  }

  //Faz a media dos N dados lidos e defini os offsets
  //Para a calibracao com o MPU parado na posicao com x e y no plano "zero",
  //o giroscopio nao deve ler rotacao (compensacao para zero)
  //enquanto o acelerometro nao deve ler forca g em x e y (compensacao para zero)
  //mas deve ser a forca g maxima em z (compensacao em 16384 (Sec 6.2 Datasheet))
  off_girX = off_girX / num_leituras;
  off_girY = off_girY / num_leituras;
  off_girZ = off_girZ / num_leituras;
  off_acelX = off_acelX / num_leituras;
  off_acelY = off_acelY / num_leituras;
  off_acelZ = (off_acelZ / num_leituras) - 16384.0;
}

