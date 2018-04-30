#define MPU 0x68 //endereço padrão do MPU
#include<Wire.h> //biblioteca para se comunicar com outros despositivos
int16_t ax, ay, az,tmp; //variáveis para receber as informações brutas do MPU: acelerômetro, temperatura e giroscópio
int16_t gx, gy, gz; 

float ganho_giro = 131.0; //constantes de sensibiliade do giroscópio e do acelerômetro;  a conta é (327686/250)/1000 --> 16 bits vai de -32785 a 32786, o 250 é tirado do datasheet do MPU
float ganho_acel = 16384.0; //    a conta é (327686/2)/1000 --> 16 bits vai de -32785 a 32786, o 2 é tirado do datasheet do MPU
float cte_filtro =  0.98; //constante do filtro complementar
float angX, angY, angZ, angX1, angY1, angZ1,pitch,roll,yaw, gx_arrumado, gy_arrumado, gz_arrumado; //variáveis para trabalhar com os valores após a aplicação das constantes

long t1; //variáveis para contar tempo
float dt;



void setup() {
  Wire.begin(21,22); //as portas 21 e 22 do ESP32 são para conexão I2C (caso do MPU) um delas é para clock e a outra é para dados
  Wire.beginTransmission(MPU); //inicia transmissão
  Wire.write(0x6B); // PWR_MGMT_1 register
  Wire.write(0); // set to zero (wakes up the MPU-6050)
  Wire.endTransmission(true);
  Serial.begin(115200); //inicia o serial
}

void loop() {
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

  Serial.print("ang X: ");Serial.print(angX); //imprime os ângulos
  Serial.print(" | ang Y: ");Serial.print(angY);
  Serial.print(" | ang Z: ");Serial.println(angZ);
  delay(20); // da um delay para melhorar a leitura

}
