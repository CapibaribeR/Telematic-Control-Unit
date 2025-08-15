#include <SD.h>
#include <Wire.h>
#include "esp_task_wdt.h"
#include "esp_system.h"
#include <HardwareSerial.h> 
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <TinyGPSPlus.h>
#include <SPI.h>
#include <Adafruit_GFX.h>
#include <U8g2lib.h>
#include "driver/twai.h"  

// --------------------------PINOS--------------------------//
#define CS_PIN 5
#define RX_CAN_PIN 16
#define TX_CAN_PIN 17
#define SCK_PIN 18
#define MISO_PIN 19
#define SDA_PIN 21
#define SCL_PIN 22
#define MOSI_PIN 23
#define RX_GPS_PIN 13
#define TX_GPS_PIN 14


// --------------------------IDS DAS MSGS--------------------------//
#define TEMP_CELL_BASE_ID  0x321
#define TEMP_CELL_LAST_ID  0x32A
#define VCU_MOTOR1_ID 0x331
#define VCU_MOTOR2_ID 0x332
#define BMS_ID 0x333
#define VCU_DIFF_ID 0x334
#define TOTAL_TEMPS 80


// --------------------------TIMEOUTS EM MILISSEGUNDOS--------------------------//
#define TIMEOUT_VCU_MS   300    // VCU manda a cada 100ms → 300ms é seguro
#define TIMEOUT_BMS_MS   1500   // BMS manda a cada 808ms → 1000ms é justo
#define TIMEOUT_SD_MS    1500   // SD salva a cada 100~1000ms dependendo da task



// --------------------------DISPLAY OLED--------------------------//
U8G2_SH1106_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, U8X8_PIN_NONE);


// --------------------------OBJETOS--------------------------//
TinyGPSPlus gps;
File myFileGeneral;
File myFileTemps;
String dataAtual;
String horaAtual;



// --------------------------FUNCOES MULTITASK--------------------------//


void datalogTask(void *param);
void displayTask(void *param);
QueueHandle_t dataDisplayQueue;





// --------------------------FUNCOES DATALOG--------------------------//
unsigned long start = millis();
void cria_arquivo();
void salva_no_arquivo_general();
void salva_no_arquivo_Temps();
void cronometro();
void receber_CAN();
void gpss();
void verificar_CAN_Timeout();
void collect_from_queue();
void atualizarDisplayOled();
void Send_to_Display();



// --------------------------FUNCOES DISPLAY--------------------------//
void bateria();           
void velocidade();
void temperatura();
void status();



// --------------------------VARIAVEIS GPS--------------------------//

float latitude = 0;
float longitude = 0;
int satelites = 0;
int dia = 0;
int mes = 0;
int ano = 0;
int hora = 0;
int minuto = 0;
int segundo = 0;


// --------------------------BOOLSEANAS--------------------------//
bool MOTOR1recebido = false;
bool MOTOR2recebido = false;
bool BMSrecebido = false;
bool DIFFrecebido = false;
bool GYROrecebido = false;
bool recebendoVCU = false;
bool recebendoBMS = false;
bool salvandoSD = false;
bool gpsOK = false;
bool firstLOC = false;
bool ultimo_recebendoVCU = false;
bool ultimo_recebendoBMS = false;
bool ultimo_salvandoSD = false;
bool ultimo_gpsOK = false;
bool sd_iniciado = false;
bool TEMPSpacketsReceived[10] = {false};  // Inicializa todos com false






// --------------------------CAN--------------------------//
unsigned long lastVCU1Time = 0;
unsigned long lastVCU2Time = 0;
int  ultimo_erroTWAI = -1;  
unsigned long lastBMSTime = 0;
unsigned long lastGeneralSDTime = 0;
unsigned long lastTempsSDTime = 0;
unsigned long lastTempsTime = 0;
unsigned long lastDIFFTime = 0;
int erroTWAI = 0;
twai_message_t message;
uint32_t alerts;
const char* errosTWAI[] = {        // Legenda dos erros TWAI - índice = erroTWAI
  "OK",          // 0
  "BUS_OFF",     // 1
  "ERR_PASS",    // 2
  "RX_FULL",     // 3
  "UNKNOWN"      // 4 (fallback para algo inesperado)
};








// --------------------------ESTRUTURAS (TIMER)--------------------------//
struct Timer {
  int horas;
  int minutos;
  int segundos;
  unsigned long milisegundos;
};
Timer horario_de_inicio = {0, 0, 0, 0};




// --------------------------ESTRUTURAS (DADOS DO MOTOR)--------------------------//


struct MotorData{
    float Supply_Voltage{0};           // Supply Voltage (V)
    int16_t Temp_Controller{0};        // Temperature, Controller  (°C)  //Range[0-255],Temp Range [-100°C to 155°C]
    int16_t Temp_Motor{0};             // Temperature, Motor  (°C)  //Range[0-255],Temp Range [-100°C to 155°C]
    uint16_t RPM{0};                   // Velocity, Motor  (RPM)
    float PWM{0};                      //0 - 100 % PWM
    uint8_t Current{0};                // Current, Motor  (A)

};
  MotorData Motor1, Motor2;

MotorData store_MOTOR1_MSG(twai_message_t message){
   
  
    uint16_t Voltage_Hb  = 0;    // Voltage High byte
    uint16_t Voltage_int = 0;    // Voltage int   
    Voltage_Hb = message.data[0] >> 4;                  // Band together Hb and Lb
    Voltage_int = (Voltage_Hb<<8) | message.data[1];
    Motor1.Supply_Voltage    = float(Voltage_int)/10;
    Motor1.Temp_Controller    = message.data[2]-100;
    Motor1.Temp_Motor        = message.data[3]-100;
    Motor1.RPM               = (message.data[5]<< 8) | message.data[4];
    Motor1.PWM               = (message.data[6]/255.0)*1000;
    Motor1.Current           = message.data[7];


    return Motor1;

}

MotorData store_MOTOR2_MSG(twai_message_t message){
  

    uint16_t Voltage_Hb  = 0;    // Voltage High byte
    uint16_t Voltage_int = 0;    // Voltage int   
    Voltage_Hb = message.data[0] >> 4;                  // Band together Hb and Lb
    Voltage_int = (Voltage_Hb<<8) | message.data[1];
    Motor2.Supply_Voltage    = float(Voltage_int)/10;
    Motor2.Temp_Controller    = message.data[2]-100;
    Motor2.Temp_Motor        = message.data[3]-100;
    Motor2.RPM               = (message.data[5]<< 8) | message.data[4];
    Motor2.PWM               = (message.data[6]/255.0)*1000;
    Motor2.Current           = message.data[7];


    return Motor2;


}

struct DiffStruct{
  int DIFF_LEVEL{0}; 
  float DIFF_GAIN{0};
  int apps_percentage{0};
  int steering_percentage{0};
  int speed{0};
};
DiffStruct DiffData;

DiffStruct store_DIFF_MSG(twai_message_t message){

  DiffData.DIFF_LEVEL            = message.data[0];
  DiffData.DIFF_GAIN             = float(message.data[1]) / 100;
  DiffData.apps_percentage       = message.data[2];
  DiffData.steering_percentage   = message.data[3];
  DiffData.speed                 = message.data[4];

  return DiffData;
}
  

// --------------------------ESTRUTURAS (DADOS DO BMS)--------------------------//
struct BMSData{

    int capacidade_total {0}; 
    float temp_acumulador_Display {0};
    float avg_cell_voltage {0};              
        
};
BMSData BMS;

BMSData store_BMS_MSG(twai_message_t message){
  
  
   BMS.capacidade_total          =    (message.data[0] / 2);
   BMS.temp_acumulador_Display   =    message.data[4];
   uint16_t raw_avg_cell_voltage = (message.data[5]<< 8) | message.data[6];
   BMS.avg_cell_voltage = raw_avg_cell_voltage / 10000.0;
return BMS;
}




struct BatteryTempData { 
  uint8_t temperatures[80];
};

BatteryTempData tempData;



 void store_TEMPS_MSG(twai_message_t message, uint8_t index) {
  if (index < 10) {
    memcpy(&tempData.temperatures[index * 8], message.data, 8);
  }
}

bool allTempsReceived() {
  for (int i = 0; i < 10; i++) {
    if (!TEMPSpacketsReceived[i]) return false;
  }
  return true;
}




// --------------------------ESTRUTURAS (DADOS DO GYRO)--------------------------//
struct GyroData{
    float ax{0};        
    float ay{0};          
    float Vibro{0};          
      
};
  GyroData Gyro;

GyroData store_GYRO_MSG(twai_message_t message){
  
  
   Gyro.ax    =   message.data[0]; 
   Gyro.ay    =   message.data[1]; 
   Gyro.Vibro =   message.data[2]; 


return Gyro;
}

// --------------------------ESTRUTURAS (DADOS A SEREM MOSTRADOS NO DISPLAY)--------------------------//
struct displayData{
  int speedDisplay {0};        
  int bateriaDisplay {0};          
  int temp_M1_Display {0};
  int temp_M2_Display {0};
  int temp_acumulador_Display {0};              
};
displayData receivedData;
displayData collectedData;

displayData coletarData(){
  collectedData.speedDisplay = DiffData.speed; 
  collectedData.bateriaDisplay = BMS.capacidade_total;  
  collectedData.temp_M1_Display = Motor1.Temp_Motor;   
  collectedData.temp_M2_Display = Motor2.Temp_Motor;    
  collectedData.temp_acumulador_Display = BMS.temp_acumulador_Display;

  return collectedData;


}


displayData NoData_BMS(){
  collectedData.speedDisplay = DiffData.speed; 
  collectedData.bateriaDisplay = 999;  
  collectedData.temp_M1_Display = Motor1.Temp_Motor;   
  collectedData.temp_M2_Display = Motor2.Temp_Motor;    
  collectedData.temp_acumulador_Display = 999;

  return collectedData;


}

displayData NoData_VCU(){
  collectedData.speedDisplay = 999; 
  collectedData.bateriaDisplay = BMS.capacidade_total;  
  collectedData.temp_M1_Display = 999;   
  collectedData.temp_M2_Display = 999;    
  collectedData.temp_acumulador_Display = BMS.temp_acumulador_Display;

  return collectedData;


}



// --------------------------DEFINIÇÃO DO UART EXTRA (PARA O GPS)--------------------------//
HardwareSerial gpsSerial(1); 






// --------------------------SETUP--------------------------//
void setup() {


// --------------------------INICIALIZA OS CANAIS SERIAIS UTILIZADOS--------------------------//
    Serial.begin(115200);          //Comunicação com Display
    gpsSerial.begin(9600, SERIAL_8N1, RX_GPS_PIN, TX_GPS_PIN);         //Comunicação com GPS

    delay(100);    


// --------------------------INICIALIZANDO SD--------------------------//
    
SD.begin(CS_PIN);

    delay(10);


// --------------------------INICIALIZANDO OLED DISPLAY--------------------------//

Wire.begin(21, 22);

u8g2.begin();
    

 delay(10);




// --------------------------CRIA ARQUIVO NO SD(CASO NÃO ENCONTRE)--------------------------//
    cria_arquivo();

    delay (10);


// --------------------------Configuração geral CAN (pinos TX e RX do transceptor MCP2551)--------------------------//

  twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(GPIO_NUM_17, GPIO_NUM_16, TWAI_MODE_NORMAL);
  twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS(); // Frequencia do barramento
  twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL(); // Aceita todas as mensagens de todos os dispositivos (inicialmente, mas mais tarde sera filtrado pelos ids das mensagens)

  // Inicialização do driver TWAI
  if ((twai_driver_install(&g_config, &t_config, &f_config) != ESP_OK ) || (twai_start() != ESP_OK)) {
   ESP.restart();
  } 

// --------------------------MULTITASK ESP32--------------------------//

 
dataDisplayQueue = xQueueCreate( 1, sizeof( displayData ) );
if (dataDisplayQueue == NULL) {
   ESP.restart();
}

  xTaskCreatePinnedToCore(datalogTask, "Datalog Task", 4096, NULL, 2, NULL, 0); // Prioridade alta no Núcleo 0
  xTaskCreatePinnedToCore(displayTask, "Display Task", 4096, NULL, 1, NULL, 1); // Prioridade baixa no Núcleo 1

  delay (100);

 

}

// --------------------------LOOP (VAZIO)--------------------------//

void loop() {


}

// --------------------------LOOP DATALOG--------------------------//

void datalogTask(void *param) {

while (true) {

  cronometro();
  receber_CAN();   
  gpss();
  salva_no_arquivo_general();
  salva_no_arquivo_Temps();
  atualizarDisplayOled();
  Send_to_Display();

   vTaskDelay(10 / portTICK_PERIOD_MS); // Aguarda 10 ms 
}
}


// --------------------------LOOP DISPLAY--------------------------//

void displayTask(void *param) {

while (true) {
  
  xQueueReceive(dataDisplayQueue, &receivedData, 0);
  bateria();           
  velocidade();
  temperatura();
  status();

vTaskDelay(300 / portTICK_PERIOD_MS); // Aguarda 300 ms
}
}


// --------------------------TIMER (CRONOMETRO)--------------------------//
void cronometro (){
unsigned long currentMillis = millis();

  if (currentMillis - start >= 1000) {
    start = currentMillis;
    horario_de_inicio.segundos += 1;

    if (horario_de_inicio.segundos >= 60) {
      horario_de_inicio.segundos = 0;
      horario_de_inicio.minutos += 1;
    }
    
    if (horario_de_inicio.minutos >= 60) {
      horario_de_inicio.minutos = 0;
      horario_de_inicio.horas += 1;
    }

    if (horario_de_inicio.horas >= 24) {
      horario_de_inicio.horas = 0;
    }
  }

  horario_de_inicio.milisegundos = currentMillis % 1000;
}


// --------------------------CRIAR ARQUIVO--------------------------//
void cria_arquivo() {


if (!SD.begin()) {
  // Cartão não está presente ou falhou
  return;
}


    if (!SD.exists("/dataloggeneral.csv")) 
    {
    
            myFileGeneral = SD.open("/dataloggeneral.csv", FILE_WRITE);

             while (!myFileGeneral) {
             SD.begin();  //Reinicializa caso tenha algum erro 
             myFileGeneral = SD.open("/dataloggeneral.csv", FILE_WRITE);
        } 

            horario_de_inicio = {0, 0, 0, 0};
            myFileGeneral.println("Data, Hora, Timer, SatÃ©lites, Velocidade (Kph), Longitude, Latitude, Supply Voltage (V) 1 , Supply Voltage (V) 2, Temperature Motor 1 (°C), Temperature Motor 2 (°C), Temperature Controller 1 (°C), Temperature Controller 2 (°C),Phase Current Motor 1 (A), Phase Current Motor 2 (A), Velocity Motor 1 (RPM), Velocity Motor 2 (RPM), SOC, AVG TEMP CELL, AVG Voltage");
            myFileGeneral.close();
        
    }
  
    if (!SD.exists("/datalogTemps.csv")) 
    {
    
            myFileTemps = SD.open("/datalogTemps.csv", FILE_WRITE);

             while (!myFileTemps) {
             SD.begin();  //Reinicializa caso tenha algum erro 
             myFileTemps = SD.open("/datalogTemps.csv", FILE_WRITE);
        } 

            horario_de_inicio = {0, 0, 0, 0};
            myFileTemps.close();
        
    }
    


}



// --------------------------RECEBER MSGS CAN--------------------------//
void receber_CAN() {

  while (twai_receive(&message, 0) == ESP_OK){    // Recebe MSG do barramento

    switch (message.identifier){                  //Faz a leitura de acordo com os IDs das mensagens
                 
        case VCU_MOTOR1_ID:
            Motor1 = store_MOTOR1_MSG(message);  //Armazena na struct
            MOTOR1recebido = true;              //Indica que as informações ja foram armazenadas e podem ser salvas no arquivo
            lastVCU1Time = millis();
            break;
        
        case VCU_MOTOR2_ID:
            Motor2 = store_MOTOR2_MSG(message);  //Armazena na struct
            MOTOR2recebido = true;              //Indica que as informações ja foram armazenadas e podem ser salvas no arquivo
            lastVCU2Time = millis();
            break;
        
        case BMS_ID:
            BMS = store_BMS_MSG(message);      //Amarzena na struct
            BMSrecebido = true;                //Indica que as informações ja foram armazenadas e podem ser salvas no arquivo
            lastBMSTime = millis();
            break;

        case VCU_DIFF_ID:
            DiffData = store_DIFF_MSG(message);  //Armazena na struct
            DIFFrecebido = true;              //Indica que as informações ja foram armazenadas e podem ser salvas no arquivo
            lastDIFFTime = millis();
            break;



            default:
  if (message.identifier >= TEMP_CELL_BASE_ID && message.identifier <= TEMP_CELL_LAST_ID) {
    uint8_t packetIndex = message.identifier - TEMP_CELL_BASE_ID;

    // Copia os 8 bytes para a posição correta
    store_TEMPS_MSG(message, packetIndex);  // Função armazena os 8 bytes no índice certo
    TEMPSpacketsReceived[packetIndex] = true;


  }

  break;


    }

}

}


// --------------------------GPS--------------------------//


void gpss(){
  while (gpsSerial.available()) {  
      char c = gpsSerial.read();
      gps.encode(c); // Processa cada caractere do GPS
  }

  if (gps.location.isUpdated() && gps.date.isUpdated() && gps.time.isUpdated()) {


    if(firstLOC == false){
      latitude = gps.location.lat();
      longitude = gps.location.lng();
      firstLOC = true;
    }



     gpsOK = true;
      satelites = gps.satellites.value();
      dia = gps.date.day();
      mes = gps.date.month();
      ano = gps.date.year();
      hora = (gps.time.hour() - 3);
      minuto = gps.time.minute();
      segundo = gps.time.second();
      

      if (hora < 0){

          hora = 24 + hora;
          dia = dia - 1;

      }

      if (DiffData.speed>3) {  


       latitude = gps.location.lat();
       longitude = gps.location.lng();
       
   
       }


}

else{
  gpsOK = false;
}

}


// --------------------------SALVAR TUDO NO SD--------------------------//
void salva_no_arquivo_general() {

     if (BMSrecebido && MOTOR1recebido && MOTOR2recebido) {         //Salva apenas se todos os pacotes forem armazenados nas estruturas

  myFileGeneral = SD.open("/dataloggeneral.csv", FILE_APPEND);

    if (myFileGeneral) {


        unsigned long currentMillis = millis();

           
            String timerString = String(horario_de_inicio.horas) + ":" +
                                 String(horario_de_inicio.minutos) + ":" +
                                 String(horario_de_inicio.segundos) + ":" +
                                 String(horario_de_inicio.milisegundos);  
            
            
                                 dataAtual = String(dia) + "/" + String(mes) + "/" + String(ano);
                                 horaAtual = String(hora) + ":" + String(minuto) + ":" + String(segundo);
           
        // Escreve data, hora e tempo no arquivo
        myFileGeneral.print(dataAtual); 
        myFileGeneral.print(","); 
        myFileGeneral.print(horaAtual); 
        myFileGeneral.print(","); 
        myFileGeneral.print(timerString); 
        myFileGeneral.print(",");

        // Escreve os dados no arquivo

        myFileGeneral.print(satelites);
        myFileGeneral.print(",");
        myFileGeneral.print(DiffData.speed);
        myFileGeneral.print(",");
        myFileGeneral.print(longitude,6);
        myFileGeneral.print(",");
        myFileGeneral.print(latitude,6);
        myFileGeneral.print(",");
        myFileGeneral.print(Motor1.Supply_Voltage);
        myFileGeneral.print(",");
        myFileGeneral.print(Motor2.Supply_Voltage);
        myFileGeneral.print(",");
        myFileGeneral.print(Motor1.Temp_Motor);
        myFileGeneral.print(",");
        myFileGeneral.print(Motor2.Temp_Motor);
        myFileGeneral.print(",");
        myFileGeneral.print(Motor1.Temp_Controller);
        myFileGeneral.print(",");
        myFileGeneral.print(Motor2.Temp_Controller);
        myFileGeneral.print(",");
        myFileGeneral.print(Motor1.Current);
        myFileGeneral.print(",");
        myFileGeneral.print(Motor2.Current);
        myFileGeneral.print(",");
        myFileGeneral.print(Motor1.RPM);
        myFileGeneral.print(",");
        myFileGeneral.print(Motor2.RPM);
        myFileGeneral.print(",");
        myFileGeneral.print(BMS.capacidade_total);
        myFileGeneral.print(",");
        myFileGeneral.print(BMS.temp_acumulador_Display);
        myFileGeneral.print(",");
        myFileGeneral.print(BMS.avg_cell_voltage);
        myFileGeneral.print(",");
        myFileGeneral.print(DiffData.DIFF_LEVEL);
        myFileGeneral.print(",");
        myFileGeneral.print(DiffData.DIFF_GAIN);
        myFileGeneral.print(",");
        myFileGeneral.print(DiffData.apps_percentage);
        myFileGeneral.print(",");
        myFileGeneral.print(DiffData.steering_percentage);
        myFileGeneral.print(",");
        myFileGeneral.println();



        myFileGeneral.close();
      
              lastGeneralSDTime = millis();

         MOTOR1recebido   = false;
         MOTOR2recebido   = false;
         BMSrecebido      = false;



        }

      }

    }


      void salva_no_arquivo_Temps(){


        if (allTempsReceived()) {
  myFileTemps = SD.open("/datalogTemps.csv", FILE_APPEND);

 
    if (myFileTemps) {
     
        unsigned long currentMillis = millis();

            String timerString = String(horario_de_inicio.horas) + ":" +
                                 String(horario_de_inicio.minutos) + ":" +
                                 String(horario_de_inicio.segundos) + ":" +
                                 String(horario_de_inicio.milisegundos);  
            
            
                                 dataAtual = String(dia) + "/" + String(mes) + "/" + String(ano);
                                 horaAtual = String(hora) + ":" + String(minuto) + ":" + String(segundo);
           
        // Escreve data, hora e tempo no arquivo
        myFileTemps.print(dataAtual); 
        myFileTemps.print(","); 
        myFileTemps.print(horaAtual); 
        myFileTemps.print(","); 
        myFileTemps.print(timerString); 
        myFileTemps.print(",");



      for (int i = 0; i < TOTAL_TEMPS; i++) {
        myFileTemps.print(tempData.temperatures[i]);
        if (i < TOTAL_TEMPS - 1) {
          myFileTemps.print(",");
      }

    }

      lastTempsSDTime = millis();
          myFileTemps.close();

  for (int i = 0; i < 10; i++) {
    TEMPSpacketsReceived[i] = false;
  }  

}

        }
      }

// --------------------------ATUALIZA OLED DISPLAY--------------------------//

void atualizarDisplayOled() {

  verificar_CAN_Timeout();

    bool houveMudanca =
    recebendoVCU != ultimo_recebendoVCU ||
    recebendoBMS != ultimo_recebendoBMS ||
    salvandoSD   != ultimo_salvandoSD   ||
    gpsOK        != ultimo_gpsOK        ||
    erroTWAI     != ultimo_erroTWAI;

  if (!houveMudanca) return; // Nada mudou, não redesenha

  // Atualiza as variáveis "último"
  ultimo_recebendoVCU = recebendoVCU;
  ultimo_recebendoBMS = recebendoBMS;
  ultimo_salvandoSD   = salvandoSD;
  ultimo_gpsOK        = gpsOK;
  ultimo_erroTWAI     = erroTWAI;


  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_6x10_tf);

  // Linha 1 Status da VCU
  u8g2.drawStr(0, 10, "CAN VCU:");
  u8g2.drawStr(110, 10, recebendoVCU ? "OK" : "NAO");

  // Linha 2 - Status do BMS
  u8g2.drawStr(0, 20, "CAN BMS:");
  u8g2.drawStr(110, 20, recebendoBMS ? "OK" : "NAO");

  
  // Linha 3 - Status do CANBUS
  u8g2.drawStr(0, 30, "CANBUS:");

  if (erroTWAI == 0) {u8g2.drawStr(110, 30, errosTWAI[erroTWAI]);}
  else {u8g2.drawStr(80, 30, errosTWAI[erroTWAI]);}


  // Linha 4 - Salvando no SD
  u8g2.drawStr(0, 40, "Salvando no SD:");
  u8g2.drawStr(110, 40, salvandoSD ? "OK" : "NAO");


   // Linha 5 - GPS funcionando
  u8g2.drawStr(0, 50, "GPS Fix:");
  u8g2.drawStr(110, 50, gpsOK ? "OK" : "NAO");

  u8g2.sendBuffer();
}



// -------------------COLETA DADOS DA FILA PARA POR NO SD  -------------------//
void Send_to_Display(){

if (recebendoVCU && recebendoBMS) {
  collectedData = coletarData();
}
else if (recebendoVCU && !recebendoBMS) {
  collectedData = NoData_BMS();  // Mostra motor, mascara BMS
}
else if (!recebendoVCU && recebendoBMS) {
  collectedData = NoData_VCU();  // Mostra BMS, mascara motor
}

  xQueueOverwrite(dataDisplayQueue, &collectedData); 

}

// --------------------------VERIFICA SE ESTA RECEBENDO CAN--------------------------//
void verificar_CAN_Timeout() {
   // Timeout para VCU (Motor1 e Motor2)
  if ((millis() - lastVCU1Time > TIMEOUT_VCU_MS) || 
      (millis() - lastVCU2Time > TIMEOUT_VCU_MS) ||
      (millis() - lastDIFFTime > TIMEOUT_VCU_MS)) {
    recebendoVCU = false;
  } else {
    recebendoVCU = true;
  }

  // Timeout para BMS (pode ser pelo BMS ou pelos 10 pacotes de temperatura)
  if ((millis() - lastBMSTime > TIMEOUT_BMS_MS) || 
      (millis() - lastTempsTime > TIMEOUT_BMS_MS)) {
    recebendoBMS = false;
  } else {
    recebendoBMS = true;
  }

  // Timeout para SD (controle se está salvando ou travou)
  if ((millis() - lastGeneralSDTime > TIMEOUT_SD_MS) ||
      (millis() - lastTempsSDTime > TIMEOUT_SD_MS)) {
    salvandoSD = false;
  } else {
    salvandoSD = true;
  }
}


// --------------------------VERIFICA FUNCIONAMENTO DO BARRAMENTO CAN--------------------------//


void verificar_TWAI_status(){
if (twai_read_alerts(&alerts, pdMS_TO_TICKS(100)) == ESP_OK) {
    if (alerts & TWAI_ALERT_BUS_OFF) {
      erroTWAI = 1;
      twai_stop();
      vTaskDelay(100 / portTICK_PERIOD_MS);      
      twai_start(); // só reinicia se o problema físico for resolvido
    } else if (alerts & TWAI_ALERT_ERR_PASS) {
      erroTWAI = 2;
    } else if (alerts & TWAI_ALERT_RX_QUEUE_FULL) {
      erroTWAI = 3;
    } else {
      erroTWAI = 0;  // Nenhum erro relevante
    }
  } else {
    erroTWAI = 4;  // Erro ao ler os alerts
  }

}






// ------------------------------------FUNÇÕES DO DISPLAY PRINCIPAL------------------------------------//


//Função que posiciona e rotaciona o ponteiro  (pronto)
void ponteiro(int ang){
  
  int angH = ang/256;
  int angL = ang%256;
  byte Cobertura[] = {0xAA,0x9C,0x00,0x04,0x00,0x75,0x00,0x0B,0x02,0xC5,0x02,0x61,0x00,0x75,0x00,0x0B,0xCC,0x33,0xC3,0x3C};
  byte Ponteiro[] = {0xAA,0x9E,0x00,0x00,0x03,0x00,0xE5,0x00,0x65,0x00,0xFD,0x00,0xEE,0x00,0xF1,0x00,0x82,angH,angL,0x00,0xED,0x00,0x85,0xCC,0x33,0xC3,0x3C};
  Serial.write(Cobertura,sizeof(Cobertura)); // Cobre a posição anterior do ponteiro com uma imagem 
  Serial.write(Ponteiro,sizeof(Ponteiro));  // Posiciona o ponteiro;
}

//Função que envia a velocidade de maneira digital e analogica (pronto)
void velocidade (){

  int speed = receivedData.speedDisplay;

  int c = speed/100;
  int d = speed/10 - 10*c;
  int u = speed%10;
  byte Velocidade[] = {0xAA,0x98,0x01,0x09,0x00,0x92,0x00,0x82,0x05,0xFF,0xFF,0x00,0x1F,c+48,d+48,u+48,0xCC,0x33,0xC3,0x3C};
  if (speed <= 10){
    ponteiro(0);}
  else if (speed > 10 && speed <140){
    ponteiro((4*speed)-40);} //conversão para que o angulo seja aproximadamente equivalente a velocidade no velocimetro
  else {
    ponteiro(522);}
    Serial.write(Velocidade,sizeof(Velocidade)); // Envia a velocidade digital para o display
 // //Serial.println(speed);

}

// Função que imprime no display a porcentagem da bateria e a controla de maneira gráfica  (pronto)
void bateria (){
  int bat = receivedData.bateriaDisplay;
  byte Palette1[] = {0xAA,0x40,0x30,0x30,0x30,0xFF,0xFF,0xFF,0xCC,0x33,0xC3,0x3C};
  Serial.write(Palette1,sizeof(Palette1));  // Define o FG como cinza e BG como branco
  
  int c = bat/100;
  int d = bat/10 - 10*c;
  int u = bat%10;
  byte Cobertura[] = {0xAA,0x5B,0x00,0x12,0x00,0x13,0x00,0x75,0x00,0x31,0x00,0x16,0x00,0xD4,0x00,0x5B,0x00,0xF6,0x00,0x16,0x00,0xAE,0x00,0x5B,0x00,0xD0,0x00,0x16,0x00,0x88,0x00,0x5B,0x00,0xAA,0x00,0x16,0x00,0x62,0x00,0x5B,0x00,0x84,0xCC,0x33,0xC3,0x3C};
  Serial.write(Cobertura,sizeof(Cobertura));      // Limpa a porcetagem e o valor gráfico da bateria
  byte Porcentagem[] = {0xAA,0x98,0x00,0x10,0x00,0x10,0x00,0x82,0x04,0xFF,0xFF,0x00,0x1F,c+48,d+48,u+48,0x20,0x25,0xCC,0x33,0xC3,0x3C};
  Serial.write(Porcentagem,sizeof(Porcentagem));      // Envia a porcentagem da bateria ao display

  byte Palette2[] = {0xAA,0x40,0x9B,0xCA,0x41,0xE6,0x4D,0x3D,0xCC,0x33,0xC3,0x3C};
  Serial.write(Palette2,sizeof(Palette2));  //Define o FG como verde e o BG como vermelho
  
  if (bat > 80 && bat <= 100){              //Posiciona os quadrados da bateria
    byte Quadrados[] = {0xAA,0x5B,0x00,0x16,0x00,0xD4,0x00,0x5B,0x00,0xF6,0x00,0x16,0x00,0xAE,0x00,0x5B,0x00,0xD0,0x00,0x16,0x00,0x88,0x00,0x5B,0x00,0xAA,0x00,0x16,0x00,0x62,0x00,0x5B,0x00,0x84,0xCC,0x33,0xC3,0x3C};
    Serial.write(Quadrados,sizeof(Quadrados));}
  else if (bat > 60 && bat <= 80){
    byte Quadrados[] = {0xAA,0x5B,0x00,0x16,0x00,0xD4,0x00,0x5B,0x00,0xF6,0x00,0x16,0x00,0xAE,0x00,0x5B,0x00,0xD0,0x00,0x16,0x00,0x88,0x00,0x5B,0x00,0xAA,0xCC,0x33,0xC3,0x3C};
    Serial.write(Quadrados,sizeof(Quadrados));}
  else if (bat > 40 && bat <= 60){
    byte Quadrados[] = {0xAA,0x5B,0x00,0x16,0x00,0xD4,0x00,0x5B,0x00,0xF6,0x00,0x16,0x00,0xAE,0x00,0x5B,0x00,0xD0,0xCC,0x33,0xC3,0x3C};
    Serial.write(Quadrados,sizeof(Quadrados));}
  else if (bat > 20 && bat <= 40){
    byte Quadrados[] = {0xAA,0x5B,0x00,0x16,0x00,0xD4,0x00,0x5B,0x00,0xF6,0xCC,0x33,0xC3,0x3C};
    Serial.write(Quadrados,sizeof(Quadrados));}
  else if (bat > 0 && bat <= 20){
    byte Quadrados[] = {0xAA,0x5A,0x00,0x16,0x00,0xD4,0x00,0x5B,0x00,0xF6,0xCC,0x33,0xC3,0x3C};
    Serial.write(Quadrados,sizeof(Quadrados));}
}

    // Função que imprime no display as temperaturas do acumulador e dos motores (pronto)
void temperatura (){
  


int temp_acum = (receivedData.temp_acumulador_Display);
int temp_motor1 = (receivedData.temp_M1_Display);
int temp_motor2 = (receivedData.temp_M2_Display);

  byte Palette1[] = {0xAA,0x40,0x20,0x20,0x20,0xFF,0xFF,0xFF,0xCC,0x33,0xC3,0x3C};
  Serial.write(Palette1,sizeof(Palette1)); // Define o FG como cinza e BG como branco
  byte Cobertura[] = {0xAA,0x5B,0x01,0xA9,0x00,0x05,0x01,0xC6,0x00,0x6A,0xCC,0x33,0xC3,0x3C};
  Serial.write(Cobertura,sizeof(Cobertura));    // Limpa os valores de temperatura
  
  int da = temp_acum/10;
  int ua = temp_acum%10;
  byte Acumulador[] = {0xAA,0x98,0x01,0xAC,0x00,0x0C,0x00,0x82,0x02,0xFF,0xFF,0x00,0x1F,da+48,ua+48,0xCC,0x33,0xC3,0x3C};
  Serial.write(Acumulador,sizeof(Acumulador));  // Envia a temperatura do acumulador
  int d1 = temp_motor1/10;
  int u1 = temp_motor1%10;
  byte Motor1[] = {0xAA,0x98,0x01,0xAC,0x00,0x2C,0x00,0x82,0x02,0xFF,0xFF,0x00,0x1F,d1+48,u1+48,0xCC,0x33,0xC3,0x3C};
  Serial.write(Motor1,sizeof(Motor1));          //Envia a temperatura do Motor1
  int d2 = temp_motor2/10;
  int u2 = temp_motor2%10;
  byte Motor2[] = {0xAA,0x98,0x01,0xAC,0x00,0x50,0x00,0x82,0x02,0xFF,0xFF,0x00,0x1F,d2+48,u2+48,0xCC,0x33,0xC3,0x3C};
  Serial.write(Motor2,sizeof(Motor2));         //Envia a temperatura do Motor2
}
   
  //Função que envia o status do veiculo
void status (){
   
  byte Cobertura[] = {0xAA,0x5B,0x01,0x35,0x00,0xDC,0x01,0x96,0x00,0xFC,0xCC,0x33,0xC3,0x3C};
  Serial.write(Cobertura,sizeof(Cobertura));  //Limpa o status 
  byte Status[] = {0xAA,0x98,0x01,0x35,0x00,0xDC,0x00,0x82,0x03,0xFF,0xFF,0x00,0x1F,0x50,0x72,0x6F,0x6E,0x74,0x6F,0xCC,0x33,0xC3,0x3C};
  Serial.write(Status,sizeof(Status));      //Envia o status: Pronto
  }




//Descrição do comandos:
  //  byte para escrever um texto:
  //  byte vetor[] = {0xAA,0x98,Xh,Xl,Yh,Yl,Lib_ID,C_Mode,C_dots,Fcolor,Fcolor,Bcolor,Bcolor,String,0xCC,0x33,0x3C,0xC3};
