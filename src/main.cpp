/***
 * Capibarib E-racing
 * Federal University of Pernambuco (UFPE)
 * Group Area: Data Acquisition
 
 * This file contains the code for the cars TCU (TELEMATIC CONTROL UNIT) that unites the:
 * 1. CAN communication with the VCU and the BMS
 * 2. The Data Logger 
 * 3. The vehicle's Display
 ***/

#include <Arduino.h>
#include <CAN.h>
#include <GeneralCAN.h>

/*===================================== COMMUNICATION PORTS (STM32 F746ZG) =====================================*/
#define TX_GPIO_NUM   4         // Connects to CTX
#define RX_GPIO_NUM   0         // Connects to CRX
#define CAN_FREQ      500E3     // CAN frequency in Kbps



uint8_t VCU_Msg[8];
MotorData Motor;
void read(int packetSize);

/*====================================== INITIALIZATION ======================================*/
void setup() {
    // Start Serial
    Serial.begin(115200);
  
    // CAN Setup
    CAN.setPins(RX_GPIO_NUM, TX_GPIO_NUM);
    CAN.begin(CAN_FREQ);

    // Callback Function
    CAN.onReceive(read);

}

/*=========================================== MAIN LOOP ===========================================*/
void loop() {

}

/* Function Called every time a CAN msg is received*/
void read(int packetSize){
    int i{0};
    uint8_t msg[8];
    
    // Reads the CAN datafield and stores it
    while(CAN.available()){
        msg[i] = CAN.read();
        i++;
    }

    switch (CAN.packetId()){
        case VCU_MOTOR_ID:
            Motor = store_VCU_MSG(msg);
            Print_Datafield(Motor);
            break;
        
        case VCU_GYRO_ID:
            break;
        
        case BMS_ID:

            break;
    }
}
