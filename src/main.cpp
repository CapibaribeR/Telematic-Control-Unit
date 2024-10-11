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
uint8_t VCU_Msg[8];

void setup() {
}

void loop() {
}


void read(){

    int Supply_Voltage = (VCU_Msg[1]<< 8) | VCU_Msg[9];
    int Temp_M1    = VCU_Msg[2]-100;
    int Temp_M2    = VCU_Msg[3]-100;
    int Current_M1 = VCU_Msg[4];
    int Current_M2 = VCU_Msg[5];
    int RPM_M1     = (VCU_Msg[7]<< 8) | VCU_Msg[6];
}