#include <Arduino.h>

#define VCU_MOTOR_ID    1       //
#define VCU_GYRO_ID     2       //
#define BMS_ID          3       //
#define TCU_ID          4       //


/* Struct that contains VCU Datafield */
struct MotorData{
    float Supply_Voltage{0};        // Supply Voltage (V)
    uint8_t Temp_M1{0};             // Temperature, Motor 1 (°C)
    uint8_t Temp_M2{0};             // Temperature, Motor 2 (°C)
    uint8_t Current_M1{0};          // Phase Current, Motor 1 (°C)
    uint8_t Current_M2{0};          // Phase Current, Motor 2 (°C)
    uint16_t RPM_M1{0};             // Velocity, Motor 1 (RPM)
};

/* Struct that contains BMS Datafield */
struct BMSData{

};


void get() {
  
    int id;
    switch (id){
        case VCU_MOTOR_ID:

            break;
        
        case VCU_GYRO_ID:
            break;
        
        case BMS_ID:

            break;
    }
}


MotorData store_VCU_MSG(uint8_t VCU_Msg[8]){
    MotorData Motor;
    Motor.Supply_Voltage    = (VCU_Msg[1]<< 8) | VCU_Msg[9];
    Motor.Temp_M1           = VCU_Msg[2]-100;
    Motor.Temp_M2           = VCU_Msg[3]-100;
    Motor.Current_M1        = VCU_Msg[4];
    Motor.Current_M2        = VCU_Msg[5];
    Motor.RPM_M1            = (VCU_Msg[7]<< 8) | VCU_Msg[6];

    return Motor;
}



