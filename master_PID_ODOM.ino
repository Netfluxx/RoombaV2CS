//Master Arduino Code ready for PID control, receives 4 floats for the speed of each wheel

#include <Wire.h>

//slave addresses
const byte SLAVE_FR = 0x10;
const byte SLAVE_FL = 0x20;
const byte SLAVE_BR = 0x30;
const byte SLAVE_BL = 0x40;

//front right = 0x10, front left = 0x20, back right = 0x30, back left = 0x40


void setup() {
  Wire.begin();
  Serial.begin(9600);
  Serial.println("Master ready, reminder: front right = 0x10, front left = 0x20, back right = 0x30, back left = 0x40");

  for (int i = 0; i < 4; i++){
    byte slave_addr = 0xff;
    String slave_name = "";
    switch (i) {
        case 0: 
            slave_addr = SLAVE_FR; 
            slave_name = "FRONT RIGHT";
            break;
        case 1: 
            slave_addr = SLAVE_FL;
            slave_name = "FRONT LEFT";
            break;
        case 2: 
            slave_addr = SLAVE_BR;
            slave_name = "BACK RIGHT"; 
            break;
        case 3: 
            slave_addr = SLAVE_BL; 
            slave_name = "BACK LEFT";
            break;
        default: 
            break;
    }

    Wire.beginTransmission(slave_addr);

    if (Wire.endTransmission() == 0){
        Serial.println(slave_name + " SLAVE DETECTED 0x" + String(slave_addr, HEX));
    }else{
        Serial.println(slave_name + " SLAVE NOT DETECTED 0x" + String(slave_addr, HEX));
    }
  }
  delay(200);//200ms

}

void loop() {
    if (Serial.available() > 0){
        String received_str = Serial.readStringUntil('\n');
        //string format sent from RPi: "FR_speed,FL_speed,BR_speed,BL_speed"
        //speeds are floats in m/s
        float speeds[4];
        int i = 0;
        char* received_cstr = (char*)received_str.c_str();
        char* token = strtok(received_cstr, ","); //split the string by comma
        while (token != NULL) {
            speeds[i] = atof(token);
            token = strtok(NULL, ",");
            i++;
        }

        //Serial.println("Master SPEEDS:"+ String(speed[0]) + "," + String(speed[1]) + "," + String(speed[2]) + "," + String(speed[3]));
        
        float slave__received_speeds[4];
        String msg = "";

        for(int slave_index=1; slave_index<5; slave_index++){
            byte slave_addr = 0xff;
            String slave_name = "";
            switch(slave_index){
                case 1:
                    slave_addr = SLAVE_FR;
                    slave_name = "FRONT RIGHT";
                    break;
                case 2:
                    slave_addr = SLAVE_FL;
                    slave_name = "FRONT LEFT";
                    break;
                case 3:
                    slave_addr = SLAVE_BR;
                    slave_name = "BACK RIGHT";
                    break;
                case 4:
                    slave_addr = SLAVE_BL;
                    slave_name = "BACK LEFT";
                    break;
                default:
                    break;
            }
            Wire.beginTransmission(slave_addr);
            Wire.write((byte*)&speeds[slave_index-1], sizeof(float));
            Wire.endTransmission();

            //request the speed from the slave
            Wire.requestFrom(slave_addr, 8);//4 bytes (1 float = 4 bytes)
            float speed_rcv = 0.0;
            byte* speed_ptr = (byte*)&speed_rcv;
            
            for (int i = 0; i < 4; i++) {
                speed_ptr[i] = Wire.read();
            }

            slave__received_speeds[slave_index-1] = speed_rcv;
            msg += slave_name + ": " + String(speed_rcv) + " , ";
        }
        
        Serial.println(msg + "\n");

    }
}
