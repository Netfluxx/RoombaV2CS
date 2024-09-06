//Master Arduino Code ready for PID control, receives 4 floats for the speed of each wheel

#include <Wire.h>

//slave addresses
const byte SLAVE_FR = 0x10;
const byte SLAVE_FL = 0x20;
const byte SLAVE_BR = 0x30;
const byte SLAVE_BL = 0x40;

//front right = 0x10, front left = 0x20, back right = 0x30, back left = 0x40

const float ref_voltage = 4.774; //measure me
const float resistor_ratio = (11.96 + 197)/11.96;//17.47157


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
  delay(500);//200ms

}

void loop() {

      int sensor_value = analogRead(A2);//measuring the battery voltage
      float voltage = sensor_value * (ref_voltage / 1023.0);
      float batt_voltage = voltage * resistor_ratio;
      Serial.print("battery voltage: "+ String(batt_voltage)+ "\n");

  //if (Serial.available() > 0){ //uncomment this when connected to the RPi
      //String received_str = Serial.readStringUntil('\n');
      //string format sent from RPi: "FR_speed,FL_speed,BR_speed,BL_speed"
      //speeds are floats in m/s
      float speeds[4] = {1.5, 1.5, 1.5, 1.5}; //for debug without RPi
      //int i = 0;
      //char* received_cstr = (char*)received_str.c_str();
      //char* token = strtok(received_cstr, ","); //split the string by comma
      //while (token != NULL) {
      //    speeds[i] = atof(token);
      //    token = strtok(NULL, ",");
      //    i++;
      //}

      //Serial.println("RPi -> Master SPEEDS: "+ String(speeds[0]) + ", " + String(speeds[1]) + ", " + String(speeds[2]) + ", " + String(speeds[3]));
      
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
          Wire.write((byte*)&speeds[slave_index-1], sizeof(float));//sending appropriate speed to the corresponding slave
          Wire.endTransmission();

          //request the speed from the slave
          Wire.requestFrom((uint8_t)slave_addr, (uint8_t)8); //4 bytes (1 float = 4 bytes)
          float speed_rcv = 0.0;
          byte* speed_ptr = (byte*)&speed_rcv;
          
          for (int i = 0; i < 4; i++) {
              speed_ptr[i] = Wire.read();
          }

          slave__received_speeds[slave_index-1] = speed_rcv;
          msg += slave_name + ": " + String(speed_rcv) + " , ";
      }
      
      Serial.println(msg + "\n");

  //}
}
