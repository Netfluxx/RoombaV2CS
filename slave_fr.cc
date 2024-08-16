//front right arduino slave code
//implements PID control and encoder reading and data transmission to master via I2C
#include <Wire.h>

const int I2C_ADDR = 0x10;

const byte CMD_FORWARD = 0x01;
const byte CMD_BACKWARD = 0x02;
const byte CMD_RIGHT = 0x03;
const byte CMD_LEFT = 0x04;
const byte CMD_STOP = 0x05;
const byte CMD_ERR = 0xff;
byte prev_command = 0xff;

const int IN1 = 9;
const int IN2 = 10;

// Encoder hardware interrupt pins
const int ENCPIN1 = 2;
const int ENCPIN2 = 3;

const float DISK_SLOTS = 40;  // TODO:update to real value
const float WHEEL_RADIUS = 0.05;  //TODO: update to real value
const float PI = 3.14159265359;

volatile int enc1_count = 0;  // Volatile because it is modified in an interrupt, not unsigned because it can be negative (reverse direction)
volatile int enc2_count = 0;

unsigned long prev_time = 0;
unsigned long current_time = 0;

volatile float received_cmd = 0.0;
volatile float distance_travelled = 0.0;
volatile float speed = 0.0;

// volatile char* data = "speed0.00dist000.00";
// volatile boolean receive_flag = false;
// volatile char data_buffer[19]

//PID gains
const float KP = 0.5;
const float KI = 0.5;
const float KD = 0.5;
volatile float prev_error = 0.0;

void setup() {
    // Initialize the motor pins
    pinMode(IN1, OUTPUT);
    pinMode(IN2, OUTPUT);
    
    // Initialize the encoder hardware interrupt pins
    pinMode(ENCPIN1, INPUT);
    pinMode(ENCPIN2, INPUT);
    attachInterrupt(digitalPinToInterrupt(ENCPIN1), isr_enc1, CHANGE);
    attachInterrupt(digitalPinToInterrupt(ENCPIN2), isr_enc2, CHANGE);

    Serial.begin(9600);
    Wire.begin(I2C_ADDR);
    Wire.onReceive(receiveEvent);
    Wire.onRequest(requestEvent);
    
    prev_time = millis();
}

void loop(){
    current_time = millis();
    if(current_time - prev_time >= 75){  // Calculate speed every 75 milliseconds (random ngl)
        int total_count = enc1_count + enc2_count;
        
        //getting wheel direction: encoders are in quadrature 
        //so we can determine direction by checking the relative phase of the two encoder signals
        int previous_phase = 0;

        static bool wheel_direction = true; //true for forward, false for backward
        //ENCPIN1 is shifted by 1 bit to the left (becomes MSB), ENCPIN2 is not shifted (LSB)
        //ENCPIN1=0, ENCPIN2=0 -> phase=00=0
        //ENCPIN1=0, ENCPIN2=1 -> phase=01=1
        //ENCPIN1=1, ENCPIN2=0 -> phase=10=2
        //ENCPIN1=1, ENCPIN2=1 -> phase=11=3
        int current_phase = (digitalRead(ENCPIN1) << 1) | digitalRead(ENCPIN2);

        //if the phase changes from one phase to another--> forwards
        //if the phase changes from one phase to another--> backwards
        //WARNING: ASSUMES CORRECT POSITION OF BOTH ENCODERS ON DISK, 
        //HIGH CHANCE OF 'OFF BY ONE' ERROR FOR THE DIRECTION
        //TODO: check if this is the correct way to determine the direction

        // if ((previous_phase == 0 && current_phase == 1) ||
        //     (previous_phase == 1 && current_phase == 3) ||
        //     (previous_phase == 3 && current_phase == 2) ||
        //     (previous_phase == 2 && current_phase == 0)) {
        //     wheel_direction = true;  // Forwards
        // } else if ((previous_phase == 0 && current_phase == 2) ||
        //         (previous_phase == 2 && current_phase == 3) ||
        //         (previous_phase == 3 && current_phase == 1) ||
        //         (previous_phase == 1 && current_phase == 0)) {
        //     wheel_direction = false;  // Backwards
        // }

        previous_phase = current_phase;

        distance_travelled += (total_count * 2 * PI * WHEEL_RADIUS) / (DISK_SLOTS * 4);
        speed = (total_count * 2 * PI * WHEEL_RADIUS) / (DISK_SLOTS * 4) / ((current_time - prev_time) / 1000.0);

        enc1_count = 0;
        enc2_count = 0;

        // Round to two decimal places
        speed = round(speed * 100) / 100.0;
        distance_travelled = round(distance_travelled * 100) / 100.0;

        prev_time = current_time;
    }
}

void receiveEvent(int howMany){ //polled by master arduino on a timer
    while (Wire.available()){
        //float = 4 bytes
        byte float_bytes[4];
        for (int i = 0; i < 4; i++) {
            float_bytes[i] = Wire.read();
        }
        received_cmd = *((float*)float_bytes);
        executeCommand(received_cmd); 
    }
}

void requestEvent(){
    //send current speed and distance travelled to master
    //-->send 2*4 bytes
    byte* speed_ptr = (byte*)&speed;
    byte* distance_ptr = (byte*)&distance_travelled;
    for (int i = 0; i < 4; i++) {
        Wire.write(speed_ptr[i]);
    }
    for (int i = 0; i < 4; i++) {
        Wire.write(distance_ptr[i]);
    }
}

void executeCommand(float command){//command = target speed in m/s
    // Implement the PID control here
    // Command is the signed float speed (m/s) value given by RPi through master arduino

    float current_speed = speed;
    float error = target_speed - current_speed;
    static float integral = 0.0;  // static so it retains its value between function calls
    float derivative = error - prev_error;
    prev_error = error;
    integral += error;
    int new_command = sign(error) * round(abs(KP * error) + abs(KI * integral) + abs(KD * derivative));    
    //TODO: check if this is the correct way to calculate the new command
    //TODO: how can I take into account the direction of the wheel in the PID control?
    
    new_command = constrain(new_command, -255, 255);

    if (new_command > 0){
        analogWrite(IN1, new_command);
        analogWrite(IN2, 0);
    } else {
        analogWrite(IN1, 0);
        analogWrite(IN2, -new_command);
    }
    
}

void isr_enc1(){
    if (digitalRead(ENCPIN1) != digitalRead(ENCPIN2)){
        enc1_count++;
    } else {
        enc1_count--;
    }
}


void isr_enc2(){
    if (digitalRead(ENCPIN2) != digitalRead(ENCPIN1)){
        enc2_count++;
    } else {
        enc2_count--;
    }
}

int sign(float value) {
    return (value > 0) - (value < 0);
}
