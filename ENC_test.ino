// arduino test code for direction detection and speed measurement, should work
// Encoder hardware interrupt pins
const int ENCPIN1 = 2;
const int ENCPIN2 = 3;

const float DISK_SLOTS = 73;  // 73 slots per revolution ???? CHECK THIS
const float WHEEL_RADIUS = 0.056;  // 60mm radius without tyre deformation(not realistic)
const float WHEEL_CIRC = 2*PI*WHEEL_RADIUS;

volatile long prev_enc1_count = 0;
volatile long prev_enc2_count = 0;
long prev_measure_time = 0;
volatile int enc1_count = 0;
volatile int enc2_count = 0;
float speed = 0.0;

String direction = "unknown";
volatile int static_counter = 0;
int direction_counter_test = 0;

//moving avg for direction
const int BUFFER_SIZE = 8;//completely random value but works experimentally
int direction_buffer[BUFFER_SIZE];
int buffer_index = 0;
int buffer_sum = 0;
int direction_avg = 0;

//moving avg for speed
const int SPEED_BUFFER_SIZE = 20;
float speed_buffer[SPEED_BUFFER_SIZE];
int speed_buffer_index = 0;
float speed_sum = 0;
float speed_avg = 0;

void setup() {
    // hardware interrupt pins
    pinMode(ENCPIN1, INPUT);
    pinMode(ENCPIN2, INPUT);
    attachInterrupt(digitalPinToInterrupt(ENCPIN1), isr_enc1, CHANGE);
    attachInterrupt(digitalPinToInterrupt(ENCPIN2), isr_enc2, CHANGE);

    Serial.begin(9600);

    // movnig avg buffer
    for (int i = 0; i < BUFFER_SIZE; i++) {
        direction_buffer[i] = 0;
    }
    for (int i = 0; i < SPEED_BUFFER_SIZE; i++) {
        speed_buffer[i] = 0.0; // Initialize the speed buffer
    }
    prev_measure_time = millis();
}

void loop() {
    long current_time = millis();
    long delta_time = current_time - prev_measure_time;

    if(delta_time>10){ //calculate every 10ms approx but dunno if thats good or not
        int delta_enc1_count = abs(enc1_count - prev_enc1_count);
        int delta_enc2_count = abs(enc2_count - prev_enc2_count);
        float revs1 = float(delta_enc1_count) / DISK_SLOTS;
        float revs2 = float(delta_enc2_count) / DISK_SLOTS;
        float revs = (revs1+revs2)/2.0;
        float distance = revs * WHEEL_CIRC;  //dist
        speed = distance / (delta_time / 1000.0);  //m/s
        //speed moving avg
        speed_sum -= speed_buffer[speed_buffer_index];
        speed_buffer[speed_buffer_index] = speed;
        speed_sum += speed;
        speed_buffer_index = (speed_buffer_index + 1) % SPEED_BUFFER_SIZE;
        speed_avg = speed_sum / SPEED_BUFFER_SIZE;

        float first_half_avg = 0;
        float second_half_avg = 0;
        for(int i = 0; i < BUFFER_SIZE / 2; i++) {
            first_half_avg += direction_buffer[i];
            second_half_avg += direction_buffer[i + BUFFER_SIZE / 2];
        }
        first_half_avg /= (BUFFER_SIZE / 2);
        second_half_avg /= (BUFFER_SIZE / 2);

        float slope = second_half_avg - first_half_avg;
        if (slope > 0) {
            direction = "FORWARD";
            static_counter = 0;
        } else if (slope < 0) {
            direction = "BACKWARDS";
            static_counter = 0;
        } else {
            static_counter++;
        }
        
        //Serial.print(String(speed_avg)+"\n");
        prev_measure_time = current_time;
        prev_enc1_count = enc1_count;
        prev_enc2_count = enc2_count;
    }
    
    // moving average of direction_counter_test
    buffer_sum -= direction_buffer[buffer_index];
    direction_buffer[buffer_index] = direction_counter_test;
    buffer_sum += direction_counter_test;

    buffer_index++;
    if (buffer_index >= BUFFER_SIZE) {
        buffer_index = 0;
    }

    direction_avg = buffer_sum / BUFFER_SIZE;

    if (static_counter > 5) {
        direction = "STATIC";
    }

    Serial.print(direction + "\n");
    Serial.print(String(direction_avg)+"\n");
    
    //reset stuff
    //prev_enc1_count = enc1_count;
    //prev_enc2_count = enc2_count;
    

}

void isr_enc1() {
    if (digitalRead(ENCPIN1) != digitalRead(ENCPIN2)) {
        enc1_count++;
        direction = "FORWARD";
        direction_counter_test++;
        static_counter = 0;
    } else {
        enc1_count--;
    }
}

void isr_enc2() {
    if (digitalRead(ENCPIN2) != digitalRead(ENCPIN1)) {
        enc2_count++;
        direction = "BACKWARD";
        direction_counter_test--;
        static_counter = 0;
    } else {
        enc2_count--;
    }
}
