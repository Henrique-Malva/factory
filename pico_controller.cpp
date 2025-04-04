#include <Arduino.h>
#include "MotorController.h"
#include <Preferences.h>
#include <LSM6.h>  // Acelerômetro e Giroscópio

LSM6 imu;

#define mag_pin 22

void parseData();      // split the data into its parts

double motor_speed[4]={0,0,0,0};
double encoder_speed[4]={0,0,0,0};
uint8_t magnetic_toggle = 0, magnetic_toggle_prev = 0;


MotorController motor1(2, 3, 6, 7);
MotorController motor2(4, 5, 8, 9);
MotorController motor3(14, 15, 10, 11);
MotorController motor4(18, 19, 12, 13);



void setup() {
    Serial.begin(57600);
    while (!Serial.available());   // Wait for serial monitor
    pinMode(mag_pin, OUTPUT);
    imu.enableDefault();

    //motor1.begin(kp1, ki1, kd1);
    //motor2.begin(kp2, ki2, kd2);
    //motor3.begin(kp3, ki3, kd3);
    //motor4.begin(kp4, ki4, kd4);

    delay(2000);

}

char command[20];
char temp_com[20];
char out[100];

void loop() {
    delay(100);
    char c=-1,i=0;
    while ((c=Serial.read()) != 0) {
        command[i]=c;
        i++;
    }
    command[i]=0;
    strcpy(temp_com, command);
    parseData();

    motor1.speeed(motor_speed[0]);
    motor2.speeed(motor_speed[1]);
    motor3.speeed(motor_speed[2]);
    motor4.speeed(motor_speed[3]);

    if(magnetic_toggle_prev!=magnetic_toggle) {digitalWrite(mag_pin, magnetic_toggle); }

    encoder_speed[0]=motor1.measure();
    encoder_speed[1]=motor2.measure();
    encoder_speed[2]=motor3.measure();
    encoder_speed[3]=motor4.measure();

    imu.read();

    // Leitura da aceleração
    float accX = imu.a.x * 0.000061 * 9.80665;
    float accY = imu.a.y * 0.000061 * 9.80665;
    float accZ = imu.a.z * 0.000061 * 9.80665;

    // Leitura do giroscópio
    float gyroX = imu.g.x * PI / 180.0;
    float gyroY = imu.g.y * PI / 180.0;
    float gyroZ = imu.g.z * PI / 180.0;

    sprintf(out, "%f,%f,%f,%f,%f,%f,%f,%f,%f,%f", encoder_speed[0], encoder_speed[1], encoder_speed[2], encoder_speed[3], accX, accY, accZ, gyroX, gyroY, gyroZ);
    Serial.print(out);

}

void parseData() {      // split the data into its parts

    char * strtokIndx; // this is used by strtok() as an index

    strtokIndx = strtok(temp_com,",");      // get the first part - the string
    motor_speed[0] = atof(strtokIndx)*9.549; // convert this part to a float

    strtokIndx = strtok(NULL, ","); // this continues where the previous call left off
    motor_speed[1] = atof(strtokIndx)*9.549; // convert this part to a float

    strtokIndx = strtok(NULL, ",");
    motor_speed[2] = atof(strtokIndx)*9.549; // convert this part to a float

    strtokIndx = strtok(NULL, ",");
    motor_speed[3] = atof(strtokIndx)*9.549; // convert this part to a float

    strtokIndx = strtok(NULL, ",");
    magnetic_toggle_prev = magnetic_toggle;
    magnetic_toggle = atoi(strtokIndx); // convert this part to a float
}