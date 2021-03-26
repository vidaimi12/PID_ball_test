/*  This code is part of the PID ball test project.
    Created by: Imre Vida © 2021 All rights reserved.

    More information (original, Hungarian): https://vidaimi.com/project/pid_ball
    More information (English): https://vidaimi.com/en/project/pid_ball */

#include <Wire.h>
#include <VL53L0X.h>
#include <Servo.h>

#define LED 3
#define Potentiometer A0
#define ServoPin 9

VL53L0X sensor;
Servo myservo;

//PID constants
const double Kp = 0.3949;
const double Ki = 0.0082;
const double Kd = 10.8231;

//variables
int pos = 93;                       //servo position
int targetlast = 180, target = 180; //for noise reduction
int last = 150, current = 0;        //for the differential part of the controller
double integrateError = 0;          //for the integral part of the controller
int a = 0, b = 0;                   //for measuring dead time

void setup()
{
    //setup pins and basic functions
    Serial.begin(9600);
    Wire.begin();
    myservo.attach(ServoPin);
    pinMode(LED, OUTPUT);
    pinMode(Potentiometer, INPUT);
    sensor.setTimeout(500);

    //check for errors and malfunctions
    if (!sensor.init())
    {
        Serial.println("Failed to detect and initialize sensor!");
        while (1) //infinite loop, sensor not detected
        {
            delay(1);
        }
    }

    //setup sensor parameters
    sensor.setSignalRateLimit(0.25);
    sensor.setVcselPulsePeriod(VL53L0X::VcselPeriodPreRange, 14);
    sensor.setVcselPulsePeriod(VL53L0X::VcselPeriodFinalRange, 10);
    sensor.setMeasurementTimingBudget(22000); //max time of measurement 22ms

    //set starting points
    digitalWrite(LED, 1);
    myservo.write(pos);
    b = millis();
    a = b;
}

void loop()
{
    current = (last + sensor.readRangeSingleMillimeters()) / 2;                                 //read the current distance of the ball, with noise reduction
    target = (targetlast + targetlast + ((analogRead(Potentiometer) / 1023.0) * 240 + 60)) / 3; //read the target position from the potentiometer, with noise reduction

    integrateError += target - current;                                          //do the integration
    pos = Kp * (target - current) + Ki * integrateError + Kd * (last - current); //calculate the control signal

    pos = 93 + pos;                                  //set the "zero point" of the servo. In my case at 93 degrees servo position the beam is paralell with the ground
    pos = (pos >= 0 ? (pos <= 180 ? pos : 180) : 0); //software limit (the servo can't go further)
    myservo.write(pos);                              //act, turn the servo

    //check for errors
    if (sensor.timeoutOccurred())
    {
        Serial.print("SENSOR TIMEOUT");
    }

    //visual feedback of the right position max abs(10mm) error
    if (abs(target - current) < 10)
    {
        digitalWrite(LED, 1); //indicate right position
    }
    else
    {
        digitalWrite(LED, 0); //indicate bad position
    }

    //communication
    Serial.print(current);
    Serial.print(' ');
    Serial.print(target);
    Serial.print(' ');

    b = millis();        //measure current time
    Serial.print(b - a); //print the approx dead time of the controller
    Serial.println();

    //update previous values (the "memory" of the code)
    targetlast = target;
    last = current;
    a = b;
}
