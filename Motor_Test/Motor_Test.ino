/* =========================================================================
   Take number from Seiral with serialEvent() and set it as throtle point
 * ========================================================================= */

#include <ESP32Servo.h>

int i = 0;
int Motor_Pins[4] = {16, 17, 18, 19};

bool stringComplete;
String inputString;

Servo MotorFL;  // create servo object to control a ESC
Servo MotorFR;
Servo MotorBL;
Servo MotorBR;

void setup()
{
    Serial.begin(115200);
    MotorFL.attach(Motor_Pins[0]);  // create servo object to control a ESC
    MotorFR.attach(Motor_Pins[1]);
    MotorBL.attach(Motor_Pins[2]);
    MotorBR.attach(Motor_Pins[3]);
}

void loop()
{
    serialEvent();
    if (stringComplete)
    {
        Serial.print("input: ");
        Serial.println(inputString);
        if (inputString.startsWith("Step"))
        {
            Serial.println("Ack: Step");
            delay(100);
            int spaceIndex = inputString.indexOf(' ');
            if (spaceIndex != -1)
            {
                String value = inputString.substring(spaceIndex + 1);
                int amount = value.toInt();
                Serial.println(amount);
                MotorFL.write(amount);
                MotorFR.write(amount);
                MotorBL.write(amount);
                MotorBR.write(amount);
            }
        }

        inputString = "";
        stringComplete = false;
    }
}

void serialEvent()
{
    while (Serial.available())
    {
        char inChar = (char)Serial.read();
        inputString += inChar;
        if (inChar == '\n')
        {
            stringComplete = true;
        }
    }
}

// void MotorPID(float setpoint){
//     // calculate Error
//     for(int i=0; i<3; i++){
//         e[i] = RPY_Setpoint[i] - rpy[i];
//     }

//     // calculate PID ctl cmd
//     for(int i=0; i<3; i++){
//         integral[i] += e[i] * dt;
//         g[i] = P[i]*e[i] + I[i]*(integral[i]) + D[i]*(e[i]-Prev_e[i])/dt;
//     }

//     // convert PID ctl cmd to motor ctl cmd
//     for(int i=0; i<3; i++){
//         Motor_Speed[i] = map(constrain(({-1, 1, -1, 1} * g(1) + {1, 1, -1, -1} * g(2) + {1, -1, -1, 1} * g(3)), -255, 255), -255, 255, 0, 180);
//     }

//     MotorFL.write(Motor_Speed[0]);
//     MotorFR.write(Motor_Speed[1]);
//     MotorBL.write(Motor_Speed[2]);
//     MotorBR.write(Motor_Speed[3]);
// }