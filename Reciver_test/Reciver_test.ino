int ReceiverPins[] = {2, 8, 4, 5};

int avg = 0;
int i = 1;
int sum = 0;

void setup(){
    Serial.begin(115200);
    // for (i = 0; i < 4; i++){
    //     pinMode(ReceiverPins[i], INPUT);
    // }
    digitalWrite(12, HIGH);
}

void loop(){
    int Roll_raw     = pulseIn(ReceiverPins[0], HIGH, 25000);
    int Pitch_raw    = pulseIn(ReceiverPins[1], HIGH, 25000);
    int Yaw_raw      = pulseIn(ReceiverPins[2], HIGH, 25000);
    int Throttle_raw = pulseIn(ReceiverPins[3], HIGH, 25000);

    int Roll        = map(constrain(Roll_raw,     1051, 1885), 1051, 1885, -254,  254);
    int Pitch       = map(constrain(Pitch_raw,    1051, 1885), 1051, 1885,  254, -254);
    int Yaw         = map(constrain(Yaw_raw,      1051, 1885), 1051, 1885, -254,  254);
    int Throttle    = map(constrain(Throttle_raw, 1051, 1885), 1051, 1885, -254,  254);

    if (abs(Roll) < 24){
        Roll = 0;
    }
    // if (abs(Pitch) < 24){
    //     Pitch = 0;
    // }
    if (abs(Yaw) < 24){
        Yaw = 0;
    }
    if (abs(Throttle) < 24){
        Throttle = 0;
    }

    Serial.print("Roll_raw:");
    Serial.print(Roll_raw);
    Serial.print(" , Roll:");
    Serial.print(Roll);

    Serial.print(" , Pitch_raw:");
    Serial.print(Pitch_raw);
    Serial.print(" , Pitch:");
    Serial.print(Pitch);

    Serial.print(" , Yaw_raw:");
    Serial.print(Yaw_raw);
    Serial.print(" , Yaw:");
    Serial.print(Yaw);

    Serial.print(" , Throttle_raw:");
    Serial.print(Throttle_raw);
    Serial.print(" , Throttle:");
    Serial.println(Throttle);
}