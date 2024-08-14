int ReceiverPins[] = {25, 26, 27, 32, 33, 34};

void setup(){
    for (int i = 0; i < 6; i++){
        pinMode(ReceiverPins[i], input);
    }
}

void loop(){
    int Roll_raw = constrain(pulseIn(12, HIGH), 1044, 1885);
    int Pitch_raw = constrain(pulseIn(13, HIGH), 1135, 1800);
    int Yaw_raw = constrain(pulseIn(11, HIGH, 25000), 1013, 1941);

    int Roll = map(Roll_raw, 1044, 1885, -254, 254);
    int Pitch = map(Pitch_raw, 1135, 1800, 254, -254);
    int Yaw = map(Yaw_raw, 1013, 1941, -254, 254);

    if (abs(Roll) < 24){
        Roll = 0;
    }
    if (abs(Pitch) < 24){
        Pitch = 0;
    }
    if (abs(Yaw) < 30){
        Yaw = 0;
    }

    Serial.print(Roll_raw);
    Serial.print(", ");
    Serial.print(Pitch_raw);
    Serial.print(", ");
    Serial.print(Yaw_raw);
    Serial.println(", ");

    Serial.print(Roll);
    Serial.print(", ");
    Serial.print(Pitch);
    Serial.print(", ");
    Serial.print(Yaw);
    Serial.println(", ");
}