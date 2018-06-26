

#define VALVE_1 2


void setup() {
  // put your setup code here, to run once:
    Serial.begin(9600);                 // Start the serial terminal
    Serial.println("Testing Active Payload System");
    Serial.println();
    pinMode(VALVE_1,OUTPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
  Serial.println("Opening Valve 1");
  digitalWrite(VALVE_1, HIGH);
  delay(2000);
  Serial.println("Closing Valve 1");
  digitalWrite(VALVE_1, LOW);
  delay(2000);
}
