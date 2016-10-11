int led = 13;
int rec;

void setup() {
Serial.begin(9600);
pinMode(led, OUTPUT);
}

void loop() {
 while(Serial.available() > 0) {
  rec = Serial.read(); 
 
 if(rec == 0){
  digitalWrite(led, HIGH); 
 }
 if(rec == 1){
  digitalWrite(led, LOW); 
 }
}
}
