void setup() {
  Serial.begin(9600);
  pinMode(13,OUTPUT);
}

void loop() {
  int incomingByte;
  
  if(Serial.available() > 0) {
    // read the incoming byte:
    incomingByte = Serial.read();
      digitalWrite(13,HIGH);
      delay(2000);
      digitalWrite(13,LOW);
    // echo
    Serial.println(incomingByte); 
if(incomingByte == 65){
      digitalWrite(13,HIGH);
      delay(2000);
      digitalWrite(13,LOW);
}
  }
}
