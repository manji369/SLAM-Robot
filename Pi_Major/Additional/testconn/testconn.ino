void setup() {
  Serial.begin(9600);
  pinMode(13,OUTPUT);
}

void loop() {
  int incomingByte;
      digitalWrite(13,HIGH);
      delay(100);y
      digitalWrite(13,LOW);
      delay(100);
      
  if(Serial.available() > 0) {
    // read the incoming byte:
    incomingByte = Serial.read();
    // echo
    Serial.println(incomingByte); 
if(incomingByte == 65){
      digitalWrite(13,HIGH);
      delay(2000);
      digitalWrite(13,LOW);
      delay(2000);
}
  }
}
