int M1_P = 5;
int M1_N = 6;
int M2_P = 10;
int M2_N = 11;
int led = 13;
int spd = 255;
int Trig1 = 7;
int echo1 = 8;
int Trig2 = 12;
int echo2 = 4;
int del = 20;
unsigned long echo = 0;
unsigned long dist = 0;
unsigned long distance = 0;
void setup() {
  Serial.begin(9600);
  pinMode(M1_P,OUTPUT);
  pinMode(M1_N,OUTPUT);
  pinMode(M2_P,OUTPUT);
  pinMode(M2_N,OUTPUT);
  pinMode(Trig1,OUTPUT);
  pinMode(echo1,INPUT);
  pinMode(Trig2,OUTPUT);
  pinMode(echo2,INPUT);
  pinMode(led,OUTPUT);
}

unsigned long ping(int x, int y){
  digitalWrite(led,HIGH);
  digitalWrite(x,LOW);
  delayMicroseconds(5);
  digitalWrite(x,HIGH);
  delayMicroseconds(10);
  digitalWrite(x,LOW);
  echo = pulseIn(y,HIGH);
  dist = echo/58.138;
  Serial.println(dist);
  digitalWrite(led,LOW);
  return dist;
}

void mov_direction(int M1_P_spd,int M1_N_spd,int M2_P_spd,int M2_N_spd){
  analogWrite(M1_P,M1_P_spd);
  analogWrite(M1_N,M1_N_spd);
  analogWrite(M2_P,M2_P_spd);
  analogWrite(M2_N,M2_N_spd);
  digitalWrite(led,HIGH);
  Serial.println("Forward");
  delay(del);
  analogWrite(M1_P,0);
  analogWrite(M1_N,0);
  analogWrite(M2_P,0);
  analogWrite(M2_N,0);
  digitalWrite(led,LOW);
}
void loop() {
  for(int kl = 1; kl<3; kl++){
  digitalWrite(led,HIGH);
  delay(1000);
  digitalWrite(led,LOW);
  delay(1000);
  }
  while(1){
  int incomingByte;
  
  if(Serial.available() > 0) {
    incomingByte = Serial.read();
    switch(incomingByte){
      case 65:
      mov_direction(spd,0,spd,0);
      break;
      case 66:
      mov_direction(0,spd,0,spd);
      break;
      case 67:
      mov_direction(spd,0,0,spd);
      break;
      case 68:
      mov_direction(0,spd,spd,0);
      break;
      case 69:
      ping(Trig1,echo1);
      break;
      case 70:
      ping(Trig2,echo2);
      break;
  }
}
  }
}
