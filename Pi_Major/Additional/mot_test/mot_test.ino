int M1_Pspd = 0;
int M1_Nspd = 255;
int M2_Pspd = 0;
int M2_Nspd = 255;
int M1_P = 5;
int M1_N = 6;
int M2_P = 10;
int M2_N = 11;
int led = 13;
void setup() {
  // put your setup code here, to run once:
  pinMode(M1_P,OUTPUT);
  pinMode(M1_N,OUTPUT);
  pinMode(M2_P,OUTPUT);
  pinMode(M2_N,OUTPUT);
  pinMode(led,OUTPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
  analogWrite(M1_P,M1_Pspd);
  analogWrite(M1_N,M1_Nspd);
  analogWrite(M2_P,M2_Pspd);
  analogWrite(M2_N,M2_Nspd);
  digitalWrite(led,HIGH);
  delay(200);
  digitalWrite(led,LOW);
  delay(200);
}
