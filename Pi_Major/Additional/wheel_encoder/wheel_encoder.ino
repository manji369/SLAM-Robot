int x = 0;
void setup() {
  // initialize serial communications at 9600 bps:
  Serial.begin(9600); 
  attachInterrupt(0, print, CHANGE);
}
void loop() {                     

}
void print() {
  x = x + 1;
  //Serial.println(x/256);
  Serial.println(x);
if(x == 40)
{
 x = 0; 
}
}
