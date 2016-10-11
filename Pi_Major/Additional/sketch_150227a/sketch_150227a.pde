import processing.serial.*;
 
Serial myPort; // The serial port:
int val = 0;
 int buff;
 float ang1;
 float x;
 float y;
void setup() {
  size(400,400);
 background(0);
  println(Serial.list());
 
  // You may need to change the number in [ ] to match 
  // the correct port for your system
  myPort = new Serial(this, Serial.list()[0], 9600);

}
 
void draw() {
  
 
 
 
  while (myPort.available () > 0) {
    buff = myPort.read();
    if(buff == 10){
     val = val+1; 
    }
    if (val == 40)
   {
     val = 0;
      background(0);
   } 
    //println(val);
  }
 
 ang1 = radians(val*9);
 x = 200 + 100*cos(ang1);
 y = 200 + 100*sin(ang1);
 ellipse(x,y,10,10);
}
