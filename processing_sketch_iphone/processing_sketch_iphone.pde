import oscP5.*;        //  Load OSC P5 library
import netP5.*;        //  Load net P5 library
import processing.serial.*;
import java.io.*;
import procontroll.*;
PFont fontA;


/////////////////////////////////////OSC INFO CONTROLLERS////////////////////////////////////
OscP5 oscP5;            //  Set oscP5 as OSC connection
float lfader=0;
float rfader=0;
float xPad;
float yPad;

////////////////////////////////////accelerometer variables////////////////////////////////
float xrot = 0;
float zrot = 0;
float xrot_targ = 0;
float zrot_targ = 0;
float orientation = 0;  //iphone should be upright (this number < 0)
float dampSpeed = 5;  //used for smoothing

////////////////////////////////////////SERIAL DATA (USED TO TALK TO ARBOTIX)////////////////////
Serial myPort;
int[] serialInArray = new int[17];    // Where we'll put what we receive
int serialCount = 0; 
int checksum = 0;
int Serial = 0;    // make 0 to test Processing only (turns off serial connections)... default is one


///////////////////////////////////SENSOR READINGS///////////////////////////////////////////////////////
int IRfront = 0;
int IRleft = 0;
int IRright = 0;
int encoder1byte1 = 0;
int encoder1byte2 = 0;
int encoder1byte3 = 0;
int encoder1byte4 = 0;
int sign_encoder1;
long encoder1 = 0;
int encoder2byte1 = 0;
int encoder2byte2 = 0;
int encoder2byte3 = 0;
int encoder2byte4 = 0;
int sign_encoder2;
long encoder2 = 0;
int stall_left = 0;
int stall_right = 0;

////////////////////////////////////GYRO INFO////////////////////////////////////////////////////////////
float [] q = new float [4];
float [] hq = null;
float [] Euler = new float [3]; // psi, theta, phi
final int VIEW_SIZE_X = 512, VIEW_SIZE_Y = 512;
int display3D = 1;      //set to one to load a 3D cube which illustrates orientation of the gyro sensor

//////////////////////////////////////////SERVO CONTROL VARIABLES//////////////////////////////////////////
int pan = 512;
int pan_high = 0;
int pan_low = 0;
int tilt = 465;
int tilt_max = 550; 
int tilt_min = 380;
int tilt_high = 0;
int tilt_low = 0;

////////////////////////////////////////////MOTOR CONTROL VARIABLES//////////////////////////////////////////////
int leftmotor_mapped = 128;
int leftmotor = 0;
int rightmotor = 0;
int deadband = 15;
int rightmotor_mapped = 128;
float turn_scale_factor = 1.5;
int stallreset = 0;
int gun = 0;
int kill = 0;

//////////////////////////////////////////SETUP FUNCTION//////////////////////////////////////////////////////////
void setup()
{
  if(display3D == 1) {size(512, 512, P3D); } //setup the screen
  else {  size(512, 512); } //setup the screen
  //if(Serial == 1) {println(Serial.list());} //list Com ports for all available serial devices
  if(Serial == 1) {myPort = new Serial(this, "COM36", 38400);}  //connect to Arbotix  //com29 for FTDI, com 36 for Xbee
  oscP5 = new OscP5(this,8000);  // Start oscP5, listening for incoming messages at port 8000  
   
  background(0);  //see here for color codes: http://www.rapidtables.com/web/color/RGB_Color.htm
  stroke(255);  //sets color to make lines
  fontA = loadFont("ArialMT-14.vlw");  //font is in a folder called "data" in the same location as the processing sketch
  textFont(fontA, 16);
  fill(255); //sets color to fill in shapes, this includes text color
}

/////////////////////////////////////////////DRAW FUNCTION/////////////////////////////////////////////////////////////
void draw()
{
  if(Serial == 1) {readSensors();}
  cursor(CROSS);
  background(0);  // erases the display each loop

  ////////////////////////////////////////////////////////KILL SWITCH////////////////////////////////////////
  if(kill==0)  //EMERGENCY STOP BUTTON
  {
    fill(255,0,0);
    rect(400, 400, 100, 100);
    fill(255);
    text("Emergency", 410,440);
    text("STOP", 430, 460);
  }
  else  //RESET BUTTON
  {
    fill(0,255,0);
    rect(400, 400, 100, 100);
    fill(0);
    text("Press to", 425,440);
    text("RESET", 430, 460);
  }
  //////////////////////////////////////////////////DISPLAY GUN INFORMATION//////////////////////////////////
  if(gun == 1)  
   {
    fill(255,0,0);  //red
    rect(400, 15, 65, 30);
    fill(255); //white
    text("Gun On", 405, 35);
   }
  else if(gun == 0)
  {
    fill(0,128,0);  //green
    rect(400, 45, 65, 30);
    fill(255);  //white
    text("Gun Off", 405, 65);
  }
  
  /////////////////////////////////////////////DISPLAY TURRET INFORMATION///////////////////////////////////////
  fill(255);  //white
  line(0, 256, 512,256);   //x axis
  line(256, 0, 256, 512);  //y axis
  fill(0,0,0);
  stroke(255,0,0);   // makes a bulls-eye at turret position
  ellipseMode(CENTER);  
  ellipse(map(pan, 200, 823, 512, 0), map(tilt, tilt_min, tilt_max, 0, 512), 25, 25);
  ellipse(map(pan, 200, 823, 512, 0), map(tilt, tilt_min, tilt_max, 0, 512), 15, 15);
  line(map(pan, 200, 823, 512, 0), map(tilt, tilt_min, tilt_max, 0, 512)+20, map(pan, 200, 823, 512, 0), map(tilt, tilt_min, tilt_max, 0, 512)-20);
  line(map(pan, 200, 823, 512, 0)+20, map(tilt, tilt_min, tilt_max, 0, 512), map(pan, 200, 823, 512, 0)-20, map(tilt, tilt_min, tilt_max, 0, 512));
  stroke(255);  //display pan and tilt numbers 
  fill(255);
  pan_low = (pan&0xFF);
  pan_high = ((pan>>8)&0x03);
  tilt_low = (tilt&0xFF);
  tilt_high = ((tilt>>8)&0x03);
  text("Pan:", 10, 410);
  text(pan, 10, 430);
  text("Tilt:", 10, 470);
  text(tilt, 10, 490);
  
  ///////////////////////////////PROCESS ACCELEROMETER DATA///////////////////////////////////////////////////
  accelerometer_smoothing();
  map_accelerometer_to_motorvalues(deadband, 100);
    
  /////////////////////////////////DISPLAY MOTOR INFORMATION///////////////////////////////////////////////////
  text("Left:", 10, 20);
  text(leftmotor, 10, 40);
  text("Right:", 100, 20);
  text(rightmotor, 100, 40);
  if(stall_left == 1) 
  { 
    fill(255,0,0);  //red
    rect(10, 60, 62, 30);
    fill(255); //white
    text("Stalled", 16, 83);
  }
  if(stall_right == 1) 
  { 
    fill(255,0,0);  //red
    rect(100, 60, 62, 30);
    fill(255); //white
    text("Stalled", 106, 83);
  }

  ///////////////////////////////////////DISPLAY GYRO INFORMATION///////////////////////////////////////////////
  if (hq != null) { // use home quaternion
    quaternionToEuler(quatProd(hq, q), Euler);
    //text("Disable home position by pressing \"n\"", 20, VIEW_SIZE_Y - 30);
  }
  else {
    quaternionToEuler(q, Euler);
    //text("Point FreeIMU's X axis to your monitor then press \"h\"", 20, VIEW_SIZE_Y - 30);
  }
  text("Psi:", 10, 240);
  text(Euler[0], 50, 240);
  text("Theta:", 10, 270);
  text(Euler[1], 50, 270);
  text("Phi:", 10, 300);
  text(Euler[2], 50, 300);
  if(display3D == 1) {drawCube(); } 
  
  //////////////////////////////////////DISPLAY SENSOR INFORMATION//////////////////////////////////////////////
  text("Distance (cm):", 130, 410);
  text(IRfront, 190, 430);
  text("Front", 130, 430);
  text(IRleft, 190, 450);
  text("Left", 130, 450);
  text(IRright, 190, 470);
  text("Right", 130, 470);
  
  encoder2 = ((encoder2byte1) + (encoder2byte2<<8) + (encoder2byte3<<16) + (encoder2byte4<<24));
  if(sign_encoder2 == 255) { encoder2 = -encoder2;}
  text("Right Encoder:", 275, 470);
  text(encoder2, 275, 490);
  
  encoder1 = ((encoder1byte1) + (encoder1byte2<<8) + (encoder1byte3<<16) + (encoder1byte4<<24));
  if(sign_encoder1 == 255) { encoder1 = -encoder1;}
  text("Left Encoder:", 275, 410);
  text(encoder1, 275, 430);

  ///////////////////////////////////////////////SEND DATA OVER SERIAL PORT/////////////////////////////////////////////
  checksum = (pan_low + pan_high + tilt_low + tilt_high + gun + leftmotor_mapped + rightmotor_mapped + stallreset)%256;
  if(Serial == 1) {sendPacket(pan_low, pan_high, tilt_low, tilt_high, gun, leftmotor_mapped, rightmotor_mapped, stallreset, checksum);}
 
  
}

void sendPacket(int _panbyte1, int _panbyte2, int _tiltbyte1, int _tiltbyte2, int _gun, int _leftmotor, int _rightmotor, int _stallreset, int _checksum) 
{
    myPort.write(char(0xFF));   //all char values need to be between 0 and 255
    myPort.write(char(_panbyte1));
    myPort.write(char(_panbyte2));     
    myPort.write(char(_tiltbyte1)); 
    myPort.write(char(_tiltbyte2));    
    myPort.write(char(_gun));
    myPort.write(char(_leftmotor));   
    myPort.write(char(_rightmotor));  
    myPort.write(char(_stallreset));  
    myPort.write(char(_checksum));    
  
}

void accelerometer_smoothing()
{
    if (xrot_targ > xrot) 
      xrot = xrot + ((xrot_targ - xrot) / dampSpeed);
       
    else 
      xrot = xrot - ((xrot - xrot_targ) / dampSpeed);
    
    if (zrot_targ > zrot) 
      zrot = zrot + ((zrot_targ - zrot) / dampSpeed);
     
    else 
      zrot = zrot - ((zrot - zrot_targ) / dampSpeed);
}

void map_accelerometer_to_motorvalues(float _deadband, float speed_limit)
{
    float adj_val1;  
    float adj_val2; 

    float deadband_high = _deadband; // sets deadband_high to be half of deadband (ie. 10/2 = 5)
    float deadband_low = _deadband * -1; // sets deadband_low to be negative half of deadband (ie. 5 * -1 = -5)

    float x; // this will represent the x coordinate
    float y; // this will represent the y coordinate

    adj_val1 = map(zrot, -90, 90, -speed_limit, speed_limit); 
    adj_val1 = constrain(adj_val1, -speed_limit, speed_limit);

    x = adj_val1;

    adj_val2 = map(xrot, -90, 90, -speed_limit, speed_limit); 
    adj_val2 = constrain(adj_val2, -speed_limit, speed_limit);

    y = adj_val2;
    
    if (y > deadband_high) 
    {  // if the Up/Down accelerometer input is above the upper threshold, go FORWARD 

      // now check to see if left/right input from accelerometer is to the left, to the right, or centered.

      if (x > deadband_high) 
      { // go forward while turning right proportional to the accelerometer left/right input
        leftmotor = int(y);
        rightmotor = int(y - (x*turn_scale_factor));
        // quadrant 1
      }

      else if (x < deadband_low) 
      {   // go forward while turning left proportional to the accelerometer left/right input
        leftmotor = int(y - ( x * -1 * turn_scale_factor));  // remember that in this case, y will be a negative number
        rightmotor = int(y);
        // quadrant 2
      }
      else 
      {   // left/right accelerometer is centered, go straight forward
        leftmotor = int(y);
        rightmotor = int(y);
      }
    }
    else if (y < deadband_low) 
    {  // if the Up/Down input is below the lower threshold, go BACKWARD 

      // now check to see if left/right input is to the left, to the right, or centered.

      if (x > deadband_high) 
      { // go forward while turning right proportional to the accelerometer left/right input
        leftmotor = int(y - ( x * -1 * turn_scale_factor)); 
        rightmotor = int(y); 
        // quadrant 3
      }

      else if (x < deadband_low) 
      {   // go forward while turning left proportional to the accelerometer left/right input
        leftmotor = int(y);  // remember that in this case, y will be a negative number
        rightmotor = int(y - (x * turn_scale_factor));
        // quadrant 4
      }
      else 
      {   // left/right accelerometer is centered, go straight forward
        leftmotor = int(y);
        rightmotor = int(y);
      }
    }
    else 
    {     // if neither of the above 2 conditions is met, the Up/Down accelerometer is centered (neutral)

      if (abs(x) > deadband_high)   //spin in place
      {
        leftmotor = int(x) ;
        rightmotor = int(-x) ;
      }  
      else //in center cube
      {
        leftmotor = 0;
        rightmotor = 0;
      }
    }
    constrain(leftmotor, -100, 100);
    constrain(rightmotor, -100, 100);
    leftmotor_mapped = int(map(leftmotor, -100, 100, 30, 225));
    rightmotor_mapped = int(map(rightmotor, -100, 100, 30, 225));
}
  


void oscEvent(OscMessage theOscMessage) 
{   //  This runs whenever there is a new OSC message
   String addr = theOscMessage.addrPattern();  //  Creates a string out of the OSC message
   if(addr.indexOf("/1/toggle1") !=-1)            //E-STOP
   {   // Filters out any toggle buttons
     int i = int((addr.charAt(9) )) - 0x30;   // returns the ASCII number so convert into a real number by subtracting 0x30
     kill  = int(theOscMessage.get(0).floatValue());     //  Puts button value into kill
     if(kill == 1)
     {
       gun = 0;
       leftmotor = 0;
       rightmotor = 0;
       rightmotor_mapped = 128;
       leftmotor_mapped = 128;
       pan = 511;
       tilt = int(map(512, 0, 1023, tilt_min, tilt_max));
      }
   }
   if(kill != 1)    //prevents anything from changing if kill switch is hit
   {
    if(addr.indexOf("/1/toggle2") !=-1)            //GUN POWER
    {   // Filters out any toggle buttons
      int i = int((addr.charAt(9) )) - 0x30;   // returns the ASCII number so convert into a real number by subtracting 0x30
      gun  = int(theOscMessage.get(0).floatValue());     //  Puts button value into gun
    }
     
    if(addr.indexOf("/1/xy") !=-1)    // the TURRET XY area
    { 
        xPad =  (theOscMessage.get(0).floatValue());
        yPad =  (theOscMessage.get(1).floatValue());
        pan = int(map(yPad, 0, 1023, 823, 200));
        tilt = int(map(xPad, 0, 1023, tilt_max, tilt_min));
    }
    if(theOscMessage.checkAddrPattern("/accxyz")==true) //accelerometer data
    {
        xrot_targ = (theOscMessage.get(0).floatValue()*90);  
        zrot_targ = (theOscMessage.get(1).floatValue()*90)*-1;
        orientation = theOscMessage.get(2).floatValue();
    }      
  }
}


void mousePressed() 
{
  if(mouseX > 400 && mouseX < 500 && mouseY > 400 && mouseY < 500)  //Kill Switch Pressed
  { 
    leftmotor = 0;
    rightmotor = 0;
    rightmotor_mapped = 128;
    leftmotor_mapped = 128;
    gun = 0;
    pan = 511;
    tilt = int(map(512, 0, 1023, tilt_min, tilt_max));
    kill = 1-kill;  
  }
}

void readSensors() {
  if (myPort.available() >= 43) {
    String inputString = myPort.readStringUntil('\n');
    //print(inputString);
    if (inputString != null && inputString.length() > 0) {
      String [] inputStringArr = split(inputString, ",");
      if (inputStringArr.length >= 20) { // q1,q2,q3,q4,\r\n so we have 5 elements
        IRfront = int(inputStringArr[0]);
        IRleft = int(inputStringArr[1]);
        IRright = int(inputStringArr[2]);
        encoder1byte1 = int(inputStringArr[3]);
        encoder1byte2 = int(inputStringArr[4]);
        encoder1byte3 = int(inputStringArr[5]);
        encoder1byte4 = int(inputStringArr[6]);
        sign_encoder1 = int(inputStringArr[7]);
        encoder2byte1 = int(inputStringArr[8]);
        encoder2byte2 = int(inputStringArr[9]);
        encoder2byte3 = int(inputStringArr[10]);
        encoder2byte4 = int(inputStringArr[11]);
        sign_encoder2 = int(inputStringArr[12]);
        stall_left = int(inputStringArr[13]);
        stall_right = int(inputStringArr[14]);
        q[0] = decodeFloat(inputStringArr[15]);
        q[1] = decodeFloat(inputStringArr[16]);
        q[2] = decodeFloat(inputStringArr[17]);
        q[3] = decodeFloat(inputStringArr[18]);
      }
    }
  }
}


float decodeFloat(String inString) {
  byte [] inData = new byte[4];

  if (inString.length() == 8) {
    inData[0] = (byte) unhex(inString.substring(0, 2));
    inData[1] = (byte) unhex(inString.substring(2, 4));
    inData[2] = (byte) unhex(inString.substring(4, 6));
    inData[3] = (byte) unhex(inString.substring(6, 8));
  }

  int intbits = (inData[3] << 24) | ((inData[2] & 0xff) << 16) | ((inData[1] & 0xff) << 8) | (inData[0] & 0xff);
  return Float.intBitsToFloat(intbits);
}

// See Sebastian O.H. Madwick report 
// "An efficient orientation filter for inertial and intertial/magnetic sensor arrays" Chapter 2 Quaternion representation

void quaternionToEuler(float [] q, float [] euler) {
  euler[0] = atan2(2 * q[1] * q[2] - 2 * q[0] * q[3], 2 * q[0]*q[0] + 2 * q[1] * q[1] - 1); // psi
  euler[1] = -asin(2 * q[1] * q[3] + 2 * q[0] * q[2]); // theta
  euler[2] = atan2(2 * q[2] * q[3] - 2 * q[0] * q[1], 2 * q[0] * q[0] + 2 * q[3] * q[3] - 1); // phi
}

float [] quatProd(float [] a, float [] b) {
  float [] q = new float[4];

  q[0] = a[0] * b[0] - a[1] * b[1] - a[2] * b[2] - a[3] * b[3];
  q[1] = a[0] * b[1] + a[1] * b[0] + a[2] * b[3] - a[3] * b[2];
  q[2] = a[0] * b[2] - a[1] * b[3] + a[2] * b[0] + a[3] * b[1];
  q[3] = a[0] * b[3] + a[1] * b[2] - a[2] * b[1] + a[3] * b[0];

  return q;
}

// returns a quaternion from an axis angle representation
float [] quatAxisAngle(float [] axis, float angle) {
  float [] q = new float[4];

  float halfAngle = angle / 2.0;
  float sinHalfAngle = sin(halfAngle);
  q[0] = cos(halfAngle);
  q[1] = -axis[0] * sinHalfAngle;
  q[2] = -axis[1] * sinHalfAngle;
  q[3] = -axis[2] * sinHalfAngle;

  return q;
}

// return the quaternion conjugate of quat
float [] quatConjugate(float [] quat) {
  float [] conj = new float[4];

  conj[0] = quat[0];
  conj[1] = -quat[1];
  conj[2] = -quat[2];
  conj[3] = -quat[3];

  return conj;
}

void keyPressed() {
  if (key == 'h') {
    println("pressed h");

    // set hq the home quaternion as the quatnion conjugate coming from the sensor fusion
    hq = quatConjugate(q);
  }
  else if (key == 'n') {
    println("pressed n");
    hq = null;
  }
}

void drawCube() {  
  pushMatrix();
  translate(VIEW_SIZE_X/5, VIEW_SIZE_Y/2, 0);
  scale(1, 1, 1);

  // a demonstration of the following is at 
  // http://www.varesano.net/blog/fabio/ahrs-sensor-fusion-orientation-filter-3d-graphical-rotating-cube
  rotateZ(-Euler[2]);
  rotateX(-Euler[1]);
  rotateY(-Euler[0]);

  buildBoxShape();

  popMatrix();
}

void buildBoxShape() {
  //box(60, 10, 40);
  noStroke();
  beginShape(QUADS);

  //Z+ (to the drawing area)
  fill(#00ff00);
  vertex(-30, -5, 20);
  vertex(30, -5, 20);
  vertex(30, 5, 20);
  vertex(-30, 5, 20);

  //Z-
  fill(#0000ff);
  vertex(-30, -5, -20);
  vertex(30, -5, -20);
  vertex(30, 5, -20);
  vertex(-30, 5, -20);

  //X-
  fill(#ff0000);
  vertex(-30, -5, -20);
  vertex(-30, -5, 20);
  vertex(-30, 5, 20);
  vertex(-30, 5, -20);

  //X+
  fill(#ffff00);
  vertex(30, -5, -20);
  vertex(30, -5, 20);
  vertex(30, 5, 20);
  vertex(30, 5, -20);

  //Y-
  fill(#ff00ff);
  vertex(-30, -5, -20);
  vertex(30, -5, -20);
  vertex(30, -5, 20);
  vertex(-30, -5, 20);

  //Y+
  fill(#00ffff);
  vertex(-30, 5, -20);
  vertex(30, 5, -20);
  vertex(30, 5, 20);
  vertex(-30, 5, 20);

  endShape();
}

