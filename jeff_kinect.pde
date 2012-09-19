 import oscP5.*;
import netP5.*;
import org.openkinect.*;
import org.openkinect.processing.*;
import ddf.minim.*;
import peasy.*;
import java.util.Vector;
import toxi.geom.*;

Minim minim;
AudioInput player;

//Size of winow
final int[] SIZE = {
//  1680, 1050
   2560, 1440
};


// Kinect Library object
Kinect kinect;

//Camera variables
float rotationY = 1.0;
float scal = 1.0;
PVector rot = new PVector();
PVector tran0 = new PVector(width/2, height/2, 530);
PVector tran = new PVector(width/2, height/2, 530);
float x0 = 0;
float y0 = 0;
PeasyCam cam;
Vec3D globalOffset, avg, cameraCenter;
float camD0;
float camDMax;
float camDMin;
float audioFactor = 1.0;

//OSC input var
PVector touch;
PVector pos;

//Depth thresholds
float leftThresh = 0;
float rightThresh = 0;
float frontThresh = 0;
float backThresh = 9;
boolean applyLeftThresh = false;
boolean applyRightThresh = false;
float MAX_THRESH = 255;
boolean useAccl = false;
boolean isColor = true;
int bThresh,rThresh,gThresh;


// Size of kinect image
int w = 640;
int h = 480;

// Kinect on a circle vars
int radius0 = 1200;
float theta0 = 1.2 * PI ;
float thetaDelta = 7 * PI;
float yMult = 6;

//Touch OSC
OscP5 oscP5;
NetAddress sendLoc;

// We'll use a lookup table so that we don't have to repeat the math over and over
float[] depthLookUp = new float[2048];

void setup() {
  size(SIZE[0], SIZE[1], P3D);
  minim = new Minim(this);
  player = minim.getLineIn(Minim.STEREO, 512);
  bThresh = rThresh = gThresh = 125;
  kinect = new Kinect(this);
  kinect.start();
  kinect.enableDepth(true);
  // We don't need the grayscale image in this example
  // so this makes it more efficient
  kinect.processDepthImage(false);
  /* start oscP5, listening for incoming messages at port 8001 */
  oscP5 = new OscP5(this, 8001);
  sendLoc = new NetAddress("192.168.42.105", 9000);

  // Lookup table for all possible depth values (0 - 2047)
  for (int i = 0; i < depthLookUp.length; i++) {
    depthLookUp[i] = rawDepthToMeters(i);
  }
  //noStroke();
  // println(gl);
  camD0 = 3000;
  camDMax = 20000;
  camDMin = 1000;
  cam = new PeasyCam(this, camD0);
  cam.setDistance(camD0);
  cam.setMinimumDistance(camDMin);
  cam.setMaximumDistance(camDMax);
  cameraCenter = new Vec3D(tran.x,tran.y,tran.z);
  avg = new Vec3D();
  globalOffset = new Vec3D(0, 0, 0);//1.f / 5, 2.f / 3);
  touch = new PVector();
  pos = new PVector();
  initOscControls();
}

void initOscControls(){
  OscMessage msg = new OscMessage("/2/rgb/3");
  msg.add(rThresh);
  oscP5.send(msg, sendLoc);
  msg = new OscMessage("/2/rgb/2");
  msg.add(gThresh);
  oscP5.send(msg, sendLoc);
  msg = new OscMessage("/2/rgb/1");
  msg.add(bThresh);
  oscP5.send(msg, sendLoc);
  msg = new OscMessage("/1/camera");
  msg.add(0);
  msg.add(0);
  oscP5.send(msg, sendLoc);
  msg = new OscMessage("/1/position");
  msg.add(0);
  msg.add(0);
  oscP5.send(msg, sendLoc);
  msg = new OscMessage("/1/size");
  msg.add(scal);
  oscP5.send(msg, sendLoc);
  msg = new OscMessage("/1/frontThresh");
  msg.add(frontThresh);
  oscP5.send(msg, sendLoc);
  msg = new OscMessage("/1/backThresh");
  msg.add(backThresh);
  oscP5.send(msg, sendLoc);
  msg = new OscMessage("/1/size");
  msg.add(scal);
  oscP5.send(msg, sendLoc);
  msg = new OscMessage("/2/doRGB");
  msg.add(isColor);
  oscP5.send(msg, sendLoc);
  msg = new OscMessage("/2/doAccl");
  msg.add(useAccl);
  oscP5.send(msg, sendLoc);
  msg = new OscMessage("/3/radius");
  msg.add(radius0);
  oscP5.send(msg, sendLoc);
  msg = new OscMessage("/3/theta");
  msg.add(theta0/PI);
  oscP5.send(msg, sendLoc);
  msg = new OscMessage("/3/thetaDelta");
  msg.add(thetaDelta/PI);
  oscP5.send(msg, sendLoc);
  msg = new OscMessage("/3/yMult");
  msg.add(yMult);
  oscP5.send(msg, sendLoc);
}

PVector maxAcc = new PVector();
PVector minAcc = new PVector();
void oscEvent(OscMessage theOscMessage) {

  String addr = theOscMessage.addrPattern();
  
  if (addr.equals("/1/frontThresh")){
    float  val  = theOscMessage.get(0).floatValue();
    frontThresh = val;//map(val, 0, 1, -600, 600);
  }
  if (addr.equals("/1/backThresh")){
    float  val  = theOscMessage.get(0).floatValue();
    backThresh = val;//map(val, 0, 1, -600, 600);
  }
  if (addr.equals("/2/audioFactor")) {
    float  val  = theOscMessage.get(0).floatValue();
    audioFactor = val;
    println("audioFactor "+audioFactor);
  }
  if(addr.equals("/1/position")) {
    float y = -1*theOscMessage.get(0).floatValue();
    float x = theOscMessage.get(1).floatValue();
    tran.x = x + x0;//map(x,0,1,-1000,000);
    tran.y = y + y0;//map(y,0,1,-3000,1000);
    println(x+", "+y);
  }
  if(addr.equals("/1/position/z")){
    OscMessage msg = new OscMessage("/1/position");
    msg.add(0);
    msg.add(0);
    x0 = tran.x;
    y0 = tran.y;
    oscP5.send(msg, sendLoc);
  }
  if(addr.equals("/1/camera")) {
    float y = theOscMessage.get(0).floatValue();
    float x = theOscMessage.get(1).floatValue();
    rot.z = map(x,-1,1,-PI/50,PI/50);
    rot.x = map(y,-1,1,-PI/50,PI/50);
    println(x+", "+y);
  }
  if(addr.equals("/1/camera/z")){
    OscMessage msg = new OscMessage("/1/camera");
    msg.add(0);
    msg.add(0);
    oscP5.send(msg, sendLoc);
  }
  if (addr.equals("/1/reset")) {
    float  val  = theOscMessage.get(0).floatValue();
    println("reset");
    if (val == 1) {
      println("reset!!!!");
      rot.x = 0;
      rot.y =0;
      rot.z =0;
      scal = 1;
      y0 = 0;
      x0 = 0;
      tran = tran0;
      audioFactor = 1;
      frontThresh = 0;
      backThresh = 9;
      initOscControls();
    }
  }
  if(addr.equals("/1/size")){
    scal = (theOscMessage.get(0).floatValue()/1000);
  }
  if (addr.equals("/2/doAccl")) {
    useAccl = !useAccl;
  }
  if(addr.equals("/2/doRGB"))
    isColor = !isColor;
  if(addr.equals("/2/rgb/1"))
    bThresh = (int)theOscMessage.get(0).floatValue();
  if(addr.equals("/2/rgb/2"))
    gThresh = (int)theOscMessage.get(0).floatValue();
  if(addr.equals("/2/rgb/3"))
    rThresh = (int)theOscMessage.get(0).floatValue();
    
  if(addr.equals("/3/radius")){
    radius0 = (int) theOscMessage.get(0).floatValue();
  }
  if(addr.equals("/3/theta")){
    float val = theOscMessage.get(0).floatValue();
    theta0 = val*PI;
  }
  if(addr.equals("/3/thetaDelta")){
    float val = theOscMessage.get(0).floatValue();
    thetaDelta = val*PI;
  }
  if(addr.equals("/3/yMult")){
    float val = theOscMessage.get(0).floatValue();
    yMult = val;
  }
  if(addr.equals("/accxyz")){
    if (useAccl) {
      float x = theOscMessage.get(0).floatValue();
      float y = theOscMessage.get(1).floatValue();
      float z = theOscMessage.get(2).floatValue();
      if(x > maxAcc.x){
        maxAcc.x = x;
        println("max xacc: "+maxAcc.x);
      }
      if(x < minAcc.x){
        minAcc.x = x;
        println("min xacc: "+minAcc.x);
      }
      if(y > maxAcc.y){
        maxAcc.y = y;
        println("max y acc: "+maxAcc.y);
      }
      if(y < minAcc.y){
        minAcc.y = y;
        println("min y acc: "+minAcc.y);
      }
      //    x = map(x, -1.5, 1.5, 
      if ( abs(y) > .3)
        rot.y += map(y, -2.1, 2.1, -PI/100, PI/100);
      if ( abs(x) > .3)
        rot.x += map(x, -2.1, 2.1, -PI/100, PI/100);
//      if ( abs(z) > .02)
//        rot.x += z;
//      println(rot);
    }
  }
  else {
    println(addr);
  }
}
float maxLC = 0;
void draw() {
  background(0);
  lights();
  fill(255);
  //textMode(SCREEN);
  //text("Kinect FR: " + (int)kinect.getDepthFPS() + "\nProcessing FR: " + (int)frameRate,10,16);

  // Get the raw depth as array of integers
  int[] depth = kinect.getRawDepth();

  // We're just going to calculate and draw every 4th pixel (equivalent of 160x120)
  int skip = 4;

  // Translate and rotate
  cam.rotateX(rot.x);
  rot.x = 0;
  cam.rotateY(rot.y);
  rot.y = 0;
  cam.rotateY(rot.z);
  rot.z = 0;
  rot = new PVector();
//  cam.setDistance(scal);
  translate(tran.x,tran.y,tran.z);
  int buf = 0;
  scale(scal);
//  stroke(255,0,0);
//  strokeWeight(5);
//  line(-2000,0, 2000,0);
//  stroke(0,255,0);
//  strokeWeight(5);
//  line(0,-2000, 0,2000);
//  stroke(0,0,255);
//  strokeWeight(5);
//  line(0,0,-2000, 0,0,2000);
  strokeWeight(1);
  float theta = theta0;
  for (int x=0; x<w; x+=skip) {
    for (int y=0; y<h; y+=skip) {
      int offset = x+y*w;

      // Convert kinect data to world xyz coordinate
      int rawDepth = depth[offset];
      PVector v = depthToWorld(x, y, rawDepth);
      float factor = 400;
      
      if (buf < player.bufferSize() - 1 ) {
        buf++;
      }
      else {
        buf = 0;
      }
      float rightChannel = player.right.get(buf);// * audioFactor;
      float leftChannel = player.left.get(buf);// * audioFactor;
      float mapRC, mapLC;
      mapLC = mapRC = 0;  
      mapRC = map(rightChannel, 0, audioFactor/2, 0,255);
      mapLC = map(leftChannel, 0, audioFactor/2, 0, 255);
//      if(v.z > maxLC){
//        maxLC = v.z;
//        println("new max: "+maxLC);
//      }
      float r , g, b, z;
      // Do the right channel
      int upperColor = (int)map(backThresh, 0, 8, 0, 255);
      int lowerColor = (int)map(frontThresh, 0, 8, 0, 255);
      r = (int) map(v.z, 2, 5, upperColor , lowerColor);//rThresh - mapRC;
      g = gThresh - mapRC;
      b = bThresh - (int) map(v.z, 2, 5, lowerColor, upperColor);
      rawDepth *= 2;
      z = rawDepth;// * rightChannel * audioFactor;  
      shapeMode(CENTER);
      beginShape(POINT);
      strokeWeight(3);
      if(isColor){
        stroke(r, g, b);
      }
      else
        stroke(r);
      if(v.z < backThresh && v.z > frontThresh)// || z < -1*zthresh)
        vertex(v.x*factor, v.y*factor, 3000 - z);
      endShape(CLOSE);
      noFill();
      // Do the left channel
      r = rThresh - mapRC - mapLC;
      g = gThresh + mapLC;
      b = bThresh - mapLC;
      z = rawDepth + (rawDepth * (leftChannel * audioFactor));
      int[] circ  = new int[3];
      int radius = radius0;
      radius += z;
      circ[0] = (int) (radius * cos(theta));
      circ[1] = (int) (y * yMult - v.y) - 2000;
      circ[2] = (int) (radius * sin(theta));
      shapeMode(CENTER);
      beginShape(POINT);
      strokeWeight(10);
      if(isColor)
        stroke(r, g, b);
      else
        stroke(r);
//        if(z > zthresh || z < -1*zthresh)
        vertex(circ[0], circ[1], circ[2]);
//        vertex(v.x*factor, v.y*factor, -200 + -1*z);
      endShape(CLOSE);
      theta += (float) (thetaDelta/(w*h));;
      //popMatrix();
    }
  }
}

// These functions come from: http://graphics.stanford.edu/~mdfisher/Kinect.html
float rawDepthToMeters(int depthValue) {
  if (depthValue < 2047) {
    return (float)(1.0 / ((double)(depthValue) * -0.0030711016 + 3.3309495161));
  }
  return 0.0f;
}

PVector depthToWorld(int x, int y, int depthValue) {

  final double fx_d = 1.0 / 5.9421434211923247e+02;
  final double fy_d = 1.0 / 5.9104053696870778e+02;
  final double cx_d = 3.3930780975300314e+02;
  final double cy_d = 2.4273913761751615e+02;

  PVector result = new PVector();
  double depth =  depthLookUp[depthValue];//rawDepthToMeters(depthValue);
  result.x = (float)((x - cx_d) * depth * fx_d);
  result.y = (float)((y - cy_d) * depth * fy_d);
  result.z = (float)(depth);
  return result;
}

void keyPressed() {
  if (keyCode == LEFT) {
    rot.y -= PI/3;
    println("rotationy: "+rotationY);
  }
  else if (keyCode == RIGHT) {
    rot.y += .5;
    println("rotationy: "+rotationY);
  }
  if (keyCode == CONTROL) {
    rot.x = mouseX;//(x - mouseX)+ x;
    rot.y = mouseY;//(y - mouseY)+ y;
  }
  else if ( key == 'p' ) {
    print("playing...\n");
    //player.play();
  }
  else if (keyCode == SHIFT) {
    rot.z =  mouseY;//(z - mouseY)+ z;
  }
  else if (keyCode == ALT) {
    tran.x = mouseX;
    tran.y = mouseY;
    println("translate at: "+tran.x+", "+tran.y);
  }
  else if (key == 'a') {
    tran.x -= 10;
    println("tran.x :: "+tran.x);
  }
  else if (key == 'd') {
    tran.x += 10;
    println("tran.x :: "+tran.x);
  }
  else if (key == 'w') {
    tran.y += 10;
    println("tran.y :: "+tran.y);
  }
  else if (key == 's') {
    tran.y -= 10;
    println("tran.y :: "+tran.y);
  }
  else if (key == '=') {
    tran.z += 10;
    println("tran.z :: "+tran.z);
  }
  else if (key == '-') {
    tran.z -= 10;
    println("tran.z :: "+tran.z);
  }
  else if (key == 'b') {
    scal*= 1.25;
    println("scale :: "+scal);
  }
  else if (key == 'l') {
    scal*= .65;
    println("scale :: "+scal);
  }
}

void stop() {
  kinect.quit();
  // always close Minim audio classes when you are done with them
  player.close();
  // always stop Minim before exiting
  minim.stop();
  super.stop();
}

