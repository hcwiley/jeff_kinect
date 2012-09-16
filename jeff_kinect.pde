 import oscP5.*;
import netP5.*;
//import org.openkinect.*;
//import org.openkinect.processing.*;
import ddf.minim.*;
import peasy.*;
import java.util.Vector;
import toxi.geom.*;

Minim minim;
AudioInput player;

//Size of winow
final int[] SIZE = {
  1680, 1050
};


// Kinect Library object
//Kinect kinect;

//Camera variables
float rotationY = 1.0;
float scal = 1.0;
PVector rot = new PVector();
PVector tran = new PVector(width/2+40, height/2+40, -50);
PeasyCam cam;
Vec3D globalOffset, avg, cameraCenter;
float camD0;
float camDMax;
float camDMin;
float audioFactor = 1;
//int scal;

//OSC input var
PVector touch;
PVector pos;

//Depth thresholds
float leftThresh = 0;
float rightThresh = 0;
float zthresh = 0;
boolean applyLeftThresh = false;
boolean applyRightThresh = false;
float MAX_THRESH = 255;
boolean useAccl = false;
boolean isColor = true;
int bThresh,rThresh,gThresh;


// Size of kinect image
int w = 640;
int h = 480;

//Touch OSC
OscP5 oscP5;
NetAddress sendLoc;

// We'll use a lookup table so that we don't have to repeat the math over and over
float[] depthLookUp = new float[2048];

void setup() {
  size(SIZE[0], SIZE[1], P3D);
  minim = new Minim(this);
  player = minim.getLineIn(Minim.STEREO, 512);
  bThresh = rThresh = gThresh = 0;
//  kinect = new Kinect(this);
//  kinect.start();
//  kinect.enableDepth(true);
  // We don't need the grayscale image in this example
  // so this makes it more efficient
//  kinect.processDepthImage(false);
  /* start oscP5, listening for incoming messages at port 8001 */
  oscP5 = new OscP5(this, 8001);
  sendLoc = new NetAddress("192.168.42.105", 9000);

  // Lookup table for all possible depth values (0 - 2047)
  for (int i = 0; i < depthLookUp.length; i++) {
    depthLookUp[i] = rawDepthToMeters(i);
  }
  //noStroke();
  // println(gl);
  camD0 = 1400;
  camDMax = 10000;
  camDMin = -10000;
  cam = new PeasyCam(this, camD0);
  cam.setDistance(camD0);
  cam.setMinimumDistance(camDMin);
  cam.setMaximumDistance(camDMax);
  cameraCenter = new Vec3D();
  avg = new Vec3D();
  globalOffset = new Vec3D(0, 0, 0);//1.f / 5, 2.f / 3);
  touch = new PVector();
  pos = new PVector();
  OscMessage msg = new OscMessage("/2/rgb/3");
  msg.add(rThresh);
  oscP5.send(msg, sendLoc);
  msg = new OscMessage("/2/rgb/2");
  msg.add(gThresh);
  oscP5.send(msg, sendLoc);
  msg = new OscMessage("/2/rgb/1");
  msg.add(bThresh);
  oscP5.send(msg, sendLoc);
}

void oscEvent(OscMessage theOscMessage) {

  String addr = theOscMessage.addrPattern();
  
  if (addr.equals("/1/thresh")) {
    float  val  = theOscMessage.get(0).floatValue();
    zthresh = val;//map(val, 0, 1, -600, 600);
    println("audioFactor "+audioFactor);
  }
  if(addr.equals("/1/position")) {
    float y = theOscMessage.get(0).floatValue();
    float x = theOscMessage.get(1).floatValue();
    tran.x = x;//map(x,0,1,-1000,000);
    tran.y = y;//map(y,0,1,-3000,1000);
    println(x+", "+y);
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
//      tran.x = SIZE[0]/2 + 40;
//      tran.y = SIZE[1]/2 + 40;
//      tran.z = -50;
    }
  }
  if(addr.equals("/1/size")){
    scal = (theOscMessage.get(0).floatValue()/200);
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
  if(addr.equals("/accxyz")){
    if (useAccl) {
      float x = theOscMessage.get(0).floatValue()*.009;
      float y = theOscMessage.get(1).floatValue()*.009;
      float z = theOscMessage.get(2).floatValue()*.009;
      //    x = map(x, -1.5, 1.5, 
      if ( abs(y) > .003)
        rot.y += y;
      if ( abs(x) > .003)
        rot.x += x;
//      if ( abs(z) > .02)
//        rot.x += z;
      println(rot);
    }
  }
  else {
    println(addr);
  }
}

void draw() {
  background(0);
  lights();
  fill(255);
  //textMode(SCREEN);
  //text("Kinect FR: " + (int)kinect.getDepthFPS() + "\nProcessing FR: " + (int)frameRate,10,16);

  // Get the raw depth as array of integers
//  int[] depth = [0];//kinect.getRawDepth();

  // We're just going to calculate and draw every 4th pixel (equivalent of 160x120)
  int skip = 4;

  // Translate and rotate
  cam.rotateX(rot.x);
  rot.x = 0;
  cam.rotateY(rot.y);
  rot.y = 0;
  cam.rotateY(rot.z);
  rot.z = 0;
  cam.setDistance(tran.z);
  translate(tran.x,tran.y,0);
  int buf = 0;
  scale(scal);
  for (int x=0; x<w; x+=skip) {
    for (int y=0; y<h; y+=skip) {
      int offset = x+y*w;

      // Convert kinect data to world xyz coordinate
      int rawDepth = 0;//depth[offset];
      PVector v = depthToWorld(x, y, rawDepth);

      stroke(255);
      float factor = 200;
      if (buf < player.bufferSize() - 1 ) {
        buf++;
      }
      else {
        buf = 0;
      }
//            fill(0,255,0);
//      sphere(10);
      shapeMode(CENTER);
      beginShape(POINT);
      float rightChannel = player.right.get(buf);// * audioFactor;
      float leftChannel = player.left.get(buf);// * audioFactor;
      rawDepth = rawDepth * 4;
      rightChannel = map(rightChannel, 0, 10, 0,255);
      leftChannel = map(leftChannel, 0, 10, 0, 255);
      float r , g, b, z;
      r = rThresh + leftChannel;//180 + 1.03 + ;
      g = gThresh + rightChannel;// * 1.003;
      b = bThresh +rightChannel;//*rawDepth * .93;
      z = rawDepth * leftChannel;
    
      if(isColor)
        stroke(r, g, b);
      else
        stroke(r);
//        if(z > zthresh || z < -1*zthresh)
        vertex(v.x*factor, v.y*factor, z);
        
      endShape(CLOSE);
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
    rotationY -= .5;
    println("rotationy: "+rotationY);
  }
  else if (keyCode == RIGHT) {
    rotationY += .5;
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
//  kinect.quit();
  // always close Minim audio classes when you are done with them
  player.close();
  // always stop Minim before exiting
  minim.stop();
  super.stop();
}

