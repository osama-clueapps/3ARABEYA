import java.util.Locale;
import java.lang.Math;

import android.content.Intent;
import android.os.Bundle;
import android.speech.tts.TextToSpeech;
import android.content.Context;
import android.content.Context;
import android.hardware.Sensor;
import android.hardware.SensorManager;
import android.hardware.SensorEvent;
import android.hardware.SensorEventListener;

import ketai.net.bluetooth.*;
import ketai.ui.*;
import ketai.sensors.*;
import ketai.camera.*;
import ketai.cv.facedetector.*;

import ketai.net.KetaiOSCMessage;

import oscP5.*;

Context context;
KetaiCamera cam;
KetaiSimpleFace[] faces;
TextToSpeech t1;

SensorManager manager;
Sensor sensor;
RotationListener listener;

KetaiBluetooth bt;
KetaiList klist;

float[] mMatrixR = new float[9];
float[] mMatrixValues = new float[3];
float rotx, roty, rotz;
boolean findFaces = false;
boolean obstacle = false;
int cooldown;
int facecount, human;
String info = "";

ArrayList<String> devicesDiscovered = new ArrayList();
String deviceName = "AT+NAME";

//********************************************************************
// The following code is required to enable bluetooth at startup.
//********************************************************************
void onCreate(Bundle savedInstanceState) {
  super.onCreate(savedInstanceState);
  bt = new KetaiBluetooth(this);
}

void onActivityResult(int requestCode, int resultCode, Intent data) {
  bt.onActivityResult(requestCode, resultCode, data);
}


void setup() {
  bt.start();
  println(bt.getPairedDeviceNames());

  cam = new KetaiCamera(this, 480, 360, 30);
  context = getActivity();
  t1 = new TextToSpeech(context, new TextToSpeech.OnInitListener() {
         @Override
         public void onInit(int status) {
            if(status != TextToSpeech.ERROR) {
               t1.setLanguage(Locale.US);
            }
         }
  });
  
  manager = (SensorManager)context.getSystemService(Context.SENSOR_SERVICE);
  sensor = manager.getDefaultSensor(Sensor.TYPE_ROTATION_VECTOR);
  listener = new RotationListener();
  manager.registerListener(listener, sensor, SensorManager.SENSOR_DELAY_GAME);

  
  orientation(LANDSCAPE);
  println(cam.list());
  cam.setCameraID(0); //front camera
  imageMode(CENTER);
  stroke(255);
  noFill();
  textSize(48);
  smooth();
}

void draw() {
  background(128);
  
    //scale(0.3);
    //translate(width/2, height/2);  // 3
    //rotate( -radians(rotx));  // 4
    //stroke(255);
    //triangle(-width/4, 0, width/4, 0, 0, -width/2);  // 5

  
  if (cam != null){
    if(!cam.isStarted()){
      pushStyle();
      textAlign(CENTER, CENTER);
      String info ="Camera Info\n";
      info += "current camera: " + cam.getCameraID() + "\n";
      info += "image dimensions: " + cam.width + "x" + cam.height + "\n";
      info += "photo dimensions: " + cam.getPhotoWidth() + "x" + cam.getPhotoHeight() + "\n";
      info += "flash state: " + cam.isFlashEnabled() + "\n";
      text(info, width/2, height/2);
      popStyle();  
    } else {
      if (findFaces){
        facecount = 0;
        image(cam, width/2, height/2, width, height); 
        faces = KetaiFaceDetector.findFaces(cam, 5);
        if (faces.length > 0)
          facecount++;
        delay(50);
        image(cam, width/2, height/2, width, height); 
        faces = KetaiFaceDetector.findFaces(cam, 5); 
        if (faces.length > 0)
          facecount++;
          
        if (facecount > 1 && cooldown <= 0) {
          human = 1;
          if (obstacle) {
            greet();
            cooldown = 2;
          }
        }
      }
    } 
    drawUI();
  }
  if (frameCount % 30 == 0) {
      if (!isConnected()) {
         bt.connectToDeviceByName(deviceName);
      }
      checkEnableCamera();
      if (cooldown > 0) {
        cooldown--;
      } else {
        human = 0;
      }
  }
}

void drawUI(){
  fill(0, 128);
  
  rect(0, 0, width/4, 100);
  rect(width/4, 0, width/4, 100);
  rect((width/4)*2, 0, width/4, 100);
  rect((width/4)*3, 0, width/4-1, 100);

  fill(255);
  if (cam.isStarted()){
    text("Cam on", 20, 70); 
  } else {
    text("Cam off", 20, 70); 
  }
  
  if (findFaces && obstacle){
    text ("Faces found: " + faces.length, (width/4)*3+20, 70);
  } else {
    
  }
  
  translate(width/2, height/2);
  if (isConnected()) {
    text("Connected\n", 0, 0, width/2, height/2);
  } else {
      text("Disconnected\n" , 0, 0, width/2, height/2);
  }

  text("Processed Rot: \n" +
      "rotx: " + nfp(rotx, 1, 3) + "\n" + 
      "sent to uC" + nfp(int(rotx) & 255,1,3)+" "+nfp((int(rotx) & 65280)>> 8,1,3)+"\n"
      , 0, 0, width, height);

}

boolean checkEnableCamera() {
  if (!cam.isStarted()){
    if(!cam.start()){
      println("Failed to start the camera.");
    }
    else {
      findFaces = true;
    }
  }
  return findFaces;
}


void onBluetoothDataEvent(String who, byte[] data)
{
  println("message rec" + who + '\n' + data[0]);
  obstacle = boolean(data[0]);
  println("obstacle: " + obstacle);
}

void greet() {
  //Toggle Camera on/off
  String toSpeak = new String("Hello");
  t1.speak(toSpeak, TextToSpeech.QUEUE_FLUSH, null);
}

void onCameraPreviewEvent(){
  cam.read();
}


class RotationListener implements SensorEventListener {
  public void onSensorChanged(SensorEvent event) {
    switch (event.sensor.getType()) {
        case Sensor.TYPE_ROTATION_VECTOR:
            SensorManager.getRotationMatrixFromVector(mMatrixR, event.values);

            SensorManager.getOrientation(mMatrixR, mMatrixValues);

            rotx = degrees(mMatrixValues[0])+180;
            roty = degrees(mMatrixValues[1])+180;
            rotz = degrees(mMatrixValues[2])+90;

    }
    
    if (isConnected())
      sendToUC();
    
  }
  public void onAccuracyChanged(Sensor sensor, int accuracy) {
  }
}

boolean isConnected() {
    for (int i = 0; i < bt.getConnectedDeviceNames().size(); i++) {
      if (bt.getConnectedDeviceNames().get(i).equals(deviceName)) {
        //println("Connected!");
        return true;  
    }
  }
  return false;
}

void sendToUC() { // problem
    byte[] a = {byte(255)};
    bt.broadcast(a);
    byte[] b = {byte(int(rotx) & 255)};
    bt.broadcast(b);
    byte[] c = {byte( (int(rotx) & 65280) >> 8)};
    bt.broadcast(c);
    byte[] d = {byte(human)};
    bt.broadcast(d);
    byte[] e = {byte(255)};
    bt.broadcast(e);
}
void initBluetooth(boolean granted) {
  if (granted) 
    println("Bluetooth Access Granted");
}
