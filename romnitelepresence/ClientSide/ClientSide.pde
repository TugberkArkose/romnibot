import java.awt.image.*; 
import javax.imageio.*;
import java.net.*;
import java.io.*;

int port = 9100; 
int port2 = 9110;
DatagramSocket ds; 
DatagramSocket ds2;
byte[] buffer = new byte[65536]; 
byte[] buffer_2 = new byte[65536];
PImage video;
String compass;
long time;
int wait = 3000;


void setup() {
  time = millis();
  size(400,300);
  try {
    ds = new DatagramSocket(port);
    ds2 = new DatagramSocket(port2);
  } catch (SocketException e) {
    e.printStackTrace();
  } 
  video = createImage(320,240,RGB);
}

 void draw() {
    checkForImage();
    
    long current_time = millis();
    if(current_time - time >= wait)
   {
    checkForCompass();  
    time = current_time;
   }  
   
    print(compass);
    background(0);
    imageMode(CORNERS);
    image(video,0,0);
  }

void checkForCompass(){
    DatagramPacket p = new DatagramPacket(buffer_2,buffer_2.length);
    try {
      ds2.receive(p);
    } catch (IOException e) {
      e.printStackTrace();
    } 
    compass =new String(p.getData(), 0, p.getLength());
  }

void checkForImage() {
    DatagramPacket p = new DatagramPacket(buffer, buffer.length); 
    try {
      ds.receive(p);
    } catch (IOException e) {
      e.printStackTrace();
    } 
    byte[] data = p.getData();
  
  
    ByteArrayInputStream bais = new ByteArrayInputStream( data );
  
    video.loadPixels();
    try {
      BufferedImage img = ImageIO.read(bais);
      img.getRGB(0, 0, video.width, video.height, video.pixels, 0, video.width);
    } catch (Exception e) {
      e.printStackTrace();
    }
    video.updatePixels();
  }
