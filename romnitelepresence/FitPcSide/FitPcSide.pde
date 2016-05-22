//import processing.video.*;
import processing.serial.*;    
import javax.imageio.*;
import java.awt.image.*; 
import java.io.*;
import java.net.*;

Serial myPort;   
boolean UpPressed = false;
boolean DownPressed = false;
boolean RightPressed = false;
boolean LeftPressed = false;
boolean R_Pressed = false;
boolean T_Pressed = false;
boolean F_Pressed = false;
boolean G_Pressed = false;
long time;
long wait = 3000;
long sendDelay = 100;
long lastSentTime;
long newSendTime;
byte[]buf ;

int clientPort = 9100; 
int clientPort_compass = 9110;
DatagramSocket ds; 
DatagramSocket ds2; 
//Capture cam;

void setup() {
  size(320,240);
  lastSentTime = millis();
  
  
  time = millis();
  //size(200, 200);                                  
  String portName = Serial.list()[0];              
  myPort = new Serial(this, portName, 9600);
  try {
    ds = new DatagramSocket();
    ds2=new DatagramSocket();
  } catch (SocketException e) {
    e.printStackTrace();
  }

//  cam = new Capture( this, 320,240,30);
//  cam.start();
}

/*void captureEvent( Capture c ) {
  c.read();
  broadcast(c);
}*/

void draw() {  
  background(255);        
  frame.setAlwaysOnTop(true);   
  newSendTime = millis();
   
  if(newSendTime - lastSentTime > sendDelay){    
  
    if ( UpPressed && !RightPressed && !LeftPressed) 
    {  
      sendMessage("w");
    } else if (RightPressed && UpPressed)
    {
      sendMessage("e");
    } else if (LeftPressed && UpPressed)
    {
      sendMessage("q");
    } else if ( DownPressed  && !RightPressed && !LeftPressed) 
    {
      sendMessage("s");
    } else if (RightPressed && DownPressed)
    {
     sendMessage("x");
    } else if (LeftPressed && DownPressed)
    {
     sendMessage("z");
    } else if (RightPressed)
    {
      sendMessage("d");
    } else if (LeftPressed)
    {
      sendMessage("a");
    } else if (R_Pressed)
    {
      sendMessage("r");
    } else if (T_Pressed)
    {
      sendMessage("t");
    } else if (F_Pressed)
    {
      sendMessage("f");
    } else if (G_Pressed)
    {
      sendMessage("g");
    }else{
      sendMessage("n");  
    }    
  }
    
 long current_time = millis(); 
 
 if(current_time - time >= wait)
 {
   if(myPort.available() > 0)
   {
    String compass = myPort.readString();
    buf = compass.getBytes();
    try
    {
    ds2.send(new DatagramPacket(buf,buf.length, InetAddress.getByName("192.168.142.1"),9110)); // IP buraya gelecek !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
    }
    catch (Exception e) 
    {
    e.printStackTrace();
    }
    myPort.clear();
    time = current_time;
   }
 }
    //delay(500);
}

void sendMessage(String c){
  myPort.write(c);
  textSize(32);
  text(c, 10, 30); 
  fill(0, 102, 153);
  lastSentTime = newSendTime;
}

void broadcast(PImage img) {
  BufferedImage bimg = new BufferedImage( img.width,img.height, BufferedImage.TYPE_INT_RGB );
  img.loadPixels();
  bimg.setRGB( 0, 0, img.width, img.height, img.pixels, 0, img.width);
  ByteArrayOutputStream baStream  = new ByteArrayOutputStream();
  BufferedOutputStream bos    = new BufferedOutputStream(baStream);
  try {
    ImageIO.write(bimg, "jpg", bos);
  } 
  catch (IOException e) {
    e.printStackTrace();
  }


  byte[] packet = baStream.toByteArray();

  try {
    ds.send(new DatagramPacket(packet,packet.length, InetAddress.getByName("192.168.142.1"),clientPort)); // ip buraya gelecek !!!!!!!!!
  } 
  catch (Exception e) {
    e.printStackTrace();
  }
}
void keyPressed()
{
  if (keyCode==UP && !UpPressed)
    UpPressed=true;
  if (keyCode==DOWN && !DownPressed)
    DownPressed=true;
  if (keyCode==LEFT && !LeftPressed)
    LeftPressed = true;
  if (keyCode == RIGHT && !RightPressed)
    RightPressed = true;
  if (key == 'r')
    R_Pressed = true;
  if (key == 't')
    T_Pressed = true;
  if (key == 'f')
    F_Pressed = true;
  if (key == 'g')
    G_Pressed = true;
}
void keyReleased()
{

  if (keyCode==UP)
    UpPressed=false;
  if (keyCode==DOWN )
    DownPressed=false;
  if (keyCode==LEFT )
    LeftPressed = false;
  if (keyCode == RIGHT )
    RightPressed = false;
  if (key == 'r')
    R_Pressed = false;
  if (key == 't')
    T_Pressed = false;
  if (key == 'f')
    F_Pressed = false;
  if (key == 'g')
    G_Pressed = false;
}