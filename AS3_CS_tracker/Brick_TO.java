

import java.io.*;

import lejos.hardware.Bluetooth;
import lejos.hardware.motor.Motor;
import lejos.remote.nxt.BTConnection;
import lejos.remote.nxt.BTConnector;
import lejos.remote.nxt.NXTConnection;
import lejos.robotics.EncoderMotor;

public class Brick_TO {

  public DataOutputStream dataOut; 
  public DataInputStream dataIn;
  public BTConnection BTLink;
  public BTConnection btLink;
  public int speed =50, turnSpeed = 50,speedBuffer, speedControl;
  public int commandValue,transmitReceived;
  public boolean[] control = new boolean[6];
  public boolean[] command = new boolean[6];
  EncoderMotor leftM;
  EncoderMotor rightM;
 
 
 public void run(EncoderMotor left, EncoderMotor right) {
	 leftM = left; rightM = right;
	 
	 connect();
	 
	  while(true)
	  { 
	   control = checkCommand();
	   speedControl = getSpeed(control);
	   move(control, speedControl);
	   }
 }
 
 public boolean[] checkCommand()//check input data
 {
    
    try {
       transmitReceived = dataIn.readInt();
       System.out.println(transmitReceived);
       if(transmitReceived == 1 || transmitReceived == 65536) {command[0] = true;}//forward
       if(transmitReceived == 10 || transmitReceived == 1048576){command[0] = false;}
       if(transmitReceived == 2 || transmitReceived == 131072) {command[1] = true;}//backward
       if(transmitReceived == 20 || transmitReceived == 2097152){command[1] = false;}
       if(transmitReceived == 3 || transmitReceived == 196608) {command[2] = true;}//leftTurn
       if(transmitReceived == 30 || transmitReceived == 3145728){command[2] = false;}
       if(transmitReceived == 4 || transmitReceived == 262144) {command[3] = true;}//rightTurn
       if(transmitReceived == 40 || transmitReceived == 4194304){command[3] = false;}
       if(transmitReceived == 5 || transmitReceived == 327680) {command[0] = false;//stop
                                  command[1] = false;
                                  command[2] = false;
                                  command[3] = false;}
       if(transmitReceived == 6) {command[4] = true;}//speed up
       if(transmitReceived == 60){command[4] = false;}
       if(transmitReceived == 7) {command[5] = true;}//slow down
       if(transmitReceived == 70){command[5] = false;}
       else{};
       }
    
    catch (IOException ioe) {
       System.out.println("IO Exception readInt");
       }
    return command;
    
 }//End checkCommand
 
 public void move(boolean[]D, int S)
 { 
  int movingSpeed;
  boolean[] direction = new boolean[4];

  direction[0] = D[0];
  direction[1] = D[1];
  direction[2] = D[2];
  direction[3] = D[3];
  
  movingSpeed = S;

  leftM.setPower(movingSpeed);
  rightM.setPower(movingSpeed);
 
  
  if(direction[0] == true)
    {
	  System.out.println("Forward");
      leftM.forward();  
      rightM.forward();
      }
  
  if(direction[1] == true)
    {
	  System.out.println("Backwards");
      leftM.backward();  
      rightM.backward();
      }
    
  if(direction[2] == true)
    {
	  System.out.println("Left");
      leftM.setPower(turnSpeed);
      rightM.setPower(turnSpeed);
      rightM.forward();
      leftM.backward();
      }
    
  if(direction[3] == true)
    {
	  System.out.println("Right");
      leftM.setPower(turnSpeed);
      rightM.setPower(turnSpeed);
      rightM.backward(); 
      leftM.forward();
      }
   
//  if(direction[0] == true && direction[2] == true)
//    {
//      speedBuffer =  (int) (movingSpeed);
//      
//      leftM.setPower(speedBuffer);
//      rightM.forward();
//      leftM.forward();
//      }
//    
//  if(direction[0] == true && direction[3] == true)
//    {
//	  speedBuffer =  (int) (movingSpeed);
//      
//      rightM.setPower(speedBuffer);
//      rightM.forward();
//      leftM.forward();
//      }
//        
//  if(direction[1] == true && direction[2] == true)
//    {
//     speedBuffer =  (int) (movingSpeed);
//      
//      leftM.setPower(speedBuffer);
//      rightM.backward();
//      leftM.backward();
//      }
//    
//  if(direction[1] == true && direction[3] == true)
//    {
//      speedBuffer =  (int) (movingSpeed);
//        
//      rightM.setPower(speedBuffer);
//      rightM.backward();
//      leftM.backward();
//      }
    
    if(direction[0] == false && direction[1] == false &&
       direction[2] == false && direction[3] == false)
    {
      leftM.stop();
      rightM.stop();
     }
   
 }//End move
 
 public void connect() 
 { 
	 System.out.println("Listeningg"); 
	 BTConnector connector = new BTConnector();
	 BTLink = (BTConnection) connector.waitForConnection(100, 0); 
	 dataOut = BTLink.openDataOutputStream(); 
	 dataIn = BTLink.openDataInputStream(); 

 }//End connect
 
 public int getSpeed(boolean[] D)
 {
    boolean accelerate, decelerate;
  
     accelerate = D[4];
     decelerate = D[5];
     
     if(accelerate == true)
     {
        speed += 10;
        command[4] = false;
        }
     
     if(decelerate == true)
     {
        speed -= 10;
        command[5] = false;
        }
     
     return speed;
     }//End getSpeed
 
}//Brick_TO Class
