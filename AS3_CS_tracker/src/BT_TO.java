package src;

/*
 * Original Source: Tawat Atigarbodee September 21, 2009
 */

import javax.swing.*;
import java.awt.*;
import java.awt.event.*;
import java.io.*;

import lejos.hardware.ev3.EV3;
import lejos.remote.nxt.BTConnection;
import lejos.remote.nxt.BTConnector;
import lejos.remote.nxt.NXTConnection;

public class BT_TO extends JFrame
{
  public static JButton quit, connect;
  public static JButton forward,reverse, leftTurn, rightTurn, stop, speedUp, slowDown;
  public static JLabel L1,L2,L3,L4,L5,L6,L7,L8,L9,L10;
  public static ButtonHandler bh = new ButtonHandler();
  public static DataOutputStream outData;
  public static DataInputStream dataIn;
  public static BTConnector connector;
  public static BTConnection connection;
  
  public static double[] y_tar = new double[2]; // target location
  public static double[] y_curr = new double[2]; // tracked object
  static TrackerReader tracker = new TrackerReader();
  public static int transmitReceived;


  
  
  public BT_TO()
  { 
    setTitle ("Control");
    setBounds(650,350,500,500);
    setLayout(new GridLayout(4,5));

    connect = new JButton(" Connect ");
    connect.addActionListener(bh);
    connect.addKeyListener(bh);
    add(connect);

    L9 = new JLabel("");
    add(L9);
    L10 = new JLabel("");
    add(L10);
    
    quit = new JButton("Quit");
    quit.addActionListener(bh);
    add(quit);

  }
  
  public static void main(String[] args)
  {
	  tracker.start();
     BT_TO NXTrc = new BT_TO();
     NXTrc.setVisible(true);
     NXTrc.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE); 
    
  }//End main
  
  public static void checkRequests() {
	  System.out.println("checking requests");
		try {
			transmitReceived = dataIn.readInt();
			System.out.println(transmitReceived);
			readTracker();
			sendValues();
		} catch (IOException ioe) {
			System.out.println("IO Exception readInt");
		}
	}
  
  public static void readTracker() {
		y_tar[0] = tracker.targetx;
		y_tar[1] = tracker.targety;
		y_curr[0] = tracker.x;
		y_curr[1] = tracker.y;
	}
	
	public static void sendValues() {
		try {
			outData.writeInt(0);
			outData.writeDouble(y_tar[0]);
			outData.writeInt(1);
			outData.writeDouble(y_tar[1]);
			outData.writeInt(2);
			outData.writeDouble(y_curr[0]);
			outData.writeInt(3);
			outData.writeDouble(y_curr[1]);
			
			outData.flush();
		}  catch (IOException ioe) {
          System.out.println("\nIO Exception writeInt");
       }

	}
  
  private static class ButtonHandler implements ActionListener, MouseListener, KeyListener
  {
//***********************************************************************
  //Buttons action
    public void actionPerformed(ActionEvent ae)
    {
      if(ae.getSource() == quit)  {System.exit(0);}
      if(ae.getSource() == connect) {connect();}
    }
     

//***********************************************************************
//Mouse actions
   public void mouseClicked(MouseEvent arg0) {}

   public void mouseEntered(MouseEvent arg0) {}

   public void mouseExited(MouseEvent arg0) {}

   public void mousePressed(MouseEvent moe) 
   {   
        
   }//End mousePressed

   public void mouseReleased(MouseEvent moe) 
   {
       
      
   }//End mouseReleased

//***********************************************************************
//Keyboard action
   public void keyPressed(KeyEvent ke) {}//End keyPressed

   public void keyTyped(KeyEvent ke) 
   {
      
   }//End keyTyped
   
   public void keyReleased(KeyEvent ke) 
   {
      
   }//End keyReleased

  }//End ButtonHandler
  
  public static void connect()
  {
	 connector = new BTConnector();
     connection = connector.connect("00:16:53:44:C1:B7", NXTConnection.LCP);
    
     if (connection == null)
     {
        System.out.println("\nCannot connect to EV3");
     }
     
     dataIn = connection.openDataInputStream();
     outData = connection.openDataOutputStream();
     System.out.println("\nEV3 is Connected");  
     
    
     
	while (true) {
		try {
			Thread.sleep(1000); // 1000 milliseconds is one second.
		} catch (InterruptedException ex) {
			Thread.currentThread().interrupt();
		}
		System.out.println(tracker.x + " " + tracker.y);
		checkRequests();
	}
     
  }//End connect
  
  public static void disconnect()
  {
     try{
        outData.close();
        connection.close();
        } 
     catch (IOException ioe) {
        System.out.println("\nIO Exception writing bytes");
     }
     System.out.println("\nClosed data streams");
     
  }//End disconnect
}//End ControlWindow class

