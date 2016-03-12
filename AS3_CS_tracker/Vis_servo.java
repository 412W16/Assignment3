import java.awt.GridLayout;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.awt.event.KeyEvent;
import java.awt.event.KeyListener;
import java.awt.event.MouseEvent;
import java.awt.event.MouseListener;
import java.io.DataInputStream;
import java.io.DataOutputStream;
import java.io.IOException;

import javax.swing.JButton;
import javax.swing.JFrame;

import lejos.remote.nxt.BTConnection;
import lejos.remote.nxt.BTConnector;
import lejos.remote.nxt.NXTConnection;
import src.TrackerReader;

public class Vis_servo extends JFrame {
	public static JButton quit, connect;
	public double[] y_tar = new double[2]; // target location
	public double[] y_curr = new double[2]; // tracked object
	public static ButtonHandler bh = new ButtonHandler();
	public static DataOutputStream outData;
	public static DataInputStream dataIn;
	public static BTConnector connector;
	public static BTConnection connection;
	public int transmitReceived;

	TrackerReader tracker = new TrackerReader();

	public Vis_servo() {
		setTitle("Control");
		setBounds(650, 350, 500, 500);
		setLayout(new GridLayout(4, 5));

		connect = new JButton(" Connect ");
		connect.addActionListener(bh);
		connect.addKeyListener(bh);
		add(connect);

		quit = new JButton("Quit");
		quit.addActionListener(bh);
		add(quit);

	}

	public void main() {
		Vis_servo NXTrc = new Vis_servo();
		NXTrc.setVisible(true);
		NXTrc.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);

		tracker.start();
		while (true) {
			try {
				Thread.sleep(1000); // 1000 milliseconds is one second.
			} catch (InterruptedException ex) {
				Thread.currentThread().interrupt();
			}
			checkRequests();
		}
	}

	public void checkRequests() {
		try {
			transmitReceived = dataIn.readInt();
			if (transmitReceived == 1) {
				readTracker();
				sendValues();
			}
		} catch (IOException ioe) {
			System.out.println("IO Exception readInt");
		}
	}

	public void readTracker() {
		y_tar[0] = tracker.targetx;
		y_tar[1] = tracker.targety;
		y_curr[0] = tracker.x;
		y_curr[1] = tracker.y;
	}
	
	public void sendValues() {
		try {
			outData.writeDouble(y_tar[0]);
			outData.writeDouble(y_tar[1]);;
			outData.writeDouble(y_curr[0]);
			outData.writeDouble(y_curr[1]);
			
			outData.flush();
		}  catch (IOException ioe) {
            System.out.println("\nIO Exception writeInt");
         }

	}

	private static class ButtonHandler implements ActionListener, MouseListener, KeyListener {
		public static void connect() {
			connector = new BTConnector();
			connection = connector.connect("00:16:53:44:C1:B7", NXTConnection.LCP);

			if (connection == null) {
				System.out.println("\nCannot connect to EV3");
			}

			outData = connection.openDataOutputStream();
			System.out.println("\nEV3 is Connected");

		}// End connect

		public static void disconnect() {
			try {
				outData.close();
				connection.close();
			} catch (IOException ioe) {
				System.out.println("\nIO Exception writing bytes");
			}
			System.out.println("\nClosed data streams");

		}// End disconnect

		@Override
		public void keyPressed(KeyEvent arg0) {
			// TODO Auto-generated method stub

		}

		@Override
		public void keyReleased(KeyEvent arg0) {
			// TODO Auto-generated method stub

		}

		@Override
		public void keyTyped(KeyEvent arg0) {
			// TODO Auto-generated method stub

		}

		@Override
		public void mouseClicked(MouseEvent arg0) {
			// TODO Auto-generated method stub

		}

		@Override
		public void mouseEntered(MouseEvent arg0) {
			// TODO Auto-generated method stub

		}

		@Override
		public void mouseExited(MouseEvent arg0) {
			// TODO Auto-generated method stub

		}

		@Override
		public void mousePressed(MouseEvent arg0) {
			// TODO Auto-generated method stub

		}

		@Override
		public void mouseReleased(MouseEvent arg0) {
			// TODO Auto-generated method stub

		}

		@Override
		public void actionPerformed(ActionEvent ae) {
			if (ae.getSource() == quit) {
				System.exit(0);
			}
			if (ae.getSource() == connect) {
				connect();
			}
		}
	}// End ControlWindow class

}
