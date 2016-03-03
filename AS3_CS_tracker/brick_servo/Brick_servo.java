package brick_servo;

import java.io.DataInputStream;
import java.io.DataOutputStream;
import java.io.IOException;

import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.remote.nxt.BTConnection;
import lejos.remote.nxt.BTConnector;
import lejos.utility.Matrix;

public class Brick_servo {

	public double[] y_tar = new double[2]; // target location
	public double[] y_curr = new double[2]; // tracked object
	public double[] e = new double[2]; // y0-y*
	public double[] x = new double[2]; // Joint angles
	public double[] deltaX = new double[2];
	public int deltaT = 5;
	public double lambda = -0.5;
	double[][] A = new double[2][2];
	Matrix J = new Matrix(A);

	EV3LargeRegulatedMotor firstM;
	EV3LargeRegulatedMotor secondM;
	public DataOutputStream dataOut;
	public DataInputStream dataIn;
	public BTConnection BTLink;
	public BTConnection btLink;
	public double transmitReceived;

	public double[] control = new double[6];

	public void run(EV3LargeRegulatedMotor f, EV3LargeRegulatedMotor s) {
		firstM = f;
		secondM = s;

		connect();

		// intialize jacobian
		initJacobian();

		// Move arm to point
		while (true) {
			try {
				Thread.sleep(1000); // 1000 milliseconds is one second.
			} catch (InterruptedException ex) {
				Thread.currentThread().interrupt();
			}
			readTracker();
			currentConfig();
			computeError();
			// bUpdate();
			updateX();
			moveJoint(firstM, x[0]);
			moveJoint(secondM, x[1]);
		}
	}

	public void initJacobian() {
		J.set(0, 0, 1);
		J.set(0, 1, 1);
		J.set(1, 0, 1);
		J.set(1, 1, 1);
		// while (true) {
		// try {
		// Thread.sleep(1000);
		// } catch (InterruptedException ex) {
		// Thread.currentThread().interrupt();
		// }
		// }
	}

	public void readTracker() {
		sendRequest();
		checkResponse();
	}
	
	public void sendRequest() {
		try {
			dataOut.writeInt(1);
			dataOut.flush();
		}  catch (IOException ioe) {
            System.out.println("\nIO Exception writeInt");
         }
	}
	
	public void checkResponse() {
		double[][] resp = new double[2][2];
		while (true) {
			try {
				transmitReceived = dataIn.readDouble();
				System.out.println("data recieved");
			} catch (IOException ioe) {
				System.out.println("IO Exception readInt");
			}
		}
	}

	public void currentConfig() {
		x[0] = firstM.getTachoCount();
		x[1] = secondM.getTachoCount();
	}

	public void bUpdate() {

	}

	public void computeError() {
		e[0] = y_tar[0] - y_curr[0];
		e[1] = y_tar[1] - y_curr[1];
	}

	public void updateJ() {
		J.set(0, 0, e[0] / deltaX[0]);
		J.set(1, 0, e[1] / deltaX[0]);
		J.set(0, 1, e[0] / deltaX[1]);
		J.set(1, 1, e[1] / deltaX[1]);
	}

	public void updateX() {
		Matrix j_temp = J.inverse();
		double[][] d_temp = new double[2][1];
		d_temp[0][0] = e[0];
		d_temp[1][0] = e[1];
		Matrix e_temp = new Matrix(d_temp);
		Matrix s = j_temp.times(e_temp).times(lambda);

		deltaX[0] = s.get(0, 0);
		deltaX[1] = s.get(1, 0);
		x[0] = x[0] + deltaX[0] * deltaT;
		x[1] = x[1] + deltaX[1] * deltaT;
	}

	public void moveJoint(EV3LargeRegulatedMotor joint, double theta) {
		joint.setSpeed(90);
		joint.rotateTo((int) theta);
		joint.stop();
		joint.resetTachoCount();
	}

	public void connect() {
		System.out.println("Listeningg");
		BTConnector connector = new BTConnector();
		BTLink = (BTConnection) connector.waitForConnection(100, 0);
		dataOut = BTLink.openDataOutputStream();
		dataIn = BTLink.openDataInputStream();

	}// End connect

}
