

import java.io.DataInputStream;
import java.io.DataOutputStream;
import java.io.IOException;

import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.remote.nxt.BTConnection;
import lejos.remote.nxt.BTConnector;
import lejos.utility.Matrix;

public class Brick_servo {

	public double[] y_tar = new double[2]; // target location
	public double[] y_curr = new double[2]; // tracked object
	public double[] e = new double[2]; // y0-y*
	public double[] x = new double[2]; // Joint angles
	public double[] deltaX = new double[2];
	public double deltaT = 0.25;
	public double lambda = -0.5;
	double[][] A = new double[2][2];
	Matrix J = new Matrix(A);

	EV3LargeRegulatedMotor firstM;
	EV3LargeRegulatedMotor secondM;
	EV3UltrasonicSensor sensor;
	public DataOutputStream dataOut;
	public DataInputStream dataIn;
	public BTConnection BTLink;
	public BTConnection btLink;
	public double transmitReceived;
	
	boolean block;
	float[] dist;
	
//	double alpha = .001;

	public void run(EV3LargeRegulatedMotor f, EV3LargeRegulatedMotor s, EV3UltrasonicSensor _sensor) {
		firstM = f;
		secondM = s;
		sensor = _sensor;

		connect();
		
		block = false;

		// intialize jacobian
		initJacobian();
		block = true;
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
			updateX();
			moveJoint(firstM, x[0]);
			moveJoint(secondM, x[1]);
		}
	}

	public void initJacobian() {
		
		readTracker();
		double[] pos1 = new double[2];
		double[] pos2 = new double[2];
		
		pos1[0] = y_curr[0];
		pos1[1] = y_curr[1];
		moveJoint(firstM, -30);
		
		readTracker();
		pos2[0] = y_curr[0];
		pos2[1] = y_curr[1];
		
		J.set(0, 0, (pos2[0]-pos1[0])/-30);
		J.set(0, 1, (pos2[1]-pos1[1])/-30);
		
		pos1[0] = y_curr[0];
		pos1[1] = y_curr[1];
		moveJoint(secondM, 30);
		
		readTracker();
		pos2[0] = y_curr[0];
		pos2[1] = y_curr[1];
		
		J.set(1, 0, (pos2[0]-pos1[0])/30);
		J.set(1, 1, (pos2[1]-pos1[1])/30);
	}

	public void readTracker() {
		if (block){
			dist = new float[1];
			sensor.getDistanceMode().fetchSample(dist, 0);
			if (dist[0] <= 0.1) {
				evasiveMove();
				block = false;
				sendRequest();
				checkResponse();
			} else {
				sendRequest();
				checkResponse();
			}
		} else {
			sendRequest();
			checkResponse();
		}
		//System.out.println("done reading");
	}
	
	private void evasiveMove() {
		// TODO Auto-generated method stub
		moveJoint(firstM, 90);
		moveJoint(secondM, 100);
		moveJoint(firstM, -90);
		moveJoint(secondM, -20);
		
	}

	public void sendRequest() {
		try {
			//System.out.println("request made");
			dataOut.writeInt(1);
			dataOut.flush();
		}  catch (IOException ioe) {
            System.out.println("\nIO Exception writeInt");
         }
	}
	
	public void checkResponse() {
		while (true) {
			try {
				int check = dataIn.readInt();
				transmitReceived = dataIn.readDouble();
				if(check == 0) {
					y_tar[0] = transmitReceived;
				} else if (check == 1) {
					y_tar[1] = transmitReceived;
				} else if (check == 2) {
					y_curr[0] = transmitReceived;
				} else if (check == 3) {
					y_curr[1] = transmitReceived;
					break;
				}
				
			} catch (IOException ioe) {
				System.out.println("IO Exception readInt");
			}
		}
	}

	public void currentConfig() {
		x[0] = firstM.getTachoCount();
		x[1] = secondM.getTachoCount();
	}

//	public void bUpdate() {
//		
//		double[][] dx = new double[2][1];
//		dx[0][0] = deltaX[0];
//		dx[1][0] = deltaX[1];
//		
//		Matrix x = new Matrix(dx);
//		
//		Matrix t = J.times(x);
//		
//		double[][] d_temp = new double[2][1];
//		d_temp[0][0] = e[0];
//		d_temp[1][0] = e[1];
//		Matrix e_temp = new Matrix(d_temp);
//		
//		t = e_temp.minus(t);
//		
//		Matrix numerator = t.times(x.transpose());
//		
//		Matrix denominator = x.transpose().times(x);
//		double denom = denominator.get(0, 0);
//		//denom = 1/denom;
//		System.out.println("Denom");
//		System.out.println(denom);
//		J = J.plus(numerator.times(denom).times(alpha));
//	}

	public void computeError() {
		e[0] = y_tar[0] - y_curr[0];
		e[1] = y_tar[1] - y_curr[1];
	}

//	public void updateJ() {
//		J.set(0, 0, e[0] / deltaX[0]);
//		J.set(1, 0, e[1] / deltaX[0]);
//		J.set(0, 1, e[0] / deltaX[1]);
//		J.set(1, 1, e[1] / deltaX[1]);
//	}

	public void updateX() {
		Matrix j_temp = J.inverse();
		double[][] d_temp = new double[2][1];
		d_temp[0][0] = e[0];
		d_temp[1][0] = e[1];
		Matrix e_temp = new Matrix(d_temp);
		Matrix s = j_temp.times(e_temp).times(lambda);

		deltaX[0] = s.get(0, 0);
		deltaX[1] = s.get(1, 0);

		//System.out.println(x[0]+deltaX[0]*deltaT);
		x[0] = x[0] + deltaX[0] * deltaT;
		x[1] = x[1] + deltaX[1] * deltaT;
	}

	public void moveJoint(EV3LargeRegulatedMotor joint, double theta) {
		joint.setSpeed(90);
		joint.rotateTo((int) theta);
		joint.stop();
//		joint.resetTachoCount();
	}

	public void connect() {
		System.out.println("Listening");
		BTConnector connector = new BTConnector();
		BTLink = (BTConnection) connector.waitForConnection(100, 0);
		dataOut = BTLink.openDataOutputStream();
		dataIn = BTLink.openDataInputStream();

	}// End connect

}
