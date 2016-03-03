package brick_servo;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.motor.NXTMotor;
import lejos.hardware.port.MotorPort;
import lejos.robotics.EncoderMotor;

public class PR {
	static Brick_servo bs = new Brick_servo();
	static EV3LargeRegulatedMotor firstM = new EV3LargeRegulatedMotor(MotorPort.A);
	static EV3LargeRegulatedMotor secondM = new EV3LargeRegulatedMotor(MotorPort.C);
	static EncoderMotor thirdM = new NXTMotor(MotorPort.D);

	public static void main(String[] args) {
		bs.run(firstM, secondM);
	}
}
