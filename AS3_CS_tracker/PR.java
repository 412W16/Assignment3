
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.motor.NXTMotor;
import lejos.hardware.port.MotorPort;
import lejos.hardware.port.SensorPort;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.robotics.EncoderMotor;

public class PR {
	static Brick_servo bs = new Brick_servo();
	static EV3LargeRegulatedMotor firstM = new EV3LargeRegulatedMotor(MotorPort.A);
	static EV3LargeRegulatedMotor secondM = new EV3LargeRegulatedMotor(MotorPort.C);
	static EncoderMotor thirdM = new NXTMotor(MotorPort.D);
	static EV3UltrasonicSensor sensor = new EV3UltrasonicSensor(SensorPort.S2);

	public static void main(String[] args) {
		bs.run(firstM, secondM, sensor);
	}
}
