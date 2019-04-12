package ca.mcgill.ecse211.claw;

import ca.mcgill.ecse211.main.PollerSystem;
import ca.mcgill.ecse211.odometer.Odometer;
import ca.mcgill.ecse211.sensor.ColorSensor;
import ca.mcgill.ecse211.util.Vehicle;
import lejos.hardware.Sound;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.LCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.motor.UnregulatedMotor;

public class Weigh {

	private PollerSystem ps;
	private ColorSensor cs = Vehicle.LEFT_CS;
	
	public enum Weight{
		HEAVY,
		LIGHT
	}
	
	private int timeThreshold;
	/**
	 * Construct the weighing system. This will require the polling system in order to turn off odometry, as well as initialize a time threshold.
	 * @param ps
	 * @throws RuntimeException
	 * @throws InterruptedException
	 */
	public Weigh(PollerSystem ps) throws RuntimeException, InterruptedException {
		this.ps = ps;
		this.timeThreshold = 11050;
	}
	

	/**
	 * Turn off the regulated motors, turn on the unregulated ones. Enter the tunnel in order to weigh the can.
	 * @return true if the can is heavy, false otherwise.
	 * @throws InterruptedException
	 */
	public boolean weighThroughTunnel() throws InterruptedException {

		ps.stop();
		Thread.sleep(300);
		Vehicle.LEFT_MOTOR.close();
		Vehicle.RIGHT_MOTOR.close();
		Thread.sleep(500);
		
		Vehicle.UNREG_LEFT_MOTOR = new UnregulatedMotor(LocalEV3.get().getPort("D"));
		Vehicle.UNREG_RIGHT_MOTOR = new UnregulatedMotor(LocalEV3.get().getPort("A"));
		
		long[] times = new long[2];
		Thread.sleep(150);
		tunnelTraversal(times);
		
		Sound.beep();
		Thread.sleep(100);
		Vehicle.LEFT_MOTOR = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("D"));
		Vehicle.RIGHT_MOTOR = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("A"));
		Thread.sleep(100);

		int interval = (int) (times[1] - times[0]);
		boolean heavy = isCanHeavy(interval);
		ps.start();
		return heavy;
	}
	
	/**
	 * Travel through the tunnel, taking the time required for tunnel traversal. If it is holding a heavy can, the time will be greater than the threshold.
	 * @param times
	 * @throws InterruptedException
	 */
	public void tunnelTraversal(long[] times) throws InterruptedException {
		/**
		 * Stop the polling system since we can't run the odometer on our unregulated motors
		 */
		setPower(60,63);


		Vehicle.UNREG_LEFT_MOTOR.forward();
		Vehicle.UNREG_RIGHT_MOTOR.forward();
		
		while (Vehicle.LEFT_CS.fetchNormalizedSample() > 0.31) {
			Vehicle.UNREG_LEFT_MOTOR.forward();
			Vehicle.UNREG_RIGHT_MOTOR.forward();
			Thread.sleep(30);
		}
		
		Thread.sleep(100);
		setPower(25,29);
		times[0] = System.currentTimeMillis();
		while (Vehicle.LEFT_CS.tunnelDetected()) {
			Vehicle.UNREG_LEFT_MOTOR.forward();
			Vehicle.UNREG_RIGHT_MOTOR.forward();
			Thread.sleep(30);
		}
		
		Thread.sleep(250); //Give it a bit of time to exit the tunnel
		
		
		Vehicle.UNREG_LEFT_MOTOR.stop();
		Vehicle.UNREG_RIGHT_MOTOR.stop();
		times[1] = System.currentTimeMillis();
		
		
		Vehicle.UNREG_LEFT_MOTOR.close();
		Vehicle.UNREG_RIGHT_MOTOR.close();
	}
	
	/**
	 * Set the power of the unregulated motors. There is more weight on the right side, so to compensate we will be passing a larger power value
	 * @param leftPower 
	 * @param rightPower
	 */
	private void setPower(int leftPower, int rightPower) {
	Vehicle.UNREG_LEFT_MOTOR.setPower(leftPower);
	Vehicle.UNREG_RIGHT_MOTOR.setPower(rightPower);
	}
	
	/**
	 * Determine the weight of a can.
	 * @param interval the time interval between the two line detections
	 */
	private boolean isCanHeavy(int interval) {
		if (interval> timeThreshold) {
			return true;
		}
		else {
			return false;
		}
	}
}
