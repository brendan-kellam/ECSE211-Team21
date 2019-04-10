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
	public Weigh(PollerSystem ps) throws RuntimeException, InterruptedException {
		this.ps = ps;
		this.timeThreshold = 11050;
//		float battery = Vehicle.power.getVoltage();
//		if (battery > 7.1 && battery <= 7.3) {
//			timeThreshold = 11450;
//		}
//		else if (battery > 7.3 && battery <= 7.5) {
//			timeThreshold = 11100;
//		}
//		else if (battery > 7.5 && battery <= 7.8) {
//			timeThreshold = 11000;
//		}
//		else if (battery > 7.8) {
//			timeThreshold = 10250;
//		}
//		else {
//			timeThreshold = 11700;
//		}
		
	}
	
	/**
	 * Drive until two lines have been detected, then compute the difference.
	 * @throws InterruptedException 
	 * 
	 */
//	public void weigh() throws InterruptedException {
//		ps.stop();
//		Thread.sleep(300);
//		Vehicle.LEFT_MOTOR.close();
//		Vehicle.RIGHT_MOTOR.close();
//		Thread.sleep(500);
//		
//		Vehicle.UNREG_LEFT_MOTOR = new UnregulatedMotor(LocalEV3.get().getPort("D"));
//		Vehicle.UNREG_RIGHT_MOTOR = new UnregulatedMotor(LocalEV3.get().getPort("A"));
//		
//		long[] times = new long[2];
//		advanceThroughTwoGridLines(times);
//		
//		Thread.sleep(100);
//		Vehicle.LEFT_MOTOR = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("D"));
//		Vehicle.RIGHT_MOTOR = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("A"));
//		Thread.sleep(100);
//
//		int interval = (int) (times[1] - times[0]);
//		PrintTimeInterval(times,interval);
//		isCanHeavy(interval);
//		ps.start();
//	}
//	
//	/**
//	 * Get the time between two grid lines
//	 * @param times
//	 * @throws InterruptedException
//	 */
//	public void advanceThroughTwoGridLines(long[] times) throws InterruptedException {
//		
//		/**
//		 * Stop the polling system since we can't run the odometer on our unregulated motors
//		 */
//		setPower(25,30);
//		
//		Vehicle.UNREG_LEFT_MOTOR.forward();
//		Vehicle.UNREG_RIGHT_MOTOR.forward();
//		
//		getTimeInterval(times);
//
//		Vehicle.UNREG_LEFT_MOTOR.stop();
//		Vehicle.UNREG_RIGHT_MOTOR.stop();
//		
//		Vehicle.UNREG_LEFT_MOTOR.close();
//		Vehicle.UNREG_RIGHT_MOTOR.close();
//
//
//	}
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
		PrintTimeInterval(times,interval);
		boolean heavy = isCanHeavy(interval);
		ps.start();
		return heavy;
	}
	
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
//			Sound.beep();
//			LCD.drawString("HEAVY CAN", 0, 7);
		}
		else {
			return false;
//			Sound.beepSequence();
//			LCD.drawString("LIGHT CAN", 0, 7);
		}
	}


	/*
	 * Prints out the time interval between the first and second line detection
	 */
	private void PrintTimeInterval(long[] times,int interval) {
		LCD.drawString( (int) times[0] + " is the first time of line detection", 0,0);
		LCD.drawString( (int) times[1] + " is the second time of line detection", 0,1);
		LCD.drawString(interval + " is the difference in time" , 0, 2);
	}

	/**
	 * Get the time interval between two successive lines.
	 * @param times
	 */
	private void getTimeInterval(long[] times) {
		for (int i=0;i<3;i++) {
			while (!cs.lineDetected()) {
				try {
					Thread.sleep(25);
				} catch (InterruptedException e) {
					e.printStackTrace();
				}
			};
			if (i==0) {
				times[0] = System.currentTimeMillis();
			}
			if (i==2) {
				times[1] = System.currentTimeMillis();
			}
//			Sound.beep();
			//times[i] =  System.currentTimeMillis();
			if (i!=2);
			try {
				Thread.sleep(150);
			} catch (InterruptedException e) {
				e.printStackTrace();
			}
		}
	}
	
}
