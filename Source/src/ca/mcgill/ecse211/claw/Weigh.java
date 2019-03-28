package ca.mcgill.ecse211.claw;

import ca.mcgill.ecse211.localization.LightLocalizerTester;
import ca.mcgill.ecse211.odometer.Odometer;
import ca.mcgill.ecse211.odometer.OdometerExceptions;
import lejos.hardware.lcd.LCD;

public class Weigh implements Runnable {

	Odometer odometer;
	
	public enum Weight{
		HEAVY,
		LIGHT
	}
	
	private final int timeThreshold = 10;
	
	public Weigh(Odometer odometer) {
		this.odometer = odometer;
	}
	
	
	/**
	 * Begin the thread that will run until 2 lines have been detected, and compute the difference.
	 * 
	 */
	public void run() {
		LightLocalizerTester lt = null;
		long[] times = new long[2];
		
		try {
			lt = new LightLocalizerTester(odometer);
		} catch (OdometerExceptions e) {
			e.printStackTrace();
		}
		
		getTimeInterval(lt, times);

		int interval = (int) (times[1] - times[0]);
		PrintTimeInterval(times,interval);
//		DetermineWeight(interval);
	}
	
	private void DetermineWeight(int interval) {
		if (interval> timeThreshold) {
			LCD.drawString("HEAVY CAN", 0, 4);
		}
		else {
			LCD.drawString("LIGHT CAN", 0, 4);
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

	/*
	 * Get the time interval between two 
	 */
	private void getTimeInterval(LightLocalizerTester lt, long[] times) {
		for (int i=0;i<2;i++) {
			while (!lt.lineDetected());
			times[i] =  System.currentTimeMillis();
		}
	}
	
}
