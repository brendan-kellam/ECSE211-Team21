package ca.mcgill.ecse211;

import java.io.FileNotFoundException;

import ca.mcgill.ecse211.claw.Claw;
import ca.mcgill.ecse211.claw.Weigh;
import ca.mcgill.ecse211.colour.ColourDetection;
import ca.mcgill.ecse211.hardware.Vehicle;
import ca.mcgill.ecse211.light.ColorSensor;
import ca.mcgill.ecse211.localization.FallingEdgeLocalizer;
import ca.mcgill.ecse211.localization.LightLocalizerTester;
import ca.mcgill.ecse211.main.FieldSearch;
import ca.mcgill.ecse211.main.PollerSystem;
import ca.mcgill.ecse211.main.SearchArea;
import ca.mcgill.ecse211.main.FieldSearch.StartingCorner;
import ca.mcgill.ecse211.navigation.Navigator;
import ca.mcgill.ecse211.odometer.Odometer;
import ca.mcgill.ecse211.odometer.OdometerExceptions;
import ca.mcgill.ecse211.odometer.OdometryCorrection;
import ca.mcgill.ecse211.ultrasonic.UltrasonicPoller;
import ca.mcgill.ecse211.util.Board;
import ca.mcgill.ecse211.util.Display;
import ca.mcgill.ecse211.util.Log;
import ca.mcgill.ecse211.localization.DualLightLocalizer;
import lejos.hardware.Button;
import lejos.hardware.Sound;
import lejos.hardware.lcd.LCD;

public class Test {

	private static Odometer odometer;
	private static UltrasonicPoller usPoller;
	
	private static OdometryCorrection odoCorrection;
	private static ColorSensor left;
	private static ColorSensor right;

	////////////////////////////////////////////////////////////////////////
	private final double TRACK = 17.73; 

	//TODO: make sure this track value changes as the project's track changes
	//TODO: If you would like to test various track values, change the value above^
	////////////////////////////////////////////////////////////////////////

	private boolean displayEnabled = true;
	
	public Test() throws OdometerExceptions {

		// Create new vehicle configuration
		Vehicle.newConfig(new Vehicle.Configuration(2.1, TRACK));

		Vehicle.setAcceleration(200, 200);
		// Starting corner
		// Create odometer
		odometer = Odometer.getOdometer();

		// Create odometery correction | disable correction
		odoCorrection = new OdometryCorrection(Vehicle.COLOR_SENSOR_LEFT);
		odoCorrection.disableCorrection();

		// Create ultrasonic poller
		usPoller = new UltrasonicPoller(Vehicle.US_SENSOR);

		// Create new display object
		//		Display odometryDisplay = new Display(Vehicle.LCD_DISPLAY);

		FieldSearch.StartingCorner SC = FieldSearch.StartingCorner.LOWER_LEFT;

		// Initialize logging
		Log.setLogging(true, true, true, true, true);

		// Set logging to write to file
		try {
			Log.setLogWriter("TEST" + ".log");
		} catch (FileNotFoundException e1) {
			e1.printStackTrace();
		}

		
		left = new ColorSensor(Vehicle.COLOR_SENSOR_LEFT, 0.3f);
		right = new ColorSensor(Vehicle.COLOR_SENSOR_RIGHT, 0.3f);

		PollerSystem pollerSystem = PollerSystem.getInstance();
		pollerSystem.addPoller(usPoller);
		pollerSystem.addPoller(odometer);
		pollerSystem.start();

	}
	////////////////////////////////////////////////////////////////////////
	/**
	 * @throws InterruptedException 
	 * @CINDY: TESTING BEGINS HERE
	 */
	public void test() throws OdometerExceptions, InterruptedException {

		Sound.beep(); // Beep when ready
		//Press escape to start
		    
		if (displayEnabled) {
		    Display odometryDisplay = new Display(Vehicle.LCD_DISPLAY);
	        Thread odoDisplayThread = new Thread(odometryDisplay);
	        odoDisplayThread.start(); 
		}
		
		while (Button.waitForAnyPress() != Button.ID_ESCAPE);
		Sound.beepSequenceUp();
		
		/**
		 * ONLY UNCOMMENT THE ONE YOU'RE TESTING
		 */

		//		testLineDetection();
		//		testUSLocalization();
		        testDualLocalization();
		
//		        testLineDetection();
		       // testColorSensors();
		        //testColorSensorLineDetection(right);
		//		testLocalizationAtOrigin();
		//		testLocalizationNE(); //Test localization when driving northeast
		//		testLocalizationNW(); //Test localization when driving northwest
		//		testLocalizationSE(); //Test localization when driving southeast
		//		testLocalizationSW(); //Test localization when driving southwest
//				testTunnelNavigationWithCan();
		//		testForward();
//				testTunnelNavigation();
//		testClawGrab();
//				testColourDetection();
//				testScanThenGrab();
//				testTrackValue();
		// Wait so that we can measure
		//		drawOdoValues();

		//		testTime();
		while (Button.waitForAnyPress() != Button.ID_ESCAPE);
	}

	
	private void testDualLocalization() {
	    while (true) {
	        Sound.beepSequenceUp();
	        
    	    Vehicle.LEFT_MOTOR.setAcceleration(6000);
    	    Vehicle.RIGHT_MOTOR.setAcceleration(6000);
    	    DualLightLocalizer dll = new DualLightLocalizer(left, right);
    	    try {
                dll.localize(Board.Heading.N);
            } catch (OdometerExceptions e1) {
                e1.printStackTrace();
            }
    	    Sound.beepSequence();
    	    
    	    try {
                Thread.sleep(4000);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }   
	    }
	}
	
	private void testColorSensorLineDetection(ColorSensor sensor) {
	    
	    while (true) {
	        if (sensor.lineDetected()) {
	            Sound.playTone(900, 300);
	        }
	        
            try {
                Thread.sleep(30);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
 	    }
	    
	}
	
	private void testColorSensors() {
	    while (true) {
	        
	        if (left.lineDetected()) {
                Sound.playTone(900, 300);
            }
	        
	        if (right.lineDetected()) {
	            Sound.playTone(400, 300);
	        }
	        
	        try {
                Thread.sleep(30);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
	    }
	    
	}

	/**
	 * Test the time it takes to get between 2 grid lines
	 */
	private void testTime() {
		Weigh weigh = new Weigh(odometer);
		Thread thread = new Thread(weigh);
		Vehicle.setMotorSpeeds(300, 300);
		thread.run();
		while (thread.isAlive());
		Vehicle.setMotorSpeeds(0, 0);
	}

	private boolean testCanSearch() throws InterruptedException {
		//Sound.beep();
		int filter = 0;
		int maxFilter = 12;
		int maxDistance = 30;
		int sweepSpeed = 100;
		
		ColourDetection cd = new ColourDetection(1,usPoller);
		
		odometer.setXYT(0, 0, 0);

		Navigator.turnTo(90, sweepSpeed, true);
		
		while (filter < maxFilter && Math.abs(Odometer.getTheta()) < 90)  {
			Thread.sleep(20);
			if (usPoller.getDistance() < maxDistance) {
				filter++;
			}
		}

		Vehicle.setMotorSpeeds(0, 0);
		//Can detected:
		if (filter >= maxFilter) {
			if (cd.checkCanColour()) {
				//claw.gra
			}
			filter = 0;
		}
//		Sound.twoBeeps();
		return false;
	}
	/**
	 * Test the track value
	 * @throws InterruptedException
	 */
	private void testTrackValue() throws InterruptedException {

		Vehicle.setAcceleration(300, 300);
		for (int i = 1; i <= 4; i++) {
			// turn 90 degrees clockwise
			Navigator.turnTo(90 * i % 360,250);
			Thread.sleep(200);
		}
	}

	/**
	 * Tunnel navgitation with can. Will grab a can directly ahead of it, then travel through a tunnel directly ahead of it.
	 * @throws OdometerExceptions
	 */
	private void testTunnelNavigationWithCan() throws OdometerExceptions {
		Claw claw = new Claw(usPoller);
		claw.grab();
		testTunnelNavigation();
	}

	/**
	 * Tunnel navigation.  
	 * @throws OdometerExceptions
	 */
	private void testTunnelNavigation() throws OdometerExceptions {

		LightLocalizerTester uc = new LightLocalizerTester(odometer);
		//Cheat the beginning.
		Vehicle.setAcceleration(3000, 3000);
		//Cheat the beginning.
		Navigator.travelSpecificDistance(75,(int) Vehicle.RIGHT_MOTOR.getMaxSpeed());
//		while (!uc.bridgeDetected());
		Vehicle.setMotorSpeeds(550, 550);
		while (uc.bridgeDetected());

		Vehicle.setMotorSpeeds(0, 0);

	}

	/**
	 * Test the claw grab
	 * @throws InterruptedException
	 */
	private void testClawGrab() throws InterruptedException {
		Claw claw = new Claw(usPoller);
		//		claw.approachCan();

		while (true) {
			while (Button.waitForAnyPress() != Button.ID_ESCAPE);
			claw.grab();
			Thread.sleep(5000);
			claw.release();
			Sound.beep();
		}
	}

	/**
	 * Scan a can then grab it
	 */
	private void testScanThenGrab() {

		ColourDetection cd = new ColourDetection(1, usPoller);	
		Claw claw = new Claw(usPoller);

		if (cd.checkCanColour()){
			claw.grab();
		}
	}
	////////////////////////////////////////////////////////////////////////////
	/*
	 * Test the line detection:
	 * This will make the robot drive forward, beep when it crosses a line, and then drive back and repeat.
	 */
	private static void testLineDetection() throws InterruptedException, OdometerExceptions {
		int speed = 100;
		while (true) {
			Vehicle.setMotorSpeeds(speed, speed);
			while (!left.lineDetected() || !right.lineDetected()) {
				Thread.sleep(30);
			}
			
            Sound.beep();
            speed = -speed;
		}
	}
	/*
	 * Test the ultrasonic localization:
	 * This will use the falling edge localization technique to scan walls and face 45 degrees
	 */
	private void testUSLocalization() throws InterruptedException {
		FallingEdgeLocalizer ul = new FallingEdgeLocalizer(odometer,usPoller);
		ul.usLocalize();
	}


	/*
	 * Test localization at the origin. This will rotate, scan every line, and then face 0 degrees
	 */
	private LightLocalizerTester testLocalizationAtOrigin() throws OdometerExceptions, InterruptedException {
		testUSLocalization();
		LightLocalizerTester uc = new LightLocalizerTester(odometer);
		uc.lightLocalize(Board.TILE_SIZE, Board.TILE_SIZE);
		return uc;
	}

	/*
	 * Test localization before heading northwest. This will first localize at the origin,
	 *  then localize at the top left coordinate.
	 */
	private void testLocalizationNW() throws OdometerExceptions, InterruptedException{
		testLocalizationAtOrigin();
		LightLocalizerTester uc = testLocalizationAtOrigin();
		Navigator.travelTo(1.5 * Board.TILE_SIZE, 1.5 * Board.TILE_SIZE, true, true); //Travel to center
		Thread.sleep(25);
		uc.lightLocalize(1 * Board.TILE_SIZE, 2);
		Navigator.turnTo(0);
	}

	/*
	 * Test localization before heading northeast. This will first localize at the origin,
	 *  then localize at the top right coordinate.
	 */
	private void testLocalizationNE() throws OdometerExceptions, InterruptedException{
		testLocalizationAtOrigin();
		LightLocalizerTester uc = testLocalizationAtOrigin();
		Navigator.travelTo(1.5 * Board.TILE_SIZE, 1.5 * Board.TILE_SIZE, true, true); //Travel to center
		Thread.sleep(25);
		uc.lightLocalize(2 * Board.TILE_SIZE, 2);
		Navigator.turnTo(0);
	}

	/*
	 * Test localization before heading southeast. This will first localize at the origin,
	 * then localize at the bottom right coordinate.
	 */
	private void testLocalizationSE() throws OdometerExceptions, InterruptedException{
		testLocalizationAtOrigin();
		LightLocalizerTester uc = testLocalizationAtOrigin();
		Navigator.travelTo(1.5 * Board.TILE_SIZE, 1.5 * Board.TILE_SIZE, true, true); //Travel to center
		Thread.sleep(25);
		uc.lightLocalize(2 * Board.TILE_SIZE, 1);
		Navigator.turnTo(0);
	}

	/*
	 * 	Test localization before heading southwest. This will first localize at the origin,
	 *  then localize at the bottom left coordinate.
	 */
	private void testLocalizationSW() throws OdometerExceptions, InterruptedException{
		testLocalizationAtOrigin();
		LightLocalizerTester uc = testLocalizationAtOrigin();
		Navigator.travelTo(1.5 * Board.TILE_SIZE, 1.5 * Board.TILE_SIZE, true, true); //Travel to center
		Thread.sleep(25);
		uc.lightLocalize(1 * Board.TILE_SIZE, 1);
		Navigator.turnTo(0);
	}

	/*
	 * This will test the colour detection. It will stay still, then if a can is in front of the robot,
	 * the robot will advance and detect the colour, then print it to the display. It will do this infinitely. 
	 */
	private void testColourDetection() {
		ColourDetection cd = new ColourDetection(1,usPoller);		
		cd.testCanColours();


	}

	/**
	 * Draw the odometer values on the display
	 */
	private void drawOdoValues() {
		LCD.clear();
		LCD.drawString("X value: " + odometer.getXYT()[0], 0, 0);
		LCD.drawString("Y value: " + odometer.getXYT()[1], 0, 1);
		LCD.drawString("Theta value: " + odometer.getXYT()[2], 0, 2);
	}
}

