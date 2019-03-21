package ca.mcgill.ecse211;

import java.io.FileNotFoundException;

import ca.mcgill.ecse211.colour.ColourDetection;
import ca.mcgill.ecse211.hardware.Vehicle;
import ca.mcgill.ecse211.localization.FallingEdgeLocalizer;
import ca.mcgill.ecse211.localization.LightLocalizerTester;
import ca.mcgill.ecse211.main.FieldSearch;
import ca.mcgill.ecse211.main.PollerSystem;
import ca.mcgill.ecse211.navigation.Navigator;
import ca.mcgill.ecse211.odometer.Odometer;
import ca.mcgill.ecse211.odometer.OdometerExceptions;
import ca.mcgill.ecse211.odometer.OdometryCorrection;
import ca.mcgill.ecse211.ultrasonic.UltrasonicPoller;
import ca.mcgill.ecse211.util.Board;
import ca.mcgill.ecse211.util.Log;
import lejos.hardware.Button;
import lejos.hardware.Sound;
import lejos.hardware.lcd.LCD;

public class Test {

	private static Odometer odometer;
	private static UltrasonicPoller usPoller;
	
	////////////////////////////////////////////////////////////////////////
	private final double TRACK = 14.3; 
	
	//TODO: make sure this track value changes as the project's track changes
	//TODO: If you would like to test various track values, change the value above^
	////////////////////////////////////////////////////////////////////////
	
	public Test() throws OdometerExceptions {

		// Create new vehicle configuration
		Vehicle.newConfig(new Vehicle.Configuration(2.1, TRACK));
		Vehicle.LEFT_MOTOR.setAcceleration(200);
		Vehicle.RIGHT_MOTOR.setAcceleration(200);

		// Starting corner
		// Create odometer
		odometer = Odometer.getOdometer();

		// Create odometery correction | disable correction
		OdometryCorrection odoCorrection = new OdometryCorrection(Vehicle.COLOR_SENSOR_BACK);
		odoCorrection.disableCorrection();

		// Create ultrasonic poller
		usPoller = new UltrasonicPoller(Vehicle.US_SENSOR);

		// Create new display object
		//		Display odometryDisplay = new Display(Vehicle.LCD_DISPLAY);

		FieldSearch.StartingCorner SC = FieldSearch.StartingCorner.LOWER_LEFT;


		// Initialize logging
		Log.setLogging(true, false, false, false, false);

		// Set logging to write to file
		try {
			Log.setLogWriter("Lab5" + ".log");
		} catch (FileNotFoundException e1) {
			e1.printStackTrace();
		}

		// Start odometer correction thread
		//		Thread odoCorrectionThread = new Thread(odoCorrection);
		//		odoCorrectionThread.start();


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
		while (Button.waitForAnyPress() != Button.ID_ESCAPE);

		/**
		 * ONLY UNCOMMENT THE ONE YOU'RE TESTING
		 */

//		testLineDetection();
//		testUSLocalization();
//		testLocalizationAtOrigin();
//		testLocalizationNE(); //Test localization when driving northeast
//		testLocalizationNW(); //Test localization when driving northwest
//		testLocalizationSE(); //Test localization when driving southeast
//		testLocalizationSW(); //Test localization when driving southwest
		testColourDetection();

		// Wait so that we can measure
		drawOdoValues();
		while (Button.waitForAnyPress() != Button.ID_ESCAPE);
	}

	////////////////////////////////////////////////////////////////////////////
	/*
	 * Test the line detection:
	 * This will make the robot drive forward, beep when it crosses a line, and then drive back and repeat.
	 */
	private static void testLineDetection() throws InterruptedException, OdometerExceptions {
		int speed = 200;
		LightLocalizerTester uc = new LightLocalizerTester(odometer);
		while (true) {
			Vehicle.setMotorSpeeds(speed, speed);
			if (uc.lineDetected()) {
				Sound.beep();
				Thread.sleep(300);
				speed = -speed;
			}
		}
	}
	/*
	 * Test the ultrasonic localization:
	 * This will use the falling edge localization technique to scan walls and face 45 degrees
	 */
	private void testUSLocalization() {
		FallingEdgeLocalizer ul = new FallingEdgeLocalizer(odometer,usPoller);
		ul.usLocalize();
	}


	/*
	 * Test localization at the origin. This will rotate, scan every line, and then face 0 degrees
	 */
	private LightLocalizerTester testLocalizationAtOrigin() throws OdometerExceptions {
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
		ColourDetection cd = new ColourDetection(1);
		cd.testCanColours();
	}

	private void drawOdoValues() {
		LCD.clear();
		LCD.drawString("X value: " + odometer.getXYT()[0], 0, 0);
		LCD.drawString("Y value: " + odometer.getXYT()[1], 0, 1);
		LCD.drawString("Theta value: " + odometer.getXYT()[2], 0, 2);
	}
}

