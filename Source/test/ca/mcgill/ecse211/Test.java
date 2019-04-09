package ca.mcgill.ecse211;

import java.io.FileNotFoundException;

import ca.mcgill.ecse211.claw.Claw;
import ca.mcgill.ecse211.claw.Weigh;
import ca.mcgill.ecse211.localization.FallingEdgeLocalizer;
import ca.mcgill.ecse211.localization.LineRunner;
import ca.mcgill.ecse211.localization.DualLightLocalizer.Config;
import ca.mcgill.ecse211.main.PollerSystem;
import ca.mcgill.ecse211.navigation.Navigator;
import ca.mcgill.ecse211.odometer.OdometerDisplay;
import ca.mcgill.ecse211.odometer.Odometer;
import ca.mcgill.ecse211.odometer.OdometerExceptions;
import ca.mcgill.ecse211.sensor.ColorSensor;
import ca.mcgill.ecse211.sensor.ColourDetection;
import ca.mcgill.ecse211.sensor.UltrasonicPoller;
import ca.mcgill.ecse211.util.Board;
import ca.mcgill.ecse211.util.EV3Math;
import ca.mcgill.ecse211.util.Log;
import ca.mcgill.ecse211.util.Tile;
import ca.mcgill.ecse211.util.Vehicle;
import ca.mcgill.ecse211.util.WifiController;
import ca.mcgill.ecse211.util.Board.Heading;
import ca.mcgill.ecse211.util.Log.Sender;
import ca.mcgill.ecse211.localization.DualLightLocalizer;
import lejos.hardware.Button;
import lejos.hardware.Sound;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.LCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.motor.UnregulatedMotor;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.hardware.sensor.SensorModes;
import lejos.robotics.EncoderMotor;
import lejos.robotics.SampleProvider;
import lejos.robotics.geometry.Rectangle;

public class Test {

	private static Odometer odometer;
	private static UltrasonicPoller usPoller;

	private static ColorSensor left;
	private static ColorSensor right;
	private static PollerSystem pollerSystem;

	////////////////////////////////////////////////////////////////////////
	private final double TRACK = 17.73; 

	//TODO: make sure this track value changes as the project's track changes
	//TODO: If you would like to test various track values, change the value above^
	////////////////////////////////////////////////////////////////////////

	private boolean displayEnabled = false;

	public Test() throws OdometerExceptions {

		// Create new vehicle configuration
		Vehicle.newConfig(new Vehicle.Configuration(2.1, TRACK));

		// Starting corner
		// Create odometer
		odometer = Odometer.getOdometer();

		// Create ultrasonic poller
		usPoller = new UltrasonicPoller(Vehicle.US_SENSOR);

		// Create new display object
		//		Display odometryDisplay = new Display(Vehicle.LCD_DISPLAY);


		// Initialize logging
		Log.setLogging(true, true, false, true, true, true);

		// Set logging to write to file
		try {
			Log.setLogWriter("TEST" + ".log");
		} catch (FileNotFoundException e1) {
			e1.printStackTrace();
		}


		left = new ColorSensor(Vehicle.COLOR_SENSOR_LEFT, 0.3f);
		right = new ColorSensor(Vehicle.COLOR_SENSOR_RIGHT, 0.3f);


		pollerSystem = PollerSystem.getInstance();
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
			OdometerDisplay odometryDisplay = new OdometerDisplay(Vehicle.LCD_DISPLAY);
			Thread odoDisplayThread = new Thread(odometryDisplay);
			odoDisplayThread.start(); 
		}

		while (Button.waitForAnyPress() != Button.ID_ESCAPE);
		//		Sound.beepSequenceUp();

		/**
		 * ONLY UNCOMMENT THE ONE YOU'RE TESTING
		 */


		//		testLineDetection();
		//				testUSLocalization();
		//		        testDualLocalization();
		//		testTime();
		testTimeWithCan();
		//		testTravelToTunnel();
		//		        testLineDetection();
		// testColorSensors();
		//testColorSensorLineDetection(right);
		//				testUSLocalization();
		//		        testDualLocalization();
		//		testUSLocalization();
		//		        testDualLocalization();
		//		        testLocalizeToSquare();
		//		        testLineTravel(); 
		//travelTo(Board.TILE_SIZE+Board.TILE_SIZE/2, Board.TILE_SIZE+Board.TILE_SIZE/2);
		//		        testShuffle();

		//		        testLineDetection();
		// testColorSensors();
		//testColorSensorLineDetection(right);
		//		testLocalizationAtOrigin();
		//		testLocalizationNE(); //Test localization when driving northeast
		//		testLocalizationNW(); //Test localization when driving northwest
		//		testLocalizationSE(); //Test localization when driving southeast
		//		testLocalizationSW(); //Test localization when driving southwest
		//				testTunnelNavigationWithCan();
		testTunnelNavigationWithCan();
		//		testForward();
		//				testTunnelNavigation();
		//		testClawGrab();
		//				testColourDetection();
		//				testScanThenGrab();
		//						testTrackValue();
		//		testSearchForCan();
		//testDriveToLine();
		//testDriveInSquare();
		//		testDualLocalization();
		//		testLocalizeToTile();
		//testLightSensorContainment();
		// Wait so that we can measure
		//		drawOdoValues();
		//		testFallingEdgeThenLocalize();

		//		testTime();
		while (Button.waitForAnyPress() != Button.ID_ESCAPE);
	}

	private void testLightSensorContainment() {

		Tile t = Tile.lowerLeft(2, 2);


		double xa = 0.0, ya = 0.0;
		double xb = 0.0, yb = 0.0;

		try {
			Navigator.travelTo(Board.TILE_SIZE*2, Board.TILE_SIZE*2, true, true);
		} catch (OdometerExceptions e1) {
			e1.printStackTrace();
		}

		Vehicle.setMotorSpeeds(-100, 100);

		while (true) {

			xa = ColorSensor.getX(Vehicle.LEFT_CS);
			ya = ColorSensor.getY(Vehicle.LEFT_CS);

			xb = ColorSensor.getX(Vehicle.RIGHT_CS);
			yb = ColorSensor.getY(Vehicle.RIGHT_CS);

			if (t.contains(xa, ya)) {
				Sound.beep();
			}

			if (t.contains(xb, yb)) {
				Sound.buzz();
			}

			//Log.log(Sender.board, "left CS (" + xa + ", " + ya + ")");


			try {
				Thread.sleep(50);
			} catch (InterruptedException e) {
				e.printStackTrace();
			}
		}

		// Vehicle.setMotorSpeeds(0, 0);
	}

	private void testLocalizeToTile() {

		DualLightLocalizer dll = new DualLightLocalizer(Vehicle.LEFT_CS, Vehicle.RIGHT_CS);

		Tile t = Tile.upperLeft(0, 2);

		try {
			Navigator.travelTo(t.getCenter().getX(), t.getCenter().getY(), true, true);
		} catch (OdometerExceptions e) {
			e.printStackTrace();
		}

		dll.localizeToTile(Heading.N, Heading.W, Heading.S);
		try {
			testTunnelNavigation();
		} catch (OdometerExceptions e) {
			e.printStackTrace();
		}
		dll.localizeToTile(Heading.N, Heading.W, Heading.N);

		Sound.beepSequence();
	}

	private void testDriveToLine() {

		//Navigator.travelSpecificDistance(Board.TILE_SIZE + Board.TILE_SIZE/2);

		DualLightLocalizer dll = new DualLightLocalizer(Vehicle.LEFT_CS, Vehicle.RIGHT_CS);
		Log.log(Sender.board, "Odometer X and Y: (" + Odometer.getX() + ", " + Odometer.getY() + ")");

		double angle = 90.0;

		for (int i = 0; i < 4; i++) {
			dll.travelToLine(200);
			Log.log(Sender.board, "Odometer X and Y: (" + Odometer.getX() + ", " + Odometer.getY() + ")");
			Log.log(Sender.board, "Odometer theta: " + Odometer.getTheta());

			// Go back to origin
			Sound.beep();

			Navigator.travelSpecificDistance(- (Vehicle.VERT_DIST_FROM_LIGHT_SENSORS_TO_WHEEL_BASE));
			Navigator.turnTo(angle);

			angle += 90.0;
		}



	}

	private void testDriveInSquare() {

		DualLightLocalizer dll = new DualLightLocalizer(Vehicle.LEFT_CS, Vehicle.RIGHT_CS);
		double angle = 90.0;

		for (int i = 0; i < 4; i++) {

			for (int j = 0; j < 3; j++) {
				dll.travelToLine(200);
				Log.log(Sender.board, "Odometer X and Y: (" + Odometer.getX() + ", " + Odometer.getY() + ")");
				Log.log(Sender.board, "Odometer theta: " + Odometer.getTheta());

			}

			Navigator.turnTo(angle);
			angle+= 90.0;
		}

		Sound.beepSequenceUp();
		try {
			Navigator.travelTo(Board.TILE_SIZE, Board.TILE_SIZE, true, true);
		} catch (OdometerExceptions e) {
			e.printStackTrace();
		}

	}

	private void testTravelToTunnel() throws OdometerExceptions {

		// Vehicle.setMotorSpeeds(300, 300);

		DualLightLocalizer dll = new DualLightLocalizer(Vehicle.LEFT_CS, Vehicle.RIGHT_CS);

		Tile tunnel = Tile.lowerRight(4, 4);

		double targetX = tunnel.getLowerLeft().getX();
		double targetY = tunnel.getLowerLeft().getY();

		Navigator.travelTo(Odometer.getX(), targetY, true, true, 300);
		dll.travelToLine(100);
		Navigator.travelSpecificDistance(5);

		try {
			Thread.sleep(1000);
		} catch (InterruptedException e) {
			e.printStackTrace();
		}

		Navigator.travelTo(targetX, Odometer.getY(), true, true, 300);
		dll.travelToLine(100); 

		try {
			Thread.sleep(1000);
		} catch (InterruptedException e1) {
			e1.printStackTrace();
		}

		Navigator.travelSpecificDistance(5);

		try {
			Thread.sleep(1000);
		} catch (InterruptedException e) {
			e.printStackTrace();
		}

		dll.localizeToSquare(Heading.N, Heading.E, Config.BACKWARD);
		testTunnelNavigation();
		dll.localizeToSquare(Heading.N, Heading.E, Config.FORWARD);
	}


	private void testFallingEdgeThenLocalize() throws InterruptedException {
		testUSLocalization();

		Claw claw = new Claw(usPoller);
		claw.grab();
		while (true) {
			Sound.beepSequenceUp();

			Vehicle.LEFT_MOTOR.setAcceleration(6000);
			Vehicle.RIGHT_MOTOR.setAcceleration(6000);
			DualLightLocalizer dll = new DualLightLocalizer(left, right);
			try {
				dll.localizeToIntersection(Board.Heading.N);
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
	private void testTimeWithCan() throws InterruptedException {
		ColourDetection cd = new ColourDetection(usPoller);	
		Claw claw = new Claw(usPoller);


		while (true) {
			while (Button.waitForAnyPress() != Button.ID_ESCAPE);
			cd.checkCanColour();
			claw.grab();
			testTime();
			claw.release();

		}		
	}

	private void testLocalizeToSquare() {

		DualLightLocalizer dll = new DualLightLocalizer(Vehicle.LEFT_CS, Vehicle.RIGHT_CS);
		try {
			dll.localizeToSquare(Board.Heading.N, Board.Heading.E, Config.FORWARD);
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

	private void testTunnelOrientation() {

		if (Board.getTunnelOrientation() == Board.TUNNEL_ORIENTATION.VERTICAL) {
			Sound.buzz();
		} else {
			Sound.beep();
		}

	}

	private void testShuffle() {
		while (true) {
			Navigator.turnTo(90);

			try {
				Thread.sleep(1000);
			} catch (InterruptedException e) {
				e.printStackTrace();
			}

			Navigator.turnTo(0);

			try {
				Thread.sleep(1000);
			} catch (InterruptedException e) {
				e.printStackTrace();
			}
		}

	}

	private void testDualLocalization() {

		Sound.beepSequenceUp();


		DualLightLocalizer dll = new DualLightLocalizer(Vehicle.LEFT_CS, Vehicle.RIGHT_CS);
		try {
			dll.localizeToIntersection(Board.Heading.N);

			Log.log(Sender.board, "Odometer X and Y : (" + Odometer.getX() + ", " + Odometer.getY() + ")");
			Log.log(Sender.board, "Odometer theta : " + Odometer.getTheta());

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
	 * @throws InterruptedException 
	 */
	private void testTime() throws InterruptedException {


		//what a shitshow
		//Plan:
		//Stop the pollers
		//close the regulated motors
		//open the unregulated motors
		//Apply power
		//Weigh the shit
		//Close the unregulated motors
		//Open the regulated motors
		//Start the pollers

		Weigh weigh = new Weigh(pollerSystem);
		weigh.weigh();
	}

	private boolean testCanSearch() throws InterruptedException {
		//Sound.beep();
		int filter = 0;
		int maxFilter = 12;
		int maxDistance = 30;
		int sweepSpeed = 100;

		ColourDetection cd = new ColourDetection(usPoller);

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

		//Cheat the beginning.
		Vehicle.setAcceleration(3000, 3000);
		//Cheat the beginning.
		Navigator.travelSpecificDistance(Board.TILE_SIZE*2,(int) Vehicle.RIGHT_MOTOR.getMaxSpeed()/2);

		Vehicle.setMotorSpeeds(300, 300);
		while (Vehicle.LEFT_CS.tunnelDetected());
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

		ColourDetection cd = new ColourDetection(usPoller);	
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
			while (!Vehicle.LEFT_CS.lineDetected() || !Vehicle.RIGHT_CS.lineDetected()) {
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
	 * This will test the colour detection. It will stay still, then if a can is in front of the robot,
	 * the robot will advance and detect the colour, then print it to the display. It will do this infinitely. 
	 */
	private void testColourDetection() {
		ColourDetection cd = new ColourDetection(usPoller);		
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

