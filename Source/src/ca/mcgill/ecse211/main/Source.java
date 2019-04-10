package ca.mcgill.ecse211.main;

import java.awt.geom.Point2D;
import java.io.FileNotFoundException;

import ca.mcgill.ecse211.Test;
import ca.mcgill.ecse211.claw.Claw;
import ca.mcgill.ecse211.claw.Weigh;
import ca.mcgill.ecse211.localization.DualLightLocalizer;
import ca.mcgill.ecse211.localization.FallingEdgeLocalizer;
import ca.mcgill.ecse211.localization.DualLightLocalizer.Config;
import ca.mcgill.ecse211.navigation.Navigator;
import ca.mcgill.ecse211.odometer.Odometer;
import ca.mcgill.ecse211.odometer.OdometerExceptions;
import ca.mcgill.ecse211.sensor.ColourDetection;
import ca.mcgill.ecse211.sensor.UltrasonicPoller;
import ca.mcgill.ecse211.util.Board;
import ca.mcgill.ecse211.util.Board.Heading;
import ca.mcgill.ecse211.util.Log;
import ca.mcgill.ecse211.util.Log.Sender;
import ca.mcgill.ecse211.util.Tile;
import ca.mcgill.ecse211.util.Vehicle;
import ca.mcgill.ecse211.util.WifiController;
import lejos.hardware.Button;
import lejos.hardware.Sound;
import lejos.hardware.lcd.LCD;

/**
 * Main entry point class for Lab4
 */

public final class Source {

	// Time to wait after display initialization (to allow graphics to apear on EV3's screen)
	private static final short DISPLAY_INIT_SLEEP_TIME = 2000;

	/**
	 * Represents a given MenuOption
	 */
	private enum MenuOption {
		TEST_COLOURS,
		NAVIGATE_MAP,
		INVALID
	}

	public static final boolean TESTING = false;



	/**
	 * Main entry of program
	 * @param args
	 * @throws OdometerExceptions 
	 * @throws InterruptedException 
	 */
	public static void main(String[] args) throws OdometerExceptions, InterruptedException {


		if (TESTING) {
			Test tester = new Test();
			tester.test();
			return;
		}

		// ----- Configuration ------

		// Create new vehicle configuration
		Vehicle.newConfig(new Vehicle.Configuration(2.1, 17.58));
		Vehicle.LEFT_MOTOR.setAcceleration(3000);
		Vehicle.RIGHT_MOTOR.setAcceleration(3000);

		// Create odometer
		Odometer odometer = Odometer.getOdometer();

		// Create ultrasonic poller
		UltrasonicPoller usPoller = new UltrasonicPoller(Vehicle.US_SENSOR);

		// Initialize logging
		Log.setLogging(false, false, false, false, false, false);

		// Set logging to write to file
		try {
			Log.setLogWriter("FinalProj" + ".log");
		} catch (FileNotFoundException e1) {
			e1.printStackTrace();
		}

		// Create claw
		Claw claw = new Claw(usPoller);

		PollerSystem pollerSystem = PollerSystem.getInstance();
		pollerSystem.addPoller(usPoller);
		pollerSystem.addPoller(odometer);
		pollerSystem.start();

		FallingEdgeLocalizer ul = new FallingEdgeLocalizer(odometer,usPoller);
		DualLightLocalizer dll = new DualLightLocalizer(Vehicle.LEFT_CS, Vehicle.RIGHT_CS);


		// ----- Configuration ------
		WifiController.fetchGameplayData();
		//Log.log(Sender.usSensor, "Tunnel LL: " + CompetitionConfig.tunnelEntranceToSearchArea.toString());
//		Log.log(Sender.board, CompetitionConfig.tostr());


		Search search = new Search(usPoller);
		// ----- Localize to grid ------
		localize(ul, dll);

		while (true) {
			claw.stow();

//			Log.log(Sender.odometer, "SET XY - X: " + Odometer.getX() + " | Y: " + Odometer.getY());



			// ------ TRAVEL TO TUNNEL -------
			travelToTunnel(dll);	  

			Heading toSearchArea = CompetitionConfig.toSearchAreaHeading;
			
    		// Traveling to search area
    		dll.localizeToTile(toSearchArea, Board.getOrthogonalHeading(toSearchArea, CompetitionConfig.tunnelEntranceToSearchArea), Board.getParallelHeading(toSearchArea));
    		Navigator.travelSpecificDistance(7);
    		travelThroughTunnel(CompetitionConfig.tunnelEntranceToStartArea);
    		dll.localizeToTile(toSearchArea, Board.getOrthogonalHeading(toSearchArea, CompetitionConfig.tunnelEntranceToStartArea), toSearchArea);
    		claw.release();


			Vehicle.setMotorSpeeds(0, 0);
			if (search.startSearch(new ColourDetection(usPoller))){
				claw.grab();
			}

			// NAVIGATE BACK TO START AREA

			Tile tunnelEntrance = CompetitionConfig.tunnelEntranceToStartArea;
			Navigator.travelTo(tunnelEntrance.getCenter().getX(), tunnelEntrance.getCenter().getY(), true, true);   

			Heading toStartArea = CompetitionConfig.toStartAreaHeading;


			dll.localizeToTile(toStartArea, Board.getOrthogonalHeading(toStartArea, CompetitionConfig.tunnelEntranceToStartArea), Board.getParallelHeading(toStartArea));

    		/*
    		 * TIME TO WEIGH THE CAN
    		 */
    		Heading currHeading = Board.getHeading(Odometer.getTheta());
    		double currX = Odometer.getX();
    		double currY = Odometer.getY();
    
    		Thread.sleep(30);
    		
    		switch (currHeading){
    		case N:
    		{
    			currY+=2.5*Board.TILE_SIZE;
    			break;
    		}
    		case E:
    		{
    			currX+=2.5*Board.TILE_SIZE;
    			break;
    			
    		}
    		case S:
    		{
    			currY-=2.5*Board.TILE_SIZE;
    			break;
    			
    		}
    		case W:
    		{
    			currX-=2.5*Board.TILE_SIZE;
    			break;
    		}
    		}
    		Navigator.travelSpecificDistance(5);
    		Weigh weigher = new Weigh(pollerSystem);
    		boolean heavy = weigher.weighThroughTunnel();
    		ColourDetection.canAssessment(heavy);
    		

			dll.localizeToTile(toStartArea, Board.getOrthogonalHeading(toStartArea), Board.getParallelHeading(toStartArea));

			odometer.setXYT(currX, currY, Board.getHeadingAngle(currHeading));
			Thread.sleep(30);

			dll.localizeToTile(toStartArea, Board.getOrthogonalHeading(toStartArea), toStartArea);
			Navigator.travelSpecificDistance(-5); //Back up a bit
			/*
			 * Weighing complete.
			 */
			Point2D start = Board.scTranslation[CompetitionConfig.corner]; // Get the coordinate of the start location.

			Navigator.travelTo(start.getX(), start.getY(), true, true);

			claw.release();
			
			for (int i=0;i<5;i++) {
				Sound.beep();
			}
			
		}
//		pollerSystem.stop();
//
//		// Wait
//		while (Button.waitForAnyPress() != Button.ID_ESCAPE);
//		System.exit(0);
	}

	/**
	 * Localize to play board
	 * 
	 * @param ul - falling edge localizer
	 * @param dl - dual light localizer
	 * @throws OdometerExceptions 
	 * @throws InterruptedException 
	 */
	private static void localize(FallingEdgeLocalizer ul, DualLightLocalizer dll) throws OdometerExceptions, InterruptedException {
		ul.usLocalize();
		dll.localizeToIntersection(Heading.N, 300);

		Point2D trans = Board.scTranslation[CompetitionConfig.corner];
		double rot = Board.scRotation[CompetitionConfig.corner];

		Odometer.getOdometer().setX(trans.getX());
		Odometer.getOdometer().setY(trans.getY());
		Odometer.getOdometer().setTheta(rot);

		// Compliance beeps
		for (int i = 0; i < 3; i++) {
			Sound.beep();
		}

//		dll.travelToLine(100);
	}

	public static void travelToTunnel(DualLightLocalizer dll) throws OdometerExceptions {

		Tile tunnelEntrance = CompetitionConfig.tunnelEntranceToSearchArea;
		Navigator.travelTo(tunnelEntrance.getCenter().getX(), tunnelEntrance.getCenter().getY(), true, true);   
	}


	/**
	 * Routine to travel to the tunnel
	 * 
	 * @param dll
	 * @throws OdometerExceptions
	 */
	/*
	private static void travelToTunnel(DualLightLocalizer dll) throws OdometerExceptions {

	    // Get tunnel lower left and right
	    Tile toSearchArea = CompetitionConfig.tunnelEntranceToSearchArea;
        Tile tunnelUR = CompetitionConfig.tunnelEntranceToStartArea;

        // Debug information
        Log.log(Sender.Navigator, "tunnelLR: " + toSearchArea.toString());
        Log.log(Sender.Navigator, "tunnelUR: " + tunnelUR.toString());

        double targetX = toSearchArea.getLowerLeft().getX();
        double targetY = toSearchArea.getLowerLeft().getY();

        Navigator.travelTo(Odometer.getX(), targetY, true, true);
        dll.travelToLine(100);

        Navigator.travelSpecificDistance(5);

        Navigator.turnTo(Navigator.getDestAngle(targetX, toSearchArea.getCenter().getY()));
        dll.travelToLine(100);

        Navigator.travelTo(targetX, toSearchArea.getCenter().getY(), true, true);
        dll.travelToLine(100); 
        Navigator.travelSpecificDistance(5);

	}
	 */

	/**
	 * 
	 * @throws OdometerExceptions
	 */
	private static void travelThroughTunnel(Tile target) throws OdometerExceptions {

		//Cheat the beginning.
		Navigator.travelSpecificDistance(Board.TILE_SIZE*2.5,(int) Vehicle.RIGHT_MOTOR.getMaxSpeed()/2);

		Vehicle.setMotorSpeeds(300, 300);
		//		while (Vehicle.LEFT_CS.tunnelDetected() || !target.contains(Odometer.getX(), Odometer.getY())) {
		while (Vehicle.LEFT_CS.tunnelDetected()) {
			try {
				Thread.sleep(50);
			} catch (InterruptedException e) {
				e.printStackTrace();
			}
		}
		Navigator.travelSpecificDistance(8);
		Vehicle.setMotorSpeeds(0, 0);
	}

	/**
	 * Gets the User's menu choice of either RISING or FALLING edge
	 * 
	 * @return MenuOption
	 */
	public static MenuOption getUserChoice() {
		Vehicle.LCD_DISPLAY.drawString("Test Colours: UP", 0, 0);
		Vehicle.LCD_DISPLAY.drawString("Navigate map: DOWN", 0, 1);

		int choice = Button.waitForAnyPress();

		if (choice == Button.ID_UP) {
			return MenuOption.TEST_COLOURS;
		}
		else if (choice == Button.ID_DOWN) {
			return MenuOption.NAVIGATE_MAP;
		}

		return MenuOption.INVALID;
	}

}