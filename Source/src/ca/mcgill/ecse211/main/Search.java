package ca.mcgill.ecse211.main;

import ca.mcgill.ecse211.navigation.Navigator;
import ca.mcgill.ecse211.odometer.Odometer;
import ca.mcgill.ecse211.odometer.OdometerExceptions;
import ca.mcgill.ecse211.sensor.ColourDetection;
import ca.mcgill.ecse211.sensor.UltrasonicPoller;
import ca.mcgill.ecse211.util.Board;
import ca.mcgill.ecse211.util.EV3Math;
import ca.mcgill.ecse211.util.Tile;
import ca.mcgill.ecse211.util.Vehicle;
import lejos.hardware.Sound;
import lejos.hardware.lcd.LCD;

public class Search {


	private UltrasonicPoller usPoller;
	private double targetX,targetY;
	private Board.Heading toSearchArea;
	private final int ROTATION_SPEED = 200;
	private final int MAX_FILTER = 12;

	public Search(UltrasonicPoller usPoller) {

		this.usPoller = usPoller;
		toSearchArea = CompetitionConfig.toSearchAreaHeading;
		Tile bottomLeft = CompetitionConfig.searchAreaLL;
		Tile topRight = CompetitionConfig.searchAreaUR;

		switch (toSearchArea){
		case N:
		{
			targetX = computeMiddle(bottomLeft.getMinX(),topRight.getMaxX());
			targetY = bottomLeft.getMinY();
			break;
		}
		case S:
		{
			targetX = computeMiddle(bottomLeft.getMinX(),topRight.getMaxX());
			targetY = topRight.getMaxY();
			break;
		}
		case W:
		{
			targetX = topRight.getMaxX();
			targetY = computeMiddle(bottomLeft.getMinY(),topRight.getMaxY());
			break;
		}
		case E:
		{
			targetX = bottomLeft.getMinX();
			targetY = computeMiddle(bottomLeft.getMinY(),topRight.getMaxY());
			break;
		}

		}

	}

	/**
	 * Search for a can by going to the middle
	 * @param cd
	 * @return
	 * @throws OdometerExceptions
	 * @throws InterruptedException
	 */
	public boolean startSearch(ColourDetection cd) throws OdometerExceptions, InterruptedException {
		Navigator.travelTo(targetX, targetY, true, true);
		for (int i=0;i<3;i++) {
			Sound.beep();
		}

		Navigator.turnTo(Board.getHeadingAngle(toSearchArea));
		Navigator.travelSpecificDistance(-5);
		for (int i=0;i<3;i++) {

			double heldAngle = Odometer.getTheta();
			Thread.sleep(50);
			double angles[] = new double[2];

			angles[0] = EV3Math.boundAngle(heldAngle-90);
			angles[1] = EV3Math.boundAngle(heldAngle+90);

			Navigator.turnTo(angles[0],ROTATION_SPEED,true);
			Thread.sleep(150);
			if (scanForCan(angles[0],cd)) {
				return true;
			}
			Navigator.turnTo(heldAngle);

			Navigator.turnTo(angles[1],ROTATION_SPEED,true);
			Thread.sleep(150);
			if (scanForCan(angles[1],cd)) {
				return true;
			}
			Navigator.turnTo(heldAngle);
			Navigator.travelSpecificDistance(Board.TILE_SIZE);
			Navigator.turnTo(Board.getHeadingAngle(toSearchArea));
		}

		return false;
	}

	/**
	 * Scan for a can near a given location.
	 * @param targetLocation
	 * @param cd
	 * @return
	 * @throws InterruptedException
	 */
	private boolean scanForCan(double targetLocation, ColourDetection cd) throws InterruptedException {
		int filter = 0;
		int maxDistance = 35;

		while (filter < MAX_FILTER && Math.abs(Odometer.getTheta() - targetLocation) > 5)  {
			Thread.sleep(30);
			if (	usPoller.getDistance() < maxDistance) {
				filter++;
			}
		}

		//Can detected:
		if (filter >= MAX_FILTER) {
			Vehicle.setMotorSpeeds(0, 0);
			Thread.sleep(50);
			if (cd.checkCanColour()) {
				return true;
			}
		}
		//		Sound.twoBeeps();
		return false;
	}

	/**
	 * Compute the middle given two coordinates
	 * @param coordOne
	 * @param coordTwo
	 * @return
	 */
	private double computeMiddle(double coordOne, double coordTwo){
		return (coordOne+coordTwo)/2;
	}
}
