package ca.mcgill.ecse211.localization;

import ca.mcgill.ecse211.navigation.Navigator;
import ca.mcgill.ecse211.odometer.Odometer;
import ca.mcgill.ecse211.odometer.OdometerExceptions;
import ca.mcgill.ecse211.sensor.ColorSensor;
import ca.mcgill.ecse211.util.Board;
import ca.mcgill.ecse211.util.Vehicle;
import ca.mcgill.ecse211.util.Board.Heading;
import ca.mcgill.ecse211.util.Tile;
import lejos.hardware.Button;
import lejos.hardware.Sound;
import lejos.hardware.motor.BaseRegulatedMotor;

/**
 * Routine to localize to a arbitrary position
 */
public class DualLightLocalizer {

    
    /**
     * Left and right color sensors
     */
    private ColorSensor leftSensor;
    private ColorSensor rightSensor;
    
    /**
     * Speed of dual localization
     */
    private final int SPEED = 200;
    
    
    /**
     * Distance the vehicle must travel backwards from a line to get to the center of a tile
     */
    private final double TRAVEL_DIST_FOR_LIGHT_SENSORS = Board.TILE_SIZE - 2;
    
    
    
    /**
     * Default constructor. Accepts two {@link ca.mcgill.ecse211.sensor.ColorSensor ColorSensor} objects representing the
     * left and right mounted color sensors.
     * 
     * @param leftSensor
     * @param rightSensor
     */
    public DualLightLocalizer(ColorSensor leftSensor, ColorSensor rightSensor) {
        this.leftSensor = leftSensor;
        this.rightSensor = rightSensor;
    }
    
    /**
     * Localize to a grid intersection
     * @throws OdometerExceptions 
     */
    public void localizeToIntersection(Heading heading) throws OdometerExceptions {
        
        // Start by turning to the given heading
        Navigator.turnTo(Board.getHeadingAngle(heading));
        
        travelToLine(SPEED);
        
        Navigator.travelSpecificDistance(-Vehicle.VERT_DIST_FROM_LIGHT_SENSORS_TO_WHEEL_BASE, -SPEED);
                
        Navigator.turnTo(90.0);
        
        travelToLine(SPEED);
        
        Navigator.travelSpecificDistance(-Vehicle.VERT_DIST_FROM_LIGHT_SENSORS_TO_WHEEL_BASE, -SPEED);
        Navigator.turnTo(4.0);
        
        // Magic
        Navigator.travelSpecificDistance(2.0);
        
        
    }
    
    public enum Config {
        FORWARD,
        BACKWARD
    }
    
    
    public boolean localizeToSquare(Heading heading, Heading finalHeading, Config config) throws OdometerExceptions {
        
        // Start by turning to the given heading
        Navigator.turnTo(Board.getHeadingAngle(heading));
        
        Navigator.travelSpecificDistance(10);
        travelToLine(SPEED);

        
        double diff = Board.TILE_SIZE - 4;
        Navigator.travelSpecificDistance(-diff, -SPEED);
        
        
        try {
            Thread.sleep(500);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
        
        Navigator.turnTo(90.0);
        
        if (config == Config.FORWARD) {
            Navigator.travelSpecificDistance(-10);
            travelToLine(SPEED);
        } else {
            Navigator.travelSpecificDistance(10);
            travelToLine(-SPEED);
        }
        
        return Board.getHeading(Odometer.getTheta()) == finalHeading;
        
    }
    
    /**
     * Localizes the vehicle to a arbitrary tile given two headings
     * 
     * @param l1 - Line 1
     * @param l2 - Line 2
     * 
     * @throws IllegalArgumentException If heading l1 and l2 are the same or they are parallel
     */
    public void localizeToTile(Heading fin, Heading l1, Heading l2) throws IllegalArgumentException {
        
        if (l1 == l2 || Board.areHeadingsParallel(l1, l2) ) {
            throw new IllegalArgumentException("Invalid line spec - headings must be orthogonal");
        }
        
        if (!Board.areHeadingsOrthogonal(fin, l1)) {
            throw new IllegalArgumentException("Invalid line spec - fin and l1 must be orthogonal");
        }
        
        if (!Board.areHeadingsParallel(fin, l2)) {
            throw new IllegalArgumentException("Invalid line spec - fin and l2 must be parallel");
        }
        
        localizeOnLine(fin, l1, 2000);
        localizeOnLine(fin, l2, 2000);
        
        // Turn to final angle
        Navigator.turnTo(Board.getHeadingAngle(fin));
    }
    
    /**
     * Localizes on  given line l, with a final heading fin
     * 
     * @param fin
     * @param l
     */
    private void localizeOnLine(Heading fin, Heading l, int lineCrossOverSleepTime) {
        // Start by turning to the final heading
        Navigator.turnTo(Board.getHeadingAngle(fin));
        
        int sign = getSpeedSign(fin, l);
        int speed = SPEED * sign;
        
        // If 1st and final headings are orthogonal, the vehicle must turn to the 1st
        if (Board.areHeadingsOrthogonal(fin, l)) {
            Navigator.turnTo(Board.getHeadingAngle(l));
        }
        
        Vehicle.setMotorSpeeds(100, 100);
        try {
            Thread.sleep(lineCrossOverSleepTime);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
        Vehicle.setMotorSpeeds(0, 0);

        // Travel to the 1st line
        travelToLine(speed);
        
        // Return back the center of tile
        if (Board.areHeadingsOrthogonal(fin, l)) {
            double diff = TRAVEL_DIST_FOR_LIGHT_SENSORS * -sign;
            Navigator.travelSpecificDistance(diff, -speed);
        }
    }
    
    /**
     * 
     * @param h1
     * @param h2
     * @return
     */
    private int getSpeedSign(Heading h1, Heading h2) {
        if (Board.areHeadingsOrthogonal(h1, h2) || h1 == h2) {
            return 1;
        }
        
        return -1;
    }
    
    /**
     * Travel to a line, stopping when the two LineRunners
     */
    public void travelToLine(float speed) {
        
        Thread lThread = LineRunner.LEFT_RUNNER.start(speed);
        Thread rThread = LineRunner.RIGHT_RUNNER.start(speed);
        
        // Wait on these threads to complete
        try {
            lThread.join();
            rThread.join();
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
        
        try {
            Board.snapToHeading(Odometer.getOdometer());
            Board.snapToGridLine(Odometer.getOdometer(), true);
        } catch (OdometerExceptions e) {
            e.printStackTrace();
        }
        
    }
    
    
    public void travelToLineWithAdaptiveChecking(Tile container, float speed) {
        
        Thread lThread = LineRunner.LEFT_RUNNER.start(speed);
        Thread rThread = LineRunner.RIGHT_RUNNER.start(speed);
        
        
        
        
    }
    
    
    
}
