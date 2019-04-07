package ca.mcgill.ecse211.localization;

import ca.mcgill.ecse211.navigation.Navigator;
import ca.mcgill.ecse211.odometer.Odometer;
import ca.mcgill.ecse211.odometer.OdometerExceptions;
import ca.mcgill.ecse211.sensor.ColorSensor;
import ca.mcgill.ecse211.util.Board;
import ca.mcgill.ecse211.util.Vehicle;
import ca.mcgill.ecse211.util.Board.Heading;
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
        
        Navigator.travelSpecificDistance(-Vehicle.DISTANCE_FROM_LIGHT_SENSORS_TO_WHEEL_BASE, -SPEED);
                
        Navigator.turnTo(90.0);
        
        travelToLine(SPEED);
        
        Navigator.travelSpecificDistance(-Vehicle.DISTANCE_FROM_LIGHT_SENSORS_TO_WHEEL_BASE, -SPEED);
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
//        
//        Board.snapToHeading(Odometer.getOdometer());
//        Board.snapToGridLine(Odometer.getOdometer(), true);
        
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
        
//        Board.snapToGridLine(Odometer.getOdometer(), true);
//        Board.snapToHeading(Odometer.getOdometer());
        
        return Board.getHeading(Odometer.getTheta()) == finalHeading;
        
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
    
    
    
}
