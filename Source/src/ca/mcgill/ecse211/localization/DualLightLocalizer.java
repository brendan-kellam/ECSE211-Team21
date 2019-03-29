package ca.mcgill.ecse211.localization;

import ca.mcgill.ecse211.navigation.Navigator;
import ca.mcgill.ecse211.odometer.Odometer;
import ca.mcgill.ecse211.odometer.OdometerExceptions;
import ca.mcgill.ecse211.sensor.ColorSensor;
import ca.mcgill.ecse211.util.Board;
import ca.mcgill.ecse211.util.Vehicle;
import ca.mcgill.ecse211.util.Board.Heading;
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
        
        Navigator.travelSpecificDistance(-13, -SPEED);
        Board.snapToHeading(Odometer.getOdometer());
        Board.snapToGridLine(Odometer.getOdometer());
        
        Navigator.turnTo(Odometer.getTheta() + 90.0);
        
        travelToLine(SPEED);
        
        Navigator.travelSpecificDistance(-13, -SPEED);
        Board.snapToGridLine(Odometer.getOdometer());
        Board.snapToHeading(Odometer.getOdometer());
                
    }
    
    public boolean localizeToSquare(Heading heading, Heading finalHeading) throws OdometerExceptions {
        // Start by turning to the given heading
        Navigator.turnTo(Board.getHeadingAngle(heading));
        
        travelToLine(SPEED);
        
        double diff = Board.TILE_SIZE - 4;
        
        Navigator.travelSpecificDistance(-diff, -SPEED);
        
        Board.snapToHeading(Odometer.getOdometer());
        Board.snapToGridLine(Odometer.getOdometer());
        
        try {
            Thread.sleep(500);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
        
        Navigator.turnTo(90.0);
        
        travelToLine(-SPEED);
        
        Board.snapToGridLine(Odometer.getOdometer());
        Board.snapToHeading(Odometer.getOdometer());
        
        return Board.getHeading(Odometer.getTheta()) == finalHeading;
        
    }
    
    /**
     * Travel to a line, stopping when the two LineRunners
     */
    private void travelToLine(float speed) {
        
        LineRunner leftRunner = new LineRunner(leftSensor, Vehicle.LEFT_MOTOR, speed);
        LineRunner rightRunner = new LineRunner(rightSensor, Vehicle.RIGHT_MOTOR, speed);
        
        Thread lThread = new Thread(leftRunner);
        Thread rThread = new Thread(rightRunner);
        
        lThread.start();
        rThread.start();
        
        // Wait on these threads to complete
        try {
            lThread.join();
            rThread.join();
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
        
    }
    
    
    
}
