package ca.mcgill.ecse211.localization;

import ca.mcgill.ecse211.hardware.Vehicle;
import ca.mcgill.ecse211.light.ColorSensor;
import ca.mcgill.ecse211.navigation.Navigator;
import ca.mcgill.ecse211.util.Board;
import ca.mcgill.ecse211.util.Board.Heading;

/**
 * Blocking routine to 
 */
public class DualLightLocalizer {

    
    /**
     * Left and right color sensors
     */
    private ColorSensor leftSensor;
    private ColorSensor rightSensor;
    
    /**
     * Default constructor. Accepts two {@link ca.mcgill.ecse211.light.ColorSensor ColorSensor} objects representing the
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
     * Localize to a line
     */
    public void localize(Heading heading) {
        
        // Start by turning to the given heading
        Navigator.turnTo(Board.getHeadingAngle(heading));
        
        // Begin by advancing forward until either the left or right sensor detects a line
        Vehicle.setMotorSpeeds(50, 50);
        while (!leftSensor.lineDetected() || !rightSensor.lineDetected());
        
        // If both sensors detected a line
        if (leftSensor.lineDetected() && rightSensor.lineDetected()) {
            Vehicle.setMotorSpeeds(0, 0);
        }
        
        // Left detected
        else if (leftSensor.lineDetected()) {
            Vehicle.LEFT_MOTOR.setSpeed(0);
            while (!rightSensor.lineDetected());
            Vehicle.RIGHT_MOTOR.setSpeed(0);
        }
        
        // right detected
        else {
            Vehicle.RIGHT_MOTOR.setSpeed(0);
            while (!leftSensor.lineDetected());
            Vehicle.LEFT_MOTOR.setSpeed(0);
        }
        
        
    }
    
}
