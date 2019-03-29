package ca.mcgill.ecse211.localization;


import ca.mcgill.ecse211.sensor.ColorSensor;
import lejos.hardware.Sound;
import lejos.hardware.motor.BaseRegulatedMotor;

/**
 * Represents a concurrent routine that will move a given {@link lejos.hardware.motor.BaseRegulatedMotor BaseRegulatedMotor} 
 * forward until a given {@link ca.mcgill.ecse211.sensor.ColorSensor ColorSensor} detects a line.
 */
public class LineRunner implements Runnable {
       
    // Amount of time to sleep during color sensor polling
    private static final int SLEEP_TIME = 30;
    
    private ColorSensor sensor;
    private BaseRegulatedMotor motor;
    private float speed;
    
    /**
     * Construct a new LineRunner
     * 
     * @param sensor - color sensor to poll from
     * @param motor - motor to move until detection
     * @param speed
     * 
     * @throws IllegalArgumentException - 
     */
    public LineRunner(ColorSensor sensor, BaseRegulatedMotor motor, float speed) throws IllegalArgumentException {
        this.sensor = sensor;
        this.motor = motor;   
            
        if (speed == 0) {
            throw new IllegalArgumentException("A speed other than 0 must be specified.");
        }
        
        this.speed = speed;
    }

    /**
     * Concurrent run - Move motor forward until line detection
     */
    @Override
    public void run() {
        
        
        motor.setSpeed(speed);
        
        // Change rotation direction dependent on the speed
        if (speed > 0) {
            motor.forward();
        } else {
            motor.backward();
        }
        
        // Continuously check for line
        while (!sensor.lineDetected()) {
            try {
                Thread.sleep(SLEEP_TIME);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }
        
        motor.stop();
    }
    
}