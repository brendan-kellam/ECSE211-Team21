package ca.mcgill.ecse211.localization;


import ca.mcgill.ecse211.sensor.ColorSensor;
import ca.mcgill.ecse211.util.Vehicle;
import lejos.hardware.motor.BaseRegulatedMotor;

/**
 * Represents a concurrent routine that will move a given {@link lejos.hardware.motor.BaseRegulatedMotor BaseRegulatedMotor} 
 * forward until a given {@link ca.mcgill.ecse211.sensor.ColorSensor ColorSensor} detects a line.
 */
public class LineRunner implements Runnable {
    
    
    public static final LineRunner LEFT_RUNNER = new LineRunner(Vehicle.LEFT_CS, Vehicle.LEFT_MOTOR, 100.0f, 500);
    public static final LineRunner RIGHT_RUNNER = new LineRunner(Vehicle.RIGHT_CS, Vehicle.RIGHT_MOTOR, 100.0f, 500);
    
    
    // Amount of time to sleep during color sensor polling
    private static final int SLEEP_TIME = 30;
    
    private ColorSensor sensor;
    private BaseRegulatedMotor motor;
    private float speed;
    private long delay;
    
    /**
     * Construct a new LineRunner
     * 
     * @param sensor - color sensor to poll from
     * @param motor - motor to move until detection
     * @param speed
     * 
     * @throws IllegalArgumentException - 
     */
    public LineRunner(ColorSensor sensor, BaseRegulatedMotor motor, float speed, long delay) throws IllegalArgumentException {
        this.sensor = sensor;
        this.motor = motor;  
        this.delay = delay;
            
        if (speed == 0) {
            throw new IllegalArgumentException("A speed other than 0 must be specified.");
        }
        
        this.speed = speed;
    }
    
    /**
     * Start a new LineRunner thread
     */
    public Thread start(float speed) {
        this.speed = speed;
        Thread thread = new Thread(this);        
        thread.start();
        return thread;
    }
    
    /**
     * Concurrent run - Move motor forward until line detection
     */
    @Override
    public void run() {
        
        // Save the current speed
        int lastSpeed = motor.getSpeed();
        
        motor.setSpeed(speed);
        
        // Change rotation direction dependent on the speed
        if (speed > 0) {
            motor.forward();
        } else {
            motor.backward();
        }
        
        try {
            Thread.sleep(delay);
        } catch (InterruptedException e1) {
            e1.printStackTrace();
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
        
        // Reset to the last speed
        motor.setSpeed(lastSpeed);
    }
    
}