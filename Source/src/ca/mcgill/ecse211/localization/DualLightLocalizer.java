package ca.mcgill.ecse211.localization;

import ca.mcgill.ecse211.navigation.Navigator;
import ca.mcgill.ecse211.odometer.Odometer;
import ca.mcgill.ecse211.odometer.OdometerExceptions;
import ca.mcgill.ecse211.sensor.ColorSensor;
import ca.mcgill.ecse211.util.Board;
import ca.mcgill.ecse211.util.Vehicle;
import ca.mcgill.ecse211.util.Board.Heading;
import lejos.hardware.motor.BaseRegulatedMotor;

/**
 * Routine to localize to a arbitrary position
 */
public class DualLightLocalizer {

    
    /**
     * Left and right line runners
     */
    private LineRunner leftRunner;
    private LineRunner rightRunner;
    
    
    
    /**
     * Default constructor. Accepts two {@link ca.mcgill.ecse211.sensor.ColorSensor ColorSensor} objects representing the
     * left and right mounted color sensors.
     * 
     * @param leftSensor
     * @param rightSensor
     */
    public DualLightLocalizer(ColorSensor leftSensor, ColorSensor rightSensor) {
        this.leftRunner = new LineRunner(leftSensor, Vehicle.LEFT_MOTOR, 100);
        this.rightRunner = new LineRunner(rightSensor, Vehicle.RIGHT_MOTOR, 100);
    }
    
    /**
     * Localize to a grid intersection
     * @throws OdometerExceptions 
     */
    public void localize(Heading heading) throws OdometerExceptions {
        
        // Start by turning to the given heading
        Navigator.turnTo(Board.getHeadingAngle(heading));
        
        travelToLine();
        
        Board.snapToGridLine(Odometer.getOdometer());
        
        Navigator.travelSpecificDistance(-13, -100);
        
        Navigator.turnTo(Odometer.getTheta() + 90.0);
        
        travelToLine();
        
        Board.snapToGridLine(Odometer.getOdometer());
        
        Navigator.travelSpecificDistance(-13, -100);
        
        Navigator.turnTo(Board.getHeadingAngle(heading));
        
    }
    
    /**
     * Travel to a line, stopping when the two LineRunners
     */
    private void travelToLine() {
        
        
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
    
    /**
     * Represents a concurrent routine that will move a given {@link lejos.hardware.motor.BaseRegulatedMotor BaseRegulatedMotor} 
     * forward until a given {@link ca.mcgill.ecse211.sensor.ColorSensor ColorSensor} detects a line.
     */
    class LineRunner implements Runnable {
       
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
    
}
