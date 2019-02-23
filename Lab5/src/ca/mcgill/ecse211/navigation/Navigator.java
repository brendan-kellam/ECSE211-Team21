package ca.mcgill.ecse211.navigation;

import ca.mcgill.ecse211.hardware.Vehicle;
import ca.mcgill.ecse211.odometer.Odometer;
import ca.mcgill.ecse211.odometer.OdometerExceptions;
import ca.mcgill.ecse211.util.EV3Math;
import ca.mcgill.ecse211.util.Log;
import ca.mcgill.ecse211.util.Log.Sender;

/**
 * Facade for performing navigation tasks 
 */
public class Navigator {

    // Degree error
    final static double DEG_ERR = 1.2;
    
    // Travel error
    final static double CM_ERR = 2.0;
    
    // Default vehicle speed
    private static final int DEFAULT_SPEED = 100;

    
    /**
     * Travel to a specific location in space
     * 
     * @param x
     * @param y
     * @param stop
     * @param turn
     * @param speed
     * @throws OdometerExceptions
     */
    public static void travelTo(double x, double y, boolean stop, boolean turn, int speed) throws OdometerExceptions {
        
        // If turn is enabled
        if (turn) {
            double minAngle;
            minAngle = getDestAngle(x, y);
            turnTo(minAngle, true, speed);
        }
        
        // Set motor speeds
        Vehicle.setMotorSpeeds(speed, speed);
        while (!checkIfDone(x, y));
        
        // Stop at location
        if (stop) {
            Vehicle.setMotorSpeeds(0, 0);
        }
    }
     
    /**
     * TurnTo function which takes an angle and boolean as arguments. The boolean controls whether or not to stop the
     * motors when the turn is completed
     * @param targetAngle
     * @param stop
     */
    public static void turnTo(double targetAngle, boolean stop, int speed) {

        // Cute diff
        double diff = EV3Math.boundAngle(targetAngle - Odometer.getTheta());
        
        // While diff is greater than DEG_ERR
        while (diff > DEG_ERR) {
          
          Log.log(Sender.Navigator, "turnTo -> target angle = " + targetAngle + " | current angle = " + Odometer.getTheta() + " | diff = " + diff);

          // clock wise
          if (diff < 180) {
            Vehicle.setMotorSpeeds(speed, -speed);
          } else {
              Vehicle.setMotorSpeeds(-speed, speed);
          }
          
          diff = EV3Math.boundAngle(targetAngle - Odometer.getTheta());
        }

        if (stop) {
            Vehicle.setMotorSpeeds(0, 0);
        }
    }
    
    /**
     * Check if the vehicle's current position is within CM_ERR of a specified x, y position
     * 
     * @param x
     * @param y
     * @return
     */
    protected static boolean checkIfDone(double x, double y) {
      
      double curX = Odometer.getX(), curY = Odometer.getY();
      
      return Math.abs(x - curX) < CM_ERR
          && Math.abs(y - curY) < CM_ERR;
    }
    
    /**
     * Get minimum destination angle
     * 
     * @param x
     * @param y
     * @return
     * @throws OdometerExceptions 
     */
    public static double getDestAngle(double x, double y) throws OdometerExceptions {
      double[] position = Odometer.getOdometer().getXYT();
      double curX =     position[0];
      double curY =     position[1];
      
      double xdiff = x - curX;
      double ydiff = y - curY;
      
      double atan = Math.atan2(xdiff, ydiff);
      double atanDeg = Math.toDegrees(atan);
      double atanDegBound = EV3Math.boundAngle(atanDeg);
                  
      return atanDegBound;
    }
    
    /**
     * Optional turnTo. Uses DEFAULT_SPEED
     * 
     * @param targetAngle
     * @param stop
     */
    public static void turnTo(double targetAngle, boolean stop) {
        turnTo(targetAngle, stop, DEFAULT_SPEED);
    }
    
    /**
     * Optional travelTo. Uses DEFAULT_SPEED
     * 
     * @param x
     * @param y
     * @param stop
     * @param turn
     * @throws OdometerExceptions 
     */
    public static void travelTo(double x, double y, boolean stop, boolean turn) throws OdometerExceptions {
        travelTo(x, y, stop, turn, DEFAULT_SPEED);
    }
    
    
}
