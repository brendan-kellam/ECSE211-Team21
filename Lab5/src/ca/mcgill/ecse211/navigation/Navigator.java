package ca.mcgill.ecse211.navigation;

import ca.mcgill.ecse211.hardware.Vehicle;
import ca.mcgill.ecse211.odometer.Odometer;
import ca.mcgill.ecse211.odometer.OdometerExceptions;
import ca.mcgill.ecse211.util.EV3Math;
import ca.mcgill.ecse211.util.Log;
import ca.mcgill.ecse211.util.Log.Sender;
import lejos.hardware.Sound;

/**
 * Facade for performing navigation tasks 
 */
public class Navigator {

    // Degree error
    final static double DEG_ERR = 1.2;
    
    // Travel error
    final static double CM_ERR = 3.5;
    
    // Default vehicle speed
    private static final int DEFAULT_SPEED = 100;

    private static final double THETA_THRESHOLD = 8;
    public static final int ROTATE_SPEED    = 150;

    
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
        
        Log.log(Log.Sender.Navigator, "TargetX: " + x + " | TargetY: " + y + " | curX: " + Odometer.getX() + " | curY: " + Odometer.getY());
        
        // If turn is enabled
        if (turn) {
            double minAngle;
            minAngle = getDestAngle(x, y);
            turnTo(minAngle, speed, false);
        }
        
        int n = 100;
        int count = 0;
        
        
        double distX = Math.abs(x - Odometer.getX());
        double distY = Math.abs(y - Odometer.getY());
        double dist = Math.hypot(distX, distY);
        
        Vehicle.setMotorSpeeds(speed, speed);
        Vehicle.LEFT_MOTOR.rotate(EV3Math.convertDistance(Vehicle.getConfig().getWheelRadius(), dist), true);
        Vehicle.RIGHT_MOTOR.rotate(EV3Math.convertDistance(Vehicle.getConfig().getWheelRadius(), dist), false);

        // Set motor speeds
        //Vehicle.setMotorSpeeds(speed, speed);
        //while (!checkIfDone(x, y));
        
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

          double goSpeed;
          
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
    
    public static void turnTo(double theta, int speed, boolean immediateReturn) {
        
        double currentHeading = 0.0;
        try {
            Odometer odo = Odometer.getOdometer();
            
            synchronized(odo) {
                currentHeading = odo.getXYT()[2];
            }
            
        } catch (OdometerExceptions e) {
            e.printStackTrace();
        }
        
        Log.log(Log.Sender.Navigator, "Current Heading: " + currentHeading);
        Log.log(Log.Sender.Navigator, "Target Heading: " + theta);
        
        
        double distance = distance(theta, currentHeading);
        
        Log.log(Log.Sender.Navigator, "Distance: " + distance);
        
        if(distance > THETA_THRESHOLD) {
            
            int cw = findTurnSide(theta, currentHeading);
            
            Vehicle.LEFT_MOTOR.setSpeed(ROTATE_SPEED);
            Vehicle.RIGHT_MOTOR.setSpeed(ROTATE_SPEED);
            
            double leftRotation = EV3Math.convertAngle(Vehicle.getConfig().getWheelRadius(),Vehicle.getConfig().getTrackWidth(),distance)*-cw;
            double rightRotation = EV3Math.convertAngle(Vehicle.getConfig().getWheelRadius(),Vehicle.getConfig().getTrackWidth(), distance)*cw;
            
            Vehicle.LEFT_MOTOR.rotate((int) leftRotation, true);
            Vehicle.RIGHT_MOTOR.rotate((int) rightRotation, immediateReturn);
        }
    }
    
    public static double distance(double theta, double d) {
        double phi = Math.abs(d - theta) % 360;       // This is either the distance or 360 - distance
        double distance = phi > 180.0 ? 360.0 - phi : phi;
        return distance;
    }
    
    public static int findTurnSide(double d, double e)
    {
         double diff = e - d;
         if(diff < 0)
             diff += 360;
         if(diff > 180)
              return -1; // left turn
         else
              return 1; // right turn
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
