/**
 * This class is meant as a skeleton for the odometer class to be used.
 * 
 * @author Rodrigo Silva
 * @author Dirk Dubois
 * @author Derek Yu
 * @author Karim El-Baba
 * @author Michael Smith
 */

package ca.mcgill.ecse211.odometer;

import ca.mcgill.ecse211.hardware.Vehicle;
import ca.mcgill.ecse211.util.Poller;

public class Odometer extends OdometerData implements Poller {

  private OdometerData odoData;
  private static Odometer odo = null; // Returned as singleton

  private int lastTachoL;
  private int lastTachoR;
  private int nowTachoL;
  private int nowTachoR;

  private static final long ODOMETER_PERIOD = 25; // odometer update period in ms
  private static final double PI = 3.14159;

  /**
   * This is the default constructor of this class. It initiates all motors and variables once.It
   * cannot be accessed externally.
   * 
   * @param leftMotor
   * @param rightMotor
   * @param TRACK
   * @param WHEEL_RAD
   * @throws OdometerExceptions
   */
  
  private Odometer() throws OdometerExceptions {
    
	  
	odoData = OdometerData.getOdometerData(0, 0); // Allows access to x,y,z
                                              // manipulation methods

    // Reset the values of x, y and z to 0
    odoData.setXYT(0, 0, 0);

    // Clear tachometer counts
    Vehicle.LEFT_MOTOR.resetTachoCount();
    Vehicle.RIGHT_MOTOR.resetTachoCount();
    lastTachoL = Vehicle.LEFT_MOTOR.getTachoCount();
    lastTachoR = Vehicle.RIGHT_MOTOR.getTachoCount();
  }

  /**
   * This method is meant to ensure only one instance of the odometer is used throughout the code.
   * s
   */
  public synchronized static Odometer getOdometer()
      throws OdometerExceptions {
    if (odo != null) { // Return existing object
      return odo;
    } else { // create object and return it
      odo = new Odometer();
      return odo;
    }
  }

  /**
   * This method is where the logic for the odometer will run. Use the methods provided from the
   * OdometerData class to implement the odometer.
   */
    @Override
    public void update() {

        // Get tachometer count from left and right encoders
        nowTachoL = Vehicle.LEFT_MOTOR.getTachoCount();
        nowTachoR = Vehicle.RIGHT_MOTOR.getTachoCount();

        double dx, dy;
        double deltaD;
        double deltaT;
        
        double distL = (PI / 180) * (nowTachoL - lastTachoL) * Vehicle.getConfig().getWheelRadius(); // Displacement of left wheel (Lamda * rL)
        double distR = (PI / 180) * (nowTachoR - lastTachoR) * Vehicle.getConfig().getWheelRadius(); // Displacement of right wheel (Phi * rR)
        
        // Update tachometer counts
        lastTachoL = nowTachoL;
        lastTachoR = nowTachoR;
        
        // Total vehicle displacement
        deltaD = 0.5 * (distL + distR);
        
        // Change in heading
        deltaT = (distL - distR) / Vehicle.getConfig().getTrackWidth();
        
        double curTheta = Math.toRadians(odo.getXYT()[2]);
        double phi = curTheta + deltaT*0.5;
        
        // Compute change in x & y
        dx = deltaD * Math.sin(phi);
        dy = deltaD * Math.cos(phi);
        
        // Update odometer values
        odo.update(dx, dy, Math.toDegrees(deltaT));
    }

}
