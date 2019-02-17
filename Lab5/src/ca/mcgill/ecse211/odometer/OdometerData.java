package ca.mcgill.ecse211.odometer;

import java.util.concurrent.locks.Condition;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;

/**
 * This class stores and provides thread safe access to the odometer data.
 * 
 * @author Rodrigo Silva
 * @author Dirk Dubois
 * @author Derek Yu
 * @author Karim El-Baba
 * @author Michael Smith
 */

public class OdometerData {

  // Position parameters	
  private static volatile double x; // x-axis position
  private static volatile double y; // y-axis position
  private static volatile double theta; // Head angle
  
  private static volatile double prevX, prevY, prevT;
  
  
  // x and y offsets
  public static volatile double xoffset;
  public static volatile double yoffset;

  // Class control variables
  private volatile static int numberOfIntances = 0; // Number of OdometerData
                                                    // objects instantiated
                                                    // so far
  
  private static final int MAX_INSTANCES = 1; // Maximum number of
                                              // OdometerData instances

  // Thread control tools
  private static Lock lock = new ReentrantLock(true); // Fair lock for
                                                      // concurrent writing
  
  private volatile boolean isReseting = false; // Indicates if a thread is
                                               // trying to reset any
                                               // position parameters
  
  private Condition doneReseting = lock.newCondition(); // Let other threads
                                                        // know that a reset
                                                        // operation is
                                                        // over.

  private volatile static OdometerData odoData = null;
  
  /**
   * Default constructor
   */
  protected OdometerData() {
  }
  
  /**
   * Default constructor. The constructor is private. A factory is used instead such that only one
   * instance of this class is ever created.
   * 
   * Initialize x, y to offset values.
   * Initialize theta to 0
   */
  protected OdometerData(double xoffset, double yoffset) {
      OdometerData.prevX = OdometerData.x = OdometerData.xoffset = xoffset;
      OdometerData.prevY = OdometerData.y = OdometerData.yoffset = yoffset;
      OdometerData.prevT = OdometerData.theta = 0;
  }
  
  
  
  public synchronized static OdometerData getOdometerData(double xoffset, double yoffset) throws OdometerExceptions {
	  if (odoData != null) { // Return existing object
	      return odoData;
	    } else if (numberOfIntances < MAX_INSTANCES) { // create object and
	                                                   // return it
	      odoData = new OdometerData(xoffset, yoffset);
	      numberOfIntances += 1;
	      return odoData;
	    } else {
	      throw new OdometerExceptions("Only one intance of the Odometer can be created.");
	    }
  }

  /**
   * OdometerData factory. Returns an OdometerData instance and makes sure that only one instance is
   * ever created. If the user tries to instantiate multiple objects, the method throws a
   * MultipleOdometerDataException.
   * 
   * @return An OdometerData object
   * @throws OdometerExceptions
   */
  public static OdometerData getOdometerData() throws OdometerExceptions {
    return OdometerData.getOdometerData(0, 0);
  }

  /**
   * Return the Odomometer data.
   * <p>
   * Writes the current position and orientation of the robot onto the odoData array. odoData[0] =
   * x, odoData[1] = y; odoData[2] = theta;
   * 
   * @param position the array to store the odometer data
   * @return the odometer data.
   */
  public synchronized double[] getXYT() {
    double[] position = new double[3];
    lock.lock();
    try {
      while (isReseting) { // If a reset operation is being executed, wait
        // until it is over.
        doneReseting.await(); // Using await() is lighter on the CPU
        // than simple busy wait.
      }

      position[0] = x;
      position[1] = y;
      position[2] = theta;

    } catch (InterruptedException e) {
      // Print exception to screen
      e.printStackTrace();
    } finally {
      lock.unlock();
    }

    return position;

  }
  
  /**
   * Adds dx, dy and dtheta to the current values of x, y and theta, respectively. Useful for
   * odometry.
   * 
   * @param dx
   * @param dy
   * @param dtheta
   */
  public synchronized void update(double dx, double dy, double dtheta) {
    lock.lock();
    isReseting = true;
    try {
      
      // Set previous before updating
      prevX = x;
      prevY = y;
      prevT = theta;
      
      x += dx;
      y += dy;
      theta = (theta + (360 + dtheta) % 360) % 360; // keeps the updates
                                                    // within 360
                                                    // degrees
      isReseting = false; // Done reseting
      doneReseting.signalAll(); // Let the other threads know that you are
                                // done reseting
    } finally {
      lock.unlock();
    }

  }

  /**
   * Overrides the values of x, y and theta. Use for odometry correction.
   * 
   * @param x the value of x
   * @param y the value of y
   * @param theta the value of theta
   */
  public synchronized void setXYT(double x, double y, double theta) {
    lock.lock();
    isReseting = true;
    try {
      
      prevX = OdometerData.x;
      prevY = OdometerData.y;
      prevT = OdometerData.theta;
      
      OdometerData.x = x + xoffset;
      OdometerData.y = y + yoffset;
      OdometerData.theta = theta;
      isReseting = false; // Done reseting
      doneReseting.signalAll(); // Let the other threads know that you are
                                // done reseting
    } finally {
      lock.unlock();
    }
  }

  /**
   * Overrides x. Use for odometry correction.
   * 
   * @param x the value of x
   */
  public synchronized void setX(double x) {
    lock.lock();
    isReseting = true;
    try {
      prevX = OdometerData.x;
      OdometerData.x = x + xoffset;
      isReseting = false; // Done reseting
      doneReseting.signalAll(); // Let the other threads know that you are
                                // done reseting
    } finally {
      lock.unlock();
    }
  }

  /**
   * Overrides y. Use for odometry correction.
   * 
   * @param y the value of y
   */
  public synchronized void setY(double y) {
    lock.lock();
    isReseting = true;
    try {
      prevY = OdometerData.y;
      OdometerData.y = y + yoffset;
      isReseting = false; // Done reseting
      doneReseting.signalAll(); // Let the other threads know that you are
                                // done reseting
    } finally {
      lock.unlock();
    }
  }

  /**
   * Overrides theta. Use for odometry correction.
   * 
   * @param theta the value of theta
   */
  public synchronized void setTheta(double theta) {
    lock.lock();
    isReseting = true;
    try {
      prevT = OdometerData.theta;
      OdometerData.theta = theta;
      isReseting = false; // Done reseting
      doneReseting.signalAll(); // Let the other threads know that you are
                                // done reseting
    } finally {
      lock.unlock();
    }
  }
  
  /**
   * Get x
   * 
   * @return x
   */
  public synchronized static double getX() {
    return x;
  }

  /**
   * Get y
   * 
   * @return y
   */
  public synchronized static double getY() {
    return y;
  }

  /**
   * Get theta
   * 
   * @return theta
   */
  public synchronized static double getTheta() {
    return theta;
  }

  /**
   * Get previous x
   * 
   * @return prevX
   */
  public synchronized static double getPrevX() {
    return prevX;
  }

  /**
   * Get previous y
   * 
   * @return prevY
   */
  public synchronized static double getPrevY() {
    return prevY;
  }

  /**
   * Get previous theta
   * 
   * @return prevT
   */
  public synchronized static double getPrevTheta() {
    return prevT;
  }
  

}
