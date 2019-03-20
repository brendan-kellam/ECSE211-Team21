package ca.mcgill.ecse211.util;

public class EV3Math {
    /**
     * This method allows the conversion of a distance to the total rotation of each wheel need to
     * cover that distance.
     * 
     * @param radius
     * @param distance
     * @return
     */
    public static int convertDistance(double radius, double distance) {
      return (int) ((180.0 * distance) / (Math.PI * radius));
    }

    /**
     * Converts a desired rotation (angle) into a linear distance
     * 
     * @param radius
     * @param width
     * @param angle
     * @return
     */
    public static int convertAngle(double radius, double width, double angle) {
      return convertDistance(radius, Math.PI * width * angle / 360.0);
    }
    
    
    /**
     * Bounds angle
     * 
     * @param angle
     * @return
     * 
     */
    public static double boundAngle(double angle) {
        // reduce the angle  
        angle =  angle % 360; 

        // force it to be the positive remainder, so that 0 <= angle < 360  
        angle = (angle + 360) % 360; 
        return angle;
    }
    
    /**
     * Compute distance between two angles
     * 
     * @param theta
     * @param d
     * @return distance from theta to d
     */
    public static double distance(double theta, double d) {
        double phi = Math.abs(d - theta) % 360;       // This is either the distance or 360 - distance
        double distance = phi > 180.0 ? 360.0 - phi : phi;
        return distance;
    }
    
}
