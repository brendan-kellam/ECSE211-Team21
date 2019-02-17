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
     */
    public static double boundAngle(double angle) {
        angle = (angle < 0) ? angle + 360 : angle;
        return angle % 360;
    }
    
    
    public static double euclidDistance(double x, double y) {
        return Math.hypot(x * x, y * y);
    }
}
