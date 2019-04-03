package ca.mcgill.ecse211.util;

import ca.mcgill.ecse211.odometer.Odometer;
import ca.mcgill.ecse211.odometer.OdometerData;

public final class Board {
    
    /** 
     * Represents the physical board with grid lines
     * 
     * 
     * Coordinate system:
     * y
     * ^
     * |
     *  -----> x
     *  
     *  Example <b>3x3</b> board:
     *  ________
     * |__|__|__|
     * |__|__|__|
     * |__|__|__| 
     * 
     *
     **/
   
    public enum Heading {
        N, 
        S,
        E,
        W
    }
    
    // Minimum tile multiple
    private static final double MIN_TILE_MULTIPLE = 0.5;
    
    // Tile size
    public static final double TILE_SIZE = 30.48;
    
    /**
     * Board width. 
     * 
     * @see <a href="https://mycourses2.mcgill.ca/d2l/le/content/372671/viewContent/4289469/View">Project Spec</a>
     */
    public static final int BOARD_WIDTH = 15;
    
    /**
     * Board height. 
     * 
     * @see <a href="https://mycourses2.mcgill.ca/d2l/le/content/372671/viewContent/4289469/View">Project Spec</a>
     */
    public static final int BOARD_HEIGHT = 9;
    
    
    /**
     * Determine vehicle's heading </br>
     * 
     * <b> NOTE: </b> This method will bound theta to: 0 <= theta < 360 <br>
     * @see EV3Math.boundAngle(theta)
     * 
     * @param theta
     * @return Heading
     */
    public static Heading getHeading(double theta) {
        
        theta = EV3Math.boundAngle(theta);
                        
        if (theta <= 45) {
            return Heading.N;
        } else if (theta <= 135) {
            return Heading.E;
        } else if (theta <= 225) {
            return Heading.S;
        } else if (theta <= 315) {
            return Heading.W;
        } else {
            return Heading.N;
        }
    }
    
    /**
     * Convert a heading to a angle
     * 
     * @param Robot heading
     * @return angle representing the heading
     */
    public static double getHeadingAngle(Heading heading) {
        switch (heading) {
        case N:
            return 0;
        case E:
            return 90;
        case S:
            return 180;
        case W:
            return 270;
        }
        
        return 0;
    }
    
    
    /**
     * Snaps to a given gridline.
     * 
     * Given a position, the method will determine the heading of the robot (either the x or y component) and determine
     * the closest gridline to it's current position in said heading
     * 
     * @param position
     * @param odometer
     */
    public static void snapToGridLine(Odometer odometer) {
        
        // Get x, y, theta
        double x = Odometer.getX();
        double y = Odometer.getY();
        double theta = Odometer.getTheta();
        
        // Fills the new position of the robot after correction
        double pos; 
        
        Heading heading = getHeading(theta);
        
        // Moving in the y direction
        if (heading == Heading.N || heading == Heading.S) {
            pos = y - OdometerData.yoffset;
        }
        
        // Moving in the x direction
        else {
            pos = x - OdometerData.xoffset;
        }
        
        // For the current position, determine the number of tileLength multiples
        double div = pos / TILE_SIZE;
        
        // If the multiple is less than the minimum multiple
        if (Math.abs(div) < MIN_TILE_MULTIPLE) {
            // Reset position to 0
            pos = 0;
            
        // Otherwise, set position to the rounded multiple multiplied by the tile length
        } else {
            pos = TILE_SIZE * Math.round(div);
        }
        
        String msg = "Snap to ";
        
        // Moving in the y direction
        if (heading == Heading.N || heading == Heading.S) {
            odometer.setY(pos);
            msg += "Y: ";
        }
        // Moving in the x direction
        else {
            odometer.setX(pos);
            msg += "X: ";
        }       
        
        msg += pos;
        
        Log.log(Log.Sender.board, msg);
    }
    
    public static void snapToHeading(Odometer odometer) {
        Heading heading = getHeading(Odometer.getTheta());
        
        odometer.setTheta(getHeadingAngle(heading));
    }
    
    /**
     * 
     */
    public static final class Config {
        
        public static int redTeam;
        public static int greenTeam;
        
        public static Tile startingAreaLL;
        public static Tile startingAreaUR;
        public static Tile islandLL;
        public static Tile islandUR;
        public static Tile searchAreaLL;
        public static Tile searchAreaUR;
        public static Tile tunnelLL;
        public static Tile tunnelUR;
        public static int corner;
    }
    
}