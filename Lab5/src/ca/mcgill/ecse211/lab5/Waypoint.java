package ca.mcgill.ecse211.lab5;

import ca.mcgill.ecse211.util.Board;

/**
 * A Waypoint represents a cartesian location on the board.
 * 
 * 
 */
public class Waypoint {

    // X and Y coordinate
    private double x, y;
    
    /**
     * Construct a new waypoint. <br>
     * 
     * <b> NOTE: </b> x, y must represents a center of a given tile
     * 
     * @param x
     * @param y
     */
    public Waypoint(double x, double y) {
        this.x = x;
        this.y = y;
    }
    
    public double getX() {
        return x;
    }
    
    public double getY() {
        return y;
    }
    
    public double getBottomLeftX() {
        return x - Board.TILE_SIZE/2;
    }
    
}
