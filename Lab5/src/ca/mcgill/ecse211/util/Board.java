package ca.mcgill.ecse211.util;

import ca.mcgill.ecse211.odometer.Odometer;
import ca.mcgill.ecse211.odometer.OdometerData;

public class Board {
    
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
    
    // Tile length (cm)
    private double tileLength;

    
    // Width in tiles
    private int boardWidthInTiles;
    
    // Height in tiles
    private int boardHeightInTiles;
    
    // Minimum tile multiple
    private static final double MIN_TILE_MULTIPLE = 0.5;
    
    private static final double TILE_SIZE = 30.48;
            
    /**
     * Constructs a square board and sets tile length and board dimensions
     * 
     * @param tileLength
     * @param boardWidthInTiles
     * @throws IllegalArgumentException
     */
    public Board(double tileLength, int boardLengthInTiles) throws IllegalArgumentException {
        this(tileLength, boardLengthInTiles, boardLengthInTiles);
    }
    
    public Board(double tileLength, int boardWidthInTiles, int boardHeightInTiles) throws IllegalArgumentException {
        
        // Check invalid arguments
        if (tileLength <= 0) {
            throw new IllegalArgumentException("Cannot define a tile with 0 or negative length");
        }
        
        if (boardWidthInTiles <= 1) {
            throw new IllegalArgumentException("Cannot define a board that's less than 2xn");
        }
        
        if (boardHeightInTiles <= 1) {
            throw new IllegalArgumentException("Cannot define a board that's less than nx2");
        }
        
        this.tileLength = tileLength;
        this.boardWidthInTiles = boardWidthInTiles;
        this.boardHeightInTiles = boardHeightInTiles;
    }
    
    /**
     * Determine vehicle's heading
     * 
     * @param theta
     * @return Heading
     */
    public static Heading getHeading(double theta) {
        double yComp = Math.abs(Math.cos(theta));
        double xComp = Math.abs(Math.sin(theta));
        
        if (yComp > xComp) {
            if (Math.cos(theta) > 0) {
                return Heading.N;
            }
            return Heading.S;
        }
        
        if (Math.sin(theta) > 0) {
            return Heading.E;
        }
        
        return Heading.W;
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
    public static void snapToGridLine(double[] position, Odometer odometer) {
        
        // Get x, y, theta
        double x = position[0];
        double y = position[1];
        double theta = Math.toRadians(position[2]);
        
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
        
        // Moving in the y direction
        if (heading == Heading.N || heading == Heading.S) {
            odometer.setY(pos);
        }
        // Moving in the x direction
        else {
            odometer.setX(pos);
        }       
    }

    public int getBoardWidthInTiles() {
        return boardWidthInTiles;
    }
    
    public int getBoardHeightInTiles() {
        return boardHeightInTiles;
    }
    
    public double getTileLength() {
        return tileLength;
    }
    
    
}