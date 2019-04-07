package ca.mcgill.ecse211.util;

import java.awt.Rectangle;
import java.awt.geom.Point2D;

/**
 * Represents a given tile in the course
 */
public class Tile {

    /**
     * Vertices of the tile
     */
    private Point2D lowerLeft, lowerRight, upperLeft, upperRight, center;
    
    //////////////// STATIC FACTORIES ////////////////
    
   
    /**
     * Private constructor for creating a new Tile. <br>
     * 
     * To construct, call one of the static factor methods bellow
     * @param x
     * @param y
     */
    private Tile(Point2D ll, Point2D lr, Point2D ul, Point2D ur) {
        this.lowerLeft = ll;
        this.lowerRight = lr;
        this.upperLeft = ul;
        this.upperRight = ur;
        this.center = new Point2D.Double(ur.getX() - Board.TILE_SIZE/2, ur.getY() - Board.TILE_SIZE/2);
    }
    
    /**
     * Create a new Tile defined by a lower left coordinate point x,y
     * 
     * @param x
     * @param y
     * @return
     */
    public static Tile lowerLeft(int x, int y) {
        double xPos = x * Board.TILE_SIZE;
        double yPos = y * Board.TILE_SIZE;
        
        return new Tile(new Point2D.Double(xPos, yPos),
                        new Point2D.Double(xPos + Board.TILE_SIZE, yPos),
                        new Point2D.Double(xPos, yPos + Board.TILE_SIZE),
                        new Point2D.Double(xPos + Board.TILE_SIZE, yPos + Board.TILE_SIZE));
    }
    
    /**
     * Create a new Tile defined by a lower right coordinate point x,y
     * 
     * @param x
     * @param y
     * @return
     */
    public static Tile lowerRight(int x, int y) {
        double xPos = x * Board.TILE_SIZE;
        double yPos = y * Board.TILE_SIZE;
        
        return new Tile(new Point2D.Double(xPos - Board.TILE_SIZE, yPos),
                        new Point2D.Double(xPos, yPos),
                        new Point2D.Double(xPos - Board.TILE_SIZE, yPos + Board.TILE_SIZE),
                        new Point2D.Double(xPos, yPos + Board.TILE_SIZE));
    }
    
    /**
     * Create a new Tile defined by a upper left coordinate point x,y
     * 
     * @param x
     * @param y
     * @return
     */
    public static Tile upperLeft(int x, int y) {
        double xPos = x * Board.TILE_SIZE;
        double yPos = y * Board.TILE_SIZE;
        
        return new Tile(new Point2D.Double(xPos, yPos-Board.TILE_SIZE),
                        new Point2D.Double(xPos+Board.TILE_SIZE, yPos-Board.TILE_SIZE),
                        new Point2D.Double(xPos, yPos),
                        new Point2D.Double(xPos + Board.TILE_SIZE, yPos));
    }
    
    /**
     * Create a new Tile defined by a upper right coordinate point x,y
     * 
     * @param x
     * @param y
     * @return
     */
    public static Tile upperRight(int x, int y) {
        double xPos = x * Board.TILE_SIZE;
        double yPos = y * Board.TILE_SIZE;
        
        return new Tile(new Point2D.Double(xPos-Board.TILE_SIZE, yPos-Board.TILE_SIZE),
                        new Point2D.Double(xPos, yPos-Board.TILE_SIZE),
                        new Point2D.Double(xPos-Board.TILE_SIZE, yPos),
                        new Point2D.Double(xPos, yPos));
    }
    
    ////////////////////////////////////////////////////////////////
    
    public Point2D getLowerLeft() {
        return lowerLeft;
    }

    public Point2D getLowerRight() {
        return lowerRight;
    }

    public Point2D getUpperLeft() {
        return upperLeft;
    }

    public Point2D getUpperRight() {
        return upperRight;
    }

    public Point2D getCenter() {
        return center;
    }
    
    public double getMinX() {
        return getLowerLeft().getX();
    }
    
    public double getMaxX() {
        return getLowerRight().getX();
    }
    
    public double getMinY() {
        return getLowerLeft().getY();
    }
    
    public double getMaxY() {
        return getUpperLeft().getY();
    }
    
    public String toString() {
        return "ll: " + lowerLeft.toString() + " | lr: " + lowerRight.toString() + " | ul: " + upperLeft.toString() + " | ur: " + upperRight.toString(); 
    }
    
    /**
     * Check if another tile intersects with this one
     * 
     * @param tile
     * @return boolean - true if the tiles intersect
     */
    public boolean intersects(Tile tile) {
                
        return this.getMinX() < tile.getMaxX() &&
               this.getMaxX() > tile.getMinX() &&
               this.getMinY() < tile.getMaxY() &&
               this.getMaxY() > tile.getMinY(); 
    }
    
    /**
     * Check if a arbitrary rectangle intersects with this tile
     * 
     * @param rect
     * @return boolean - true if the rectangle and tile intersect
     */
    public boolean intersects(Rectangle rect) {
        
        return this.getMinX() < rect.getMaxX() &&
               this.getMaxX() > rect.getMinX() &&
               this.getMinY() < rect.getMaxY() &&
               this.getMaxY() > rect.getMinY(); 
    }
    
    
    /**
     * Check if a points is contained in this tile
     * 
     * @param x
     * @param y
     * @return boolean - true if rectangle contains this point
     */
    public boolean contains(double x, double y) {
        
        return this.getMinX() < x &&
               this.getMaxX() > x &&
               this.getMinY() < y &&
               this.getMaxY() > y;
        
    }
    
    
}
