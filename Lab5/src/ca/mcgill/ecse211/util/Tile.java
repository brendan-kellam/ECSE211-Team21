package ca.mcgill.ecse211.util;

import java.awt.geom.Point2D;
import java.util.ArrayList;

/**
 * Represents a given tile in the course
 */
public class Tile {

    /**
     * Vertices of the tile
     */
    private Point2D lowerLeft, lowerRight, upperLeft, upperRight, center;
    private Quadrant llQuadrant, lrQuadrant,ulQuadrant,urQuadrant;
    
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
    
    public String toString() {
        return "ll: " + lowerLeft.toString() + " | lr: " + lowerRight.toString() + " | ul: " + upperLeft.toString() + " | ur: " + upperRight.toString(); 
    }

    ////////////////////////////////////////////////////////////////
    
	public Quadrant getLlQuadrant() {
		return llQuadrant;
	}

	private void setLlQuadrant(Quadrant llQuadrant) {
		this.llQuadrant = llQuadrant;
	}

	public Quadrant getLrQuadrant() {
		return lrQuadrant;
	}

	private void setLrQuadrant(Quadrant lrQuadrant) {
		this.lrQuadrant = lrQuadrant;
	}

	private Quadrant getUlQuadrant() {
		return ulQuadrant;
	}

	private void setUlQuadrant(Quadrant ulQuadrant) {
		this.ulQuadrant = ulQuadrant;
	}

	private Quadrant getUrQuadrant() {
		return urQuadrant;
	}

	private void setUrQuadrant(Quadrant urQuadrant) {
		this.urQuadrant = urQuadrant;
	}
	
	/**
	 * Creates quadrants for a tile
	 * @param tile
	 */
	public static void createQuadrantsForTile(Tile tile){
		tile.setLlQuadrant(new Quadrant(tile.getLowerLeft(),tile.getCenter(),tile));
		tile.setLrQuadrant(new Quadrant(tile.getCenter(),tile.getLowerRight(),tile,true));
		tile.setUlQuadrant(new Quadrant(tile.getUpperLeft(),tile.getCenter(),tile,true));
		tile.setUrQuadrant(new Quadrant(tile.getCenter(),tile.getUpperRight(),tile));
	}
}
