package ca.mcgill.ecse211.util;

import java.awt.geom.Point2D;
import java.util.ArrayList;

public class Quadrant {


    private Point2D lowerLeft, lowerRight, upperLeft, upperRight, center;
    private Quadrant adjacentQuadrant; //Points to the quadrant that is adjacent to it, but outside it's tile
    private Tile withinTile;
    
    private enum quadrantStatus{
    	UNCHECKED,
    	IDENTIFIEDCAN,
    	UNIDENTIFIEDCAN
    };
    private quadrantStatus quadrantStatus;
    
    /*
     * Construct a quadrant by giving it the lower left and the top right of the quadrant
     */
    public Quadrant(Point2D ll, Point2D ur,Tile tile) {
        this.lowerLeft = ll;
        this.lowerRight = new Point2D.Double(ur.getX(),ll.getY());
        this.upperLeft = new Point2D.Double(ll.getX(),ur.getY());
        this.upperRight = ur;
        this.center = new Point2D.Double(ur.getX() - Board.TILE_SIZE/4, ur.getY() - Board.TILE_SIZE/4);
        this.quadrantStatus = quadrantStatus.UNCHECKED;
    }
    
    /**
     * Construct a quadrant by giving it a top left and a bottom right of the quadrant
     * @param ll
     * @param ur
     * @param tile
     * @param passedCenter
     */
    public Quadrant(Point2D ul, Point2D lr,Tile tile,boolean upperLeftPassed) {
        this.lowerLeft = new Point2D.Double(ul.getX(),lr.getY());
        this.lowerRight = lr;
        this.upperLeft = ul;
        this.upperRight = new Point2D.Double(ul.getY(),lr.getX());
        this.center = new Point2D.Double(ul.getX() + Board.TILE_SIZE/4, lr.getY() + Board.TILE_SIZE/4);
        this.quadrantStatus = quadrantStatus.UNCHECKED;
    }
    
    
	public Quadrant getAdjacentQuadrant() {
		return adjacentQuadrant;
	}
	private void setAdjacentQuadrant(Quadrant adjacentQuadrant) {
		this.adjacentQuadrant = adjacentQuadrant;
	}
	
	public Tile getWithinTile() {
		return withinTile;
	}
	private void setWithinTile(Tile withinTile) {
		this.withinTile = withinTile;
	}
	
	/**
	 * Takes a US reading and adds it to the quadrant
	 * @param carX
	 * @param carY
	 * @param orientation
	 * @param distanceToCan
	 */
	public void addCanToQuadrant(double carX,double carY, double orientation, int distanceToCan) {
		double canX = carX +  distanceToCan * Math.sin(Math.toRadians(orientation));
		double canY = carY +  distanceToCan * Math.cos(Math.toRadians(orientation));
		findQuadrant(canX,canY).setQuadrantStatus(quadrantStatus.UNIDENTIFIEDCAN);
	}
	

	/**
	 * Need to search through the tiles, then place it in the quadrant
	 * @param canX
	 * @param canY
	 * @return
	 */
	private Quadrant findQuadrant(double canX, double canY) {
		// TODO Auto-generated method stub
		return null;
	}
	
	public void setQuadrantStatus(quadrantStatus status) {
		this.quadrantStatus = status;
	}

    
    
}
