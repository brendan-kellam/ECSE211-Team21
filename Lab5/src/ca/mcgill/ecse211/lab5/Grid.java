package ca.mcgill.ecse211.lab5;

import java.awt.Point;
import java.util.ArrayList;

import ca.mcgill.ecse211.colour.ColourDetection;
import ca.mcgill.ecse211.hardware.Vehicle;
import ca.mcgill.ecse211.lab5.FieldSearch.StartingCorner;
import ca.mcgill.ecse211.navigation.Navigator;
import ca.mcgill.ecse211.odometer.Odometer;
import ca.mcgill.ecse211.odometer.OdometerExceptions;
import ca.mcgill.ecse211.util.Board;
import ca.mcgill.ecse211.util.Tile;
import lejos.hardware.Sound;
import lejos.hardware.lcd.LCD;

public class Grid {

	
	private ArrayList<Tile> gridTiles = new ArrayList<Tile>();

	/**
	 * Initialize the grid
	 * @param lowerLeftX
	 * @param lowerLeftY
	 * @param topRightX
	 * @param topRightY
	 * @throws OdometerExceptions 
	 */
	
	public Grid(int lowerLeftX,int lowerLeftY, int topRightX,int topRightY) {

		int differenceY = topRightY - lowerLeftY;
		int differenceX = topRightX - lowerLeftX;

		/**
		 * Initialize the grid of tiles
		 */
		for (int j = 0; j < differenceY; j++) {
			for (int i = 0; i < differenceX;i++) {
				gridTiles.add(Tile.lowerLeft(lowerLeftX + i, lowerLeftY));
			}
		}
		
		/**
		 * Add quadrants to the tiles
		 */
		for (Tile currTile : gridTiles) {
			Tile.createQuadrantsForTile(currTile);
		}
	}
	
	public Tile getFirst() {
		return gridTiles.get(0);
	}
	/**
	 * Return a tile
	 * @param lowerLeftX
	 * @param lowerLeftY
	 * @return
	 */
	public Tile getTile(int lowerLeftX,int lowerLeftY) {
		for (Tile currTile : gridTiles) {
			if (currTile.getLowerLeft().getX() == lowerLeftX * Board.TILE_SIZE
					&& currTile.getLowerLeft().getY() == lowerLeftY * Board.TILE_SIZE) {
				return currTile;
			}
		}
		return null;
	}
}
