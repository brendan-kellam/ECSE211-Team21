package ca.mcgill.ecse211.main;

import ca.mcgill.ecse211.util.Board.Heading;
import ca.mcgill.ecse211.util.Tile;

/**
 * Describes the configuration of current configuration
 */
public class CompetitionConfig {
    
    public static int redTeam;
    public static int greenTeam;
    
    public static Tile startingAreaLL;
    public static Tile startingAreaUR;
    public static Tile islandLL;
    public static Tile islandUR;
    public static Tile searchAreaLL;
    public static Tile searchAreaUR;
    
    /**
     * 
     */
    public static Tile tunnelEntranceToSearchArea;
    public static Tile tunnelEntranceToStartArea;
    
    public static Heading toSearchAreaHeading;
    public static Heading toStartAreaHeading;
    
    public static int corner;
}