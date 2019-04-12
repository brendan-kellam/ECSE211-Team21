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
    
    /**
     * Append an item to the passed string
     * @param builder
     * @param value
     */
    private static void appendString(StringBuilder builder, String value) {
        builder.append(value + System.lineSeparator());
    }
    
    /**
     * Convert the list of parameters to a string to read them. 
     * @return
     */
    public static String tostr() {
        StringBuilder builder = new StringBuilder();
        
        builder.append(System.lineSeparator());
        appendString(builder, "Red Team: " + redTeam);
        appendString(builder, "Green Team: " + greenTeam);
        appendString(builder, "Starting Area LL: " + startingAreaLL.toString());
        appendString(builder, "Starting Area UR: " + startingAreaUR.toString());
        appendString(builder, "Island LL: " + islandLL.toString());
        appendString(builder, "Island UR: " + islandUR.toString());
        appendString(builder, "Search Area LL: " + searchAreaLL.toString());
        appendString(builder, "Search Area UR: " + searchAreaUR.toString());
        appendString(builder, "Tunnel Entrance To Search Area: " + tunnelEntranceToSearchArea.toString());
        appendString(builder, "Tunnel Entrance To Start Area: " + tunnelEntranceToStartArea.toString());
        appendString(builder, "To search area heading: " + toSearchAreaHeading.toString());
        appendString(builder, "To start area heading: " + toStartAreaHeading.toString());
        appendString(builder, "Starting Corner: " + corner);

        
        return builder.toString();
    }
}