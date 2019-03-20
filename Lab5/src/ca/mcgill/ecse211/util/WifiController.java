package ca.mcgill.ecse211.util;

import java.io.IOException;
import java.net.UnknownHostException;
import java.util.Map;

import org.json.simple.parser.ParseException;

import ca.mcgill.ecse211.WiFiClient.WifiConnection;
import lejos.hardware.Button;

public class WifiController {

    // ** Set these as appropriate for your team and current situation **//
    private static final String SERVER_IP = "192.168.2.50";
    private static final int TEAM_NUMBER = 21;

    // Enable/disable printing of debug info from the WiFi class
    private static final boolean ENABLE_DEBUG_WIFI_PRINT = true;
    
    private static int redTeam;
    private static int redCorner;
    

    private static int greenTeam;
    private static Tile redLL;
    private static Tile redUR;
    private static Tile islandLL;
    private static Tile islandUR;
    private static Tile tunnelLL;
    private static Tile tunnelUR;
    
    public static void fetchGameplayData() {
        // Initialize WifiConnection class
        WifiConnection conn = new WifiConnection(SERVER_IP, TEAM_NUMBER, ENABLE_DEBUG_WIFI_PRINT);
        
        try {
            Map data = conn.getData();
            
            redTeam = getInt(data, "RedTeam");
            redCorner = getInt(data, "RedCorner");
            greenTeam = getInt(data, "GreenTeam");
            
            redLL =    Tile.lowerLeft(getInt(data, "Red_LL_x"),     getInt(data, "Red_LL_y"));
            redUR =    Tile.upperRight(getInt(data, "Red_UR_x"),    getInt(data, "Red_UR_y"));            
            islandLL = Tile.lowerLeft(getInt(data, "Island_LL_x"),  getInt(data, "Island_LL_y"));            
            islandUR = Tile.upperRight(getInt(data, "Island_UR_x"), getInt(data, "Island_UR_y"));            
            tunnelLL = Tile.lowerRight(getInt(data, "TNR_LL_x"),     getInt(data, "TNR_LL_y"));            
            tunnelUR = Tile.upperLeft(getInt(data, "TNR_UR_x"),    getInt(data, "TNR_UR_y"));            
            
        } catch (Exception e) {
            System.err.println("Error: " + e.getMessage());
        }
        
        Button.waitForAnyPress();
    }
    
    private static int getInt(Map data, String key) {
        return ((Long) data.get(key)).intValue();
    }
    
    public static int getRedTeam() {
        return redTeam;
    }

    public static int getRedCorner() {
        return redCorner;
    }

    public static int getGreenTeam() {
        return greenTeam;
    }

    public static Tile getRedLL() {
        return redLL;
    }

    public static Tile getRedUR() {
        return redUR;
    }

    public static Tile getIslandLL() {
        return islandLL;
    }

    public static Tile getIslandUR() {
        return islandUR;
    }

    public static Tile getTunnelLL() {
        return tunnelLL;
    }

    public static Tile getTunnelUR() {
        return tunnelUR;
    }
    
}
