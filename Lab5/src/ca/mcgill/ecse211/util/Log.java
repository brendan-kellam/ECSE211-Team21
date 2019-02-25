package ca.mcgill.ecse211.util;

import java.io.File;
import java.io.FileNotFoundException;
import java.io.PrintStream;

/**
 * Logging system -> Enables logging to console or file
 */
public class Log {

	static PrintStream writer = System.out;

	public static enum Sender {
		odometer, Navigator, usSensor, avoidance, USLocalization, colourDetection
	}

	static boolean printOdometer;
	static boolean printNavigator;
	static boolean printUsSensor;
	static boolean printAvoidance;
	static boolean printColour;

	public static void log(Sender sender, String message) {
		long timestamp = System.currentTimeMillis() % 100000;

		if (sender == Sender.Navigator && printNavigator) {
			writer.println("NAV::" + timestamp + ": " + message);
		}
		if (sender == Sender.odometer && printOdometer) {
			writer.println("ODO::" + timestamp + ": " + message);
		}
		if (sender == Sender.usSensor && printUsSensor) {
			writer.println("US::" + timestamp + ": " + message);
		}
		if (sender == Sender.avoidance && printAvoidance){
			writer.println("OA::" + timestamp + ": " + message);
		}
		if (sender == Sender.USLocalization && printAvoidance) {
	        writer.println("USL::" + timestamp + ": " + message);
		}
		if (sender == Sender.colourDetection && printColour) {
	        writer.println("CD::" + timestamp + ": " + message);
		}

	}

	public static void setLogging(boolean nav, boolean odom, boolean us,boolean avoid,boolean colourDetect) {
		printNavigator = nav;
		printOdometer = odom;
		printUsSensor = us;
		printAvoidance = avoid;
		printColour = colourDetect;
	}

	public static void setLogWriter(String filename) throws FileNotFoundException {
		writer = new PrintStream(new File(filename));
	}

}
