package ca.mcgill.ecse211.claw;

import ca.mcgill.ecse211.sensor.UltrasonicPoller;
import ca.mcgill.ecse211.util.Vehicle;
import lejos.hardware.motor.EV3MediumRegulatedMotor;

public class Claw {

	/**
	 * Colour sensor motor
	 */
	//One motor for the light sensor.
	public final static EV3MediumRegulatedMotor clawMotor = Vehicle.CLAW_MOTOR;
	// Ultrasonic poller
	private UltrasonicPoller usPoller;
	
	public Claw(UltrasonicPoller usPoller) {
 		this.usPoller = usPoller;
	}
	
	/**
	 * Allow the claw to grab a can that is directly in front of the robot.
	 */
	public void grab() {
		clawMotor.rotate(-130);
	}
	
	/**
	 * Allow the claw to release the can that it is holding 
	 */
	public void release() {
		clawMotor.rotate(120);
	}
	
	/*
	 * Allow the motor to approach the can at a distance that can grab it.
	 */
	public void approachCan() {
		Vehicle.setMotorSpeeds(100, 100); 
		while (usPoller.getDistance() > 3);
		Vehicle.setMotorSpeeds(0, 0);
	}
	
	
}
