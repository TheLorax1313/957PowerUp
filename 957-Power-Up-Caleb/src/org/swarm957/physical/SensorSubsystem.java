package org.swarm957.physical;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DigitalInput;

public class SensorSubsystem {

	// Cube Distance Sensors
	AnalogInput distanceSensor0 = new AnalogInput(0);
	AnalogInput distanceSensor1 = new AnalogInput(1);
	
	// Beam Sensor
	DigitalInput beam2 = new DigitalInput(2);
	
	// Time cube has been in possession of the robot
	double cubeTime = 0;
	
	/* Returns the distance to the cube.
	   If the cube has not broken the beam sensor
	   (the frame perimeter), it returns -1. */
	public double getDistance() {
		if(!beam2.get()) {
			return (distanceSensor0.getAverageVoltage() + distanceSensor1.getAverageVoltage())/2;
		}
		return -1;
	}
	
	// Returns if the robot has a cube or not
	public boolean hasCube() {
		if(!beam2.get()) {
			return true;
		}
		return false;
	}
	
	// Returns the time the cube has been in the robot's possession and increases it
	// This function should be returned to a global variable every time Auto loops only once
	public double cubeTime() {
		if(!beam2.get()) {
			cubeTime = cubeTime + 0.02;
			return cubeTime-0.02;
		}
		cubeTime = 0;
		return cubeTime;	
	}

	// Resets the time the cube has been in robot possession
	// Ran in TeleopInit and AutoInit
	public void resetCubeTime() {
		cubeTime = 0;
	}
}
