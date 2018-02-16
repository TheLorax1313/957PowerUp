package org.swarm957.physical;

import edu.wpi.first.wpilibj.AnalogInput;

public class SensorSubsystem {

	AnalogInput distanceSensor = new AnalogInput(0);
	
	public double getDistance() {
		return distanceSensor.getVoltage();
	}
}
