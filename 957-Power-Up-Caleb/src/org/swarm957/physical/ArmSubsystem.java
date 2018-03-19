package org.swarm957.physical;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

public class ArmSubsystem {

	// Arm Talon SRX definitions
	TalonSRX leftWheels = new TalonSRX(7),
			rightWheels = new TalonSRX(8);
	
	// Arm speed
	double speed = 0;
	
	// TalonSRX timeout
	int globalTimeOut = 20;
	
	public ArmSubsystem(double speed) {
		
		// Set default wheel speed
		this.speed = speed;
		
		// Invert the right talon
		rightWheels.setInverted(true);
		
		// Set current limits on both talons
		leftWheels.configPeakCurrentLimit(10, globalTimeOut);
		leftWheels.configPeakCurrentDuration(10, globalTimeOut);
		leftWheels.configContinuousCurrentLimit(7, globalTimeOut);
		leftWheels.enableCurrentLimit(true);
		rightWheels.configPeakCurrentLimit(10, globalTimeOut);
		rightWheels.configPeakCurrentDuration(10, globalTimeOut);
		rightWheels.configContinuousCurrentLimit(7, globalTimeOut);
		rightWheels.enableCurrentLimit(true);
	}
	
	public void grab() {
		leftWheels.set(ControlMode.PercentOutput, -speed);
		rightWheels.set(ControlMode.PercentOutput, -speed);
	}
	
	public void eject() {
		leftWheels.set(ControlMode.PercentOutput, speed);
		rightWheels.set(ControlMode.PercentOutput, speed);
	}
	
	public void stop() {
		leftWheels.set(ControlMode.PercentOutput, 0);
		rightWheels.set(ControlMode.PercentOutput, 0);
	}
	
	public void directControl(double speed) {
		rightWheels.set(ControlMode.PercentOutput, speed);
		rightWheels.set(ControlMode.PercentOutput, speed);
	}
}