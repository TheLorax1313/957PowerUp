package org.swarm957.physical;


import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ElevatorSubsystem {

	// Elevator Talon
	TalonSRX elevator = new TalonSRX(6);
	DriverStation ds = DriverStation.getInstance();
	
	// Limit Switches (ROBO RIO DIO!)
	DigitalInput lowSwitch = new DigitalInput(0);
	DigitalInput highSwitch = new DigitalInput(1);
	
	// Timeout for all Talon SRX functions
	public int globalTimeOut = 20;
	
	// Lift Positions (Rounded):
	// Ground, Exchange, Switch, Scale Low, Scale Medium, Scale High
	int[] liftPositions = {30,1724,10400,20000,24236,28250};
	
	// Position Motion Magic homes towards
	int targetPosition = 0;
	
	// Current elevator position
	int currentPos = 0;
	
	// Stores current lift level
	int liftLevel = 0;
	int currentLevel = 0;
	
	// Boolean to check if we have hit a limit switch and encoder
	// position is in the direction of the limit switch
	boolean switchTouched = false;
	public ElevatorSubsystem() {
		
		// Configure Amperage draw limits
		// Current can peak to 29 amps for 10 ms and continous limit at 25 amps
		elevator.configPeakCurrentLimit(29, globalTimeOut);
		elevator.configPeakCurrentDuration(10, globalTimeOut);
		elevator.configContinuousCurrentLimit(25, globalTimeOut);
		elevator.enableCurrentLimit(true);
		
		// Switch to PID slot 0
		
		// Configure our Versa Planetary encoder
		elevator.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, globalTimeOut);
		
		// Configure encoder phase
		elevator.setSensorPhase(true);

		// Configure Feed-Forward
		elevator.config_kF(0,.2177, globalTimeOut);
		
		// Configure P, I, and D
		elevator.config_kP(0, 1.5493, globalTimeOut);
		elevator.config_kI(0, 0.016, globalTimeOut);
		elevator.config_kD(0,15.493, globalTimeOut);
		
		// Configure the zone which I is most effective
		elevator.config_IntegralZone(0, 50, 20);

		// Configure Motion Magic movement rates
		elevator.configMotionCruiseVelocity(2000,globalTimeOut);
		elevator.configMotionAcceleration(2000, globalTimeOut);

		// Reset the encoder to 0
		elevator.setSelectedSensorPosition(0, 0, globalTimeOut);
	}
	
	// Sets the target position to the selected index of lift position array
	// Will not run if the lift has not hit the low limit switch
	public void setLevel(int level) {	
		// Sets the target position of the lift along with the current level of
		// the lift
		targetPosition = liftPositions[level];
		liftLevel = level;
		
		// Checks if the lift is at the low point
		// This is reset in disabled, so it will continue to run
		if(lowSwitch.get()) {
			switchTouched = true;
			SmartDashboard.putBoolean("low switch",switchTouched);
		}

		// If we can move, do so
		if(switchTouched) {
			System.out.println("set level");
			elevator.set(ControlMode.MotionMagic, liftPositions[level]);
		}else {
			// Else, do not move
			elevator.set(ControlMode.PercentOutput, 0);
		}
	}
	
	// Returns the target level of the lift
	public int getLevel() {	
		return liftLevel;
	}
	
	// Returns the raw encoder value
	public int getRaw() {
		return elevator.getSelectedSensorPosition(0);
	}
	
	/** Returns the percentage modifier used by drive code
	 *  when extended
	 *  Prevents tipping due to high speed
	 */
	public double percent() {
		currentPos = elevator.getSelectedSensorPosition(0);
		if(currentPos <= 1300) {
			currentLevel = 0;
		}
		if(currentPos > 1300 && currentPos <= 1800) {
			currentLevel = 1;
		}
		if(currentPos > 1800 && currentPos <= 10500) {
			currentLevel = 2;
		}
		if(currentPos > 10500 && currentPos <= 20500) {
			currentLevel = 3;
		}
		if(currentPos > 20500 && currentPos <= 26000) {
			currentLevel = 4;
		}
		if(currentPos > 26000 && currentPos <= 26000) {
			currentLevel = 5;
		}
		if(currentPos > 20500 && currentPos <= 26000) {
			currentLevel = 6;
		}
		if(currentLevel < 2) {
			return 1;
		}
		if(currentLevel < 3) {
			return 0.65;
		}
		return 0.5;
	}
	
	/** Returns the percentage modifier used by drive code
	 *  when extended
	 *  Prevents tipping due to high speed
	 */
	public double returnTurn() {
		currentPos = elevator.getSelectedSensorPosition(0);
		if(currentPos <= 1300) {
			currentLevel = 0;
		}
		if(currentPos > 1300 && currentPos <= 1800) {
			currentLevel = 1;
		}
		if(currentPos > 1800 && currentPos <= 10500) {
			currentLevel = 2;
		}
		if(currentPos > 10500 && currentPos <= 20500) {
			currentLevel = 3;
		}
		if(currentPos > 20500 && currentPos <= 26000) {
			currentLevel = 4;
		}
		if(currentPos > 26000 && currentPos <= 26000) {
			currentLevel = 5;
		}
		if(currentPos > 20500 && currentPos <= 26000) {
			currentLevel = 6;
		}
		if(currentLevel < 2) {
			return 0.75;
		}
		if(currentLevel < 3) {
			return 0.6;
		}
		return 0.5;
	}
	
	/** Runs during disabled and when initalizing Auto or Teleop
	 *  Sends the Talon information to not move, sets if the bottom switch is touched to false
	 *  
	 *  WHY IS IT HERE:
	 *  This is to prevent the lift from starting at max speed due to a Talon glitch
	 */
	public void disabled() {
		switchTouched = false;
		elevator.set(ControlMode.PercentOutput, 0);
	}
	
	public boolean get() {
		return lowSwitch.get();
	}
}

