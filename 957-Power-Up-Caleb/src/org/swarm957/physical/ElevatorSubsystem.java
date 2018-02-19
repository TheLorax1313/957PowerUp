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
	
	Thread liftControl = new Thread(new elevatorControl());
	
	// Timeout for all Talon SRX functions
	public int globalTimeOut = 20;
	
	// Lift Positions (Rounded):
	// Ground, Exchange, Switch, Scale Low, Scale Medium, Scale High
	int[] liftPositions = {30,1574,10000,20000,24236,28250};
	
	// Position Motion Magic homes towards
	int targetPosition = 0;
	
	// Current elevator position
	int currentPos = 0;
	
	// Stores current lift level
	int liftLevel = 0;
	int currentLevel = 0;
	
	// Boolean to check if we have hit a limit switch and encoder
	// position is in the direction of the limit switch
	boolean hitSwitch = false;
	
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
		
		// Begin thread controlling the lift mechanism
		liftControl.setDaemon(true);
		liftControl.start();
	
	}
	
	public void setLevel(int level) {
		// Sets the target position to the selected index of lift position array
		targetPosition = liftPositions[level];
		liftLevel = level;
	}
	
	public int getLevel() {
		// Returns the target level of the lift
		return liftLevel;
	}
	
	public double percent() {
		
		if(currentLevel < 2) {
			return 1;
		}
		if(currentLevel < 3) {
			return 0.65;
		}
		return 0.5;
	}
	
	public double returnTurn() {
		
		if(currentLevel < 2) {
			return 0.75;
		}
		if(currentLevel < 3) {
			return 0.6;
		}
		return 0.5;
	}
	
	
	
	public boolean get() {
		return lowSwitch.get();
	}
	
	public class elevatorControl implements Runnable {

		public void run() {
			while(true) {
				
				if(ds.isDisabled()) {
					while((ds.isDisabled()) || (!lowSwitch.get())||(!lowSwitch.get() && ds.isEnabled())) {
						elevator.set(ControlMode.PercentOutput, 0);
						SmartDashboard.putBoolean("Able to move", false);
					}
					try {
						Thread.sleep(1);
					} catch (Exception e) {}
					targetPosition = liftPositions[0];
					liftLevel = 0;
				}
				SmartDashboard.putBoolean("Able to move", true);
				// Gets the current encoder position
				currentPos = elevator.getSelectedSensorPosition(0);

				// If the current position is higher than the target position and the low switch is pressed OR
				// the current position is lower than the target position and the high switch is pressed:
				if((currentPos > targetPosition && lowSwitch.get()) || currentPos < targetPosition && highSwitch.get()) {
					
					// Check if the switch has been hit once (set later)
					if(hitSwitch = false) {
						// Sets the target position to the current position (RAN ONCE!!!!!)
						targetPosition = currentPos;
					}
					
					 //Tells the program that the switch has been hit
					hitSwitch = true;
				}else {
					
					//Tells the program that the witch is not hit
					hitSwitch = false;
				}
				
				// Sets the elevator position based on MotionMagic values (TALON SRX)
				elevator.set(ControlMode.MotionMagic, targetPosition);
				
				// Reports the position of the elevator in encoder counts
				SmartDashboard.putNumber("Elevator: ",elevator.getSelectedSensorPosition(0));
				SmartDashboard.putNumber("Amp Usage", elevator.getOutputCurrent());

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
				
				
				try {
					Thread.sleep(10);
				} catch (Exception e) {}
			}
		}
	}
	
	
}

