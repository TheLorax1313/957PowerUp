/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.swarm957.physical;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.swarm957.data.*;
import org.swarm957.vision.VisionSubsystem;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.kauailabs.navx.frc.AHRS;

public class MainControlSystem extends TimedRobot {
	
	// Start Ethernet subsystem communication
	DataControlSubsystem m_ethernet = new DataControlSubsystem();
	
	// Start Vision Tracking
	VisionSubsystem m_vision = new VisionSubsystem();
	
	// Enable Encoder tracking
	srxEncoders m_encoders = new srxEncoders(256,2);
	
	// Start Gyro
	AHRS m_ahrs = new AHRS(I2C.Port.kMXP);
	
	// Begin Elevator control
	ElevatorSubsystem m_elevator = new ElevatorSubsystem();
	
	// Begin Arm control
	ArmSubsystem m_arm = new ArmSubsystem(1);
	
	// Drive-train Initialization
	WPI_TalonSRX m_r1 = new WPI_TalonSRX(0);	// Talons
	WPI_TalonSRX m_r2 = new WPI_TalonSRX(1);
	WPI_TalonSRX m_r3 = new WPI_TalonSRX(2);
	WPI_TalonSRX m_l1 = new WPI_TalonSRX(3);
	WPI_TalonSRX m_l2 = new WPI_TalonSRX(4);
	WPI_TalonSRX m_l3 = new WPI_TalonSRX(5);
	SpeedControllerGroup m_right = new SpeedControllerGroup(m_r1, m_r2, m_r3);	// Talon Groups
	SpeedControllerGroup m_left = new SpeedControllerGroup(m_l1, m_l2, m_l3);
	DifferentialDrive m_drive = new DifferentialDrive(m_left, m_right);			// Drivetrain object
	
	// Joystick Initialization
	Joystick m_joystick0 = new Joystick(0);	// Left/Single joystick
	Joystick m_joystick1 = new Joystick(1);	// Right joystick

	// Auto variable initalization
	int m_autoStep = 0;
	int autoCount = 0;
	int m_autoMode = 0;
	double speed = 0;
	
	// Global variables to hold scale/switch locations and encoder counts
	String m_switchLocation = "L";
	String m_scaleLocation = "L";
	int encoderCount = 0;
	
	// Variables used for calculating the RPM of the wheels in RobotPeriodic
	double encoderOffset = 0;
	int runs = 0;
	double RPM = 0;
	
	// Ran once on robot initalization
	public void robotInit() {	
		
		// Configure the right drivetrain encoder
		m_r1.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 50);
		m_r1.setSensorPhase(false);
		
		// Configure the left drivetrain encoder
		m_l3.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 50);
		m_l3.setSensorPhase(true);
		
		// Reset encoders
		m_encoders.reset();
		
		// Set Talon ramping
		setCurrentLimits(m_r1,13,15,50);
		setCurrentLimits(m_r2,13,15,50);
		setCurrentLimits(m_r3,13,15,50);
		setCurrentLimits(m_l1,13,15,50);
		setCurrentLimits(m_l2,13,15,50);
		setCurrentLimits(m_l3,13,15,50);
		
	}

	public void disabledPeriodic() {
		m_elevator.disabled();
	}
	public void teleopInit() {
		m_elevator.disabled();
	}
	
	// Resets values to prepare for Auto
	public void autonomousInit() {
		
		// Enable elevator safeties
		m_elevator.disabled();
		
		// Reset Auto stage counter
		m_autoStep = 0;	
		
		// Reset Encoders
		m_encoders.reset();
		
		// Reset Gyro
		m_ahrs.reset();
		

		// Obtain Autonomus mode
		m_autoMode = m_ethernet.autoMode();
		
		// Obtain Autonomus locations
		m_switchLocation = Character.toString(m_ethernet.switchLocation()[0]);
		m_scaleLocation = Character.toString(m_ethernet.switchLocation()[1]);
		
	}

	public void autonomousPeriodic() {
	
		// SWITCH CENTER AUTO
		if(m_autoMode == 0) {
			switch(m_autoStep) {
			
			// Drive forward a small amount to get away from the wall
			case 0:
				m_elevator.setLevel(ElevatorSubsystem.liftLevels.EXCHANGE);
				if(m_encoders.getDistance() > 6) {
					m_drive.tankDrive(0, 0);
					m_autoStep = 1;
				}else {
					driveStraight(0.6,0);
				}
				
				break;
				
			// Roll forward to 1 foot then move to the next step
			case 1:
				
				if(m_encoders.getDistance() > 11) {
					m_autoStep = 2;
				}
				break;
			
			// Turn towards the side of the Switch is ours
			case 2:
				
				if(m_switchLocation == "L") {
					if(m_ahrs.getYaw() < -35) {
						speed = 0.55;
					}else {
						speed = 0.65;
					}
					
					if(m_ahrs.getYaw() < -43) {
						m_drive.tankDrive(0, 0);
						m_autoStep = 3;
						m_encoders.reset();
					}
					m_drive.tankDrive(-speed,speed);
				}
				
				if(m_switchLocation == "R") {
					if(m_ahrs.getYaw() > 35) {
						speed = 0.4;
					}else {
						speed = 0.7;
					}
					
					if(m_ahrs.getYaw() > 43) {
						m_drive.tankDrive(0, 0);
						m_autoStep = 3;
						m_encoders.reset();
					}
					m_drive.tankDrive(speed,-speed);
				}
				
				break;
				
			// Drive forward to about the center of our switch
			case 3:
				if(m_encoders.getDistance() > 40) {
					speed = 0.4;
				}else {
					speed = 0.6;
				}
				
				
				if(m_encoders.getDistance() > 55) {
					m_drive.tankDrive(0, 0);
					m_autoStep = 4;
				}else {
					if(m_switchLocation == "R")
					driveStraight(speed,45);
					if(m_switchLocation == "L")
						driveStraight(speed,-45);
				}
				break;
				
				
			// Turn forward to face the switch
			case 4:
				
				if(m_switchLocation == "L") {
					if(m_ahrs.getYaw() > -20) {
						speed = 0.45;
					}else {
						speed = 0.6;
					}
					
					if(m_ahrs.getYaw() > -3) {
						m_drive.tankDrive(0, 0);
						m_autoStep = 5;
						m_encoders.reset();
					}
					m_drive.tankDrive(speed,-speed);
				}
				
				if(m_switchLocation == "R") {
					if(m_ahrs.getYaw() < 20) {
						speed = 0.45;
					}else {
						speed = 0.6;
					}
					
					if(m_ahrs.getYaw() < 3) {
						m_drive.tankDrive(0, 0);
						m_autoStep = 5;
						m_encoders.reset();
						autoCount = 0;
					}
					m_drive.tankDrive(-speed,speed);
				}
				
				break;
			
			// Raise lift and drive forward to the switch, and use vision processing if the Limelight is plugged in
			case 5:
				m_elevator.setLevel(ElevatorSubsystem.liftLevels.SWITCH);
				autoCount++;
				autoDrive(0.6,2,m_ethernet.xFinal());
				if(m_ethernet.targetArea() > 4.1 || (RPM < 2 && autoCount > 25)) {
					m_drive.tankDrive(0, 0);
					m_autoStep = 6;
					autoCount = 0;
				}
				break;
			
			// Eject the cube
			case 6:
				
				m_arm.eject();
				autoCount++;
				
				if(autoCount > 50) {
					m_arm.stop();
					m_autoStep = 7;
					autoCount = 0;
				}
				break;
			
			// Back up for 2 seconds
			case 7:
				
				autoCount++;
				
				if(autoCount > 100) {
					m_drive.tankDrive(0, 0);
					m_autoStep = 8;
					m_elevator.setLevel(ElevatorSubsystem.liftLevels.GROUND);
				}else {
					driveStraight(-0.5,0);
				}
				
				break;
				
			case 8:
				
				break;
			}
		}
		
		// SCALE right AUTO
		if(m_autoMode == 2) {
			
			// If the scale is on our side, proceed driving to the scale
			if(m_scaleLocation == "R") {
				
				switch(m_autoStep) {
				
				// Drive to the scale
				case 0:		
					if(m_encoders.getDistance() > 230) {
						speed = 0.5;
					}else {
						speed = 0.8;
					}		
					if((m_encoders.getDistance() > 292)) {
						m_drive.tankDrive(0, 0);
						m_autoStep = 1;
						m_encoders.reset();
						autoCount = 0;
					}else {
						driveStraight(speed,0);
					}		
					if( m_encoders.getDistance() > 384) {
						m_drive.tankDrive(0, 0);
					}
					
					break;
				
				// Turn to face the Scale
				case 1:
		
					if(m_ahrs.getYaw() < -80) {
						speed = 0.7;
					}else {
						speed = 0.7;
					}
					
					if(m_ahrs.getYaw() < -88) {
						m_drive.tankDrive(0, 0);
						m_autoStep = 2;
						m_encoders.reset();
						autoCount = 0;
					}
					m_drive.tankDrive(-speed,speed);
					
					break;
				
				// Back up and square against the wall
				case 2:
					autoCount++;
					m_elevator.setLevel(ElevatorSubsystem.liftLevels.EXCHANGE);
					driveStraight(-0.55, -90);
					if(RPM < 2 && autoCount > 25) {
						m_autoStep = 3;
						autoCount = 0;
					}
					
					break;
					
				// Lift the elevator and proceed after 2 seconds	
				case 3:
					autoCount++;
					m_elevator.setLevel(ElevatorSubsystem.liftLevels.SCALEHIGH);
					if(autoCount > 100) {
						m_autoStep = 4;
						m_encoders.reset();
					}
					break;
				
				// Drive forward to the Scale
				case 4:
					driveStraight(0.5, -90);
					if(m_encoders.getDistance() > 24) {
						m_drive.tankDrive(0, 0);
						m_autoStep = 5;
						autoCount = 0;
					}
					break;
					
				// Eject the cube
				case 5:
					m_arm.eject();
					autoCount++;
					
					if(autoCount > 50) {
						m_arm.stop();
						m_autoStep = 6;
						autoCount = 0;
					}
					break;
					
				// Drive backwards until the wall is hit
				case 6:
					driveStraight(-0.5, -90);
					autoCount++;
					
					if(autoCount > 100 || (RPM < 2 && autoCount > 25)) {
						m_drive.tankDrive(0,0);
						m_autoStep = 7;
						autoCount = 0;
					}
					break;
				
				// Lower the lift
				case 7:
					m_elevator.setLevel(ElevatorSubsystem.liftLevels.GROUND);
					break;
					
				// Empty Case
				case 8:
					break;
				}
			
			// If the scale is not on our side, cross the line
			}else {
				switch(m_autoStep) {
				
				// Cross the line
				case 0:
					m_elevator.setLevel(ElevatorSubsystem.liftLevels.EXCHANGE);
					if(m_encoders.getDistance() > 125) {
						speed = 0.45;
					}else {
						speed = 0.6;
					}
					
					
					if(m_encoders.getDistance() > 155) {
						m_drive.tankDrive(0, 0);
						m_autoStep = 1;
					}else {
						driveStraight(speed,0);
					}
					break;
				
				// Empty Case
				case 1:	
				break;
			}	
		}
		}
		
		// SCALE Left AUTO
		if(m_autoMode == 1) {
			
			// If the scale is on our side
			if(m_scaleLocation == "L") {
				switch(m_autoStep) {
				
				// Drive forward to the Scale
				case 0:	
					if(m_encoders.getDistance() > 170) {
						speed = 0.6;
					}else {
						speed = 0.8;
					}		
					if((m_encoders.getDistance() > 285)) {
						m_drive.tankDrive(0, 0);
						m_autoStep = 1;
						m_encoders.reset();
						autoCount = 0;
					}else {
						driveStraight(speed,0);
					}
					if( m_encoders.getDistance() > 384) {
						m_drive.tankDrive(0, 0);
					}
					
					break;
				
				// Turn to face the Scale
				case 1:
		
					if(m_ahrs.getYaw() > 80) {
						speed = 0.7;
					}else {
						speed = 0.7;
					}
					
					if(m_ahrs.getYaw() > 88) {
						m_drive.tankDrive(0, 0);
						m_autoStep = 2;
						m_encoders.reset();
						autoCount = 0;
					}
					m_drive.tankDrive(speed,-speed);
					break;
				
				// Back up until the Robot hits the wall
				case 2:
					autoCount++;
					m_elevator.setLevel(ElevatorSubsystem.liftLevels.EXCHANGE);
					driveStraight(-0.55, -90);
					if(RPM < 2 && autoCount > 25) {
						m_autoStep = 3;
						autoCount = 0;
					}
					break;
					
				// Lift the lift and wait 2 seconds
				case 3:
					autoCount++;
					m_elevator.setLevel(ElevatorSubsystem.liftLevels.SCALEHIGH);
					if(autoCount > 100) {
						m_autoStep = 4;
						m_encoders.reset();
					}
					break;
				
				// Drive toward the Scale
				case 4:
					driveStraight(0.5, -90);
					if(m_encoders.getDistance() > 24) {
						m_drive.tankDrive(0, 0);
						m_autoStep = 5;
						autoCount = 0;
					}
					break;
					
				// Eject the cube
				case 5:
					m_arm.eject();
					autoCount++;
					
					if(autoCount > 50) {
						m_arm.stop();
						m_autoStep = 6;
						autoCount = 0;
					}
					break;
					
				// Drive back until the wall is hit or 2 seconds pass
				case 6:
					driveStraight(-0.5, -90);
					autoCount++;
					
					if(autoCount > 100 || (RPM < 2 && autoCount > 25)) {
						m_drive.tankDrive(0,0);
						m_autoStep = 7;
						autoCount = 0;
					}
					break;
					
				// Move the elevator to ground level
				case 7:
					m_elevator.setLevel(ElevatorSubsystem.liftLevels.GROUND);
					break;
					
				// Empty Case
				case 8:
					break;
				}
			}else {
				
				// If the Scale is not on our side, cross the line
				switch(m_autoStep) {
				
				// Cross The Line
				case 0:
					m_elevator.setLevel(ElevatorSubsystem.liftLevels.EXCHANGE);
					if(m_encoders.getDistance() > 125) {
						speed = 0.45;
					}else {
						speed = 0.7;
					}	
					if(m_encoders.getDistance() > 155) {
						m_drive.tankDrive(0, 0);
						m_autoStep = 1;			
					}else {
						driveStraight(speed,0);
					}
					break;
				
				// Empty Case
				case 1:
					break;
				}		
			}		
		}

		// CROSS AUTOLINE AUTO
		if(m_autoMode == 3) {
			switch(m_autoStep) {
			case 0:
				
				if(m_encoders.getDistance() > 70) {
					speed = 0.4;
				}else {
					speed = 0.7;
				}
				
				
				if(m_encoders.getDistance() > 101) {
					m_drive.tankDrive(0, 0);
					m_autoStep = 1;
				}else {
					driveStraight(speed,0);
				}
				
				break;
				
			case 1:
				
				break;
			}
		}
		
		// SWITCH LEFT OR CROSS THE LINE
		if(m_autoMode == 4) {
			switch(m_autoStep) {
			
			// Raise lift slightly and drive forward
			case 0:
				m_elevator.setLevel(ElevatorSubsystem.liftLevels.EXCHANGE);
				if(m_encoders.getDistance() > 125) {
					speed = 0.45;
				}else {
					speed = 0.7;
				}
				if(m_encoders.getDistance() > 155) {
					m_drive.tankDrive(0, 0);
					if(m_switchLocation == "L") {
						m_autoStep = 1;
					}
					
				}else {
					driveStraight(speed,0);
				}
				break;
			
			// Turn and lift the elevator to Switch height
			case 1:
				if(m_ahrs.getYaw() > 80) {
					speed = 0.45;
				}else {
					speed = 0.7;
				}
				m_elevator.setLevel(ElevatorSubsystem.liftLevels.SWITCH);
				if(m_ahrs.getYaw() > 88) {
					m_drive.tankDrive(0, 0);
					m_autoStep = 2;
					m_encoders.reset();
					autoCount = 0;
				}
				m_drive.tankDrive(speed,-speed);
				break;
				
			// Drive forward until the wall is hit
			case 2:	
				autoCount++;
				
				if(autoCount > 25 && RPM < 2) {
					m_drive.tankDrive(0, 0);
					m_autoStep = 3;
					m_encoders.reset();
					autoCount = 0;
				}else {
					driveStraight(0.6,90);
				}
				break;
				
			// Eject the cube
			case 3:
				m_arm.eject();
				autoCount++;
				
				if(autoCount > 50) {
					m_arm.stop();
					m_autoStep = 4;
					autoCount = 0;
				}
				break;
			
			// Drive back until the wall is hit
			case 4:
				driveStraight(-0.6, 90);
				autoCount++;
				
				if(autoCount > 100 || (RPM < 2 && autoCount > 25)) {
					m_drive.tankDrive(0,0);
					m_autoStep = 5;
					autoCount = 0;
				}
				break;
				
			// Lower the elevator
			case 5:
				m_elevator.setLevel(ElevatorSubsystem.liftLevels.GROUND);
				break;
			}
		}
		
		// SWITCH RIGHT OR CROSS THE LINE
		if(m_autoMode == 5) {
			
			switch(m_autoStep) {
			
			// Lift elevator and drive to the Switch
			case 0:
				m_elevator.setLevel(ElevatorSubsystem.liftLevels.EXCHANGE);
				if(m_encoders.getDistance() > 125) {
					speed = 0.45;
				}else {
					speed = 0.6;
				}	
				
				if(m_encoders.getDistance() > 155) {
					m_drive.tankDrive(0, 0);
					if(m_switchLocation == "R") {
						m_autoStep = 1;
					}		
				}else {
					driveStraight(speed,0);
				}
				break;
			
			// Turn to the Switch and raise the lift
			case 1:
				if(m_ahrs.getYaw() < -80) {
					speed = 0.45;
				}else {
					speed = 0.6;
				}
				m_elevator.setLevel(ElevatorSubsystem.liftLevels.SWITCH);
				if(m_ahrs.getYaw() < -88) {
					m_drive.tankDrive(0, 0);
					m_autoStep = 2;
					m_encoders.reset();
					autoCount = 0;
				}
				m_drive.tankDrive(-speed,speed);
				break;
			
			// Drive to the switch
			case 2:	
				autoCount++;
				
				if(autoCount > 25 && RPM < 2) {
					m_drive.tankDrive(0, 0);
					m_autoStep = 3;
					m_encoders.reset();
					autoCount = 0;
				}else {
					driveStraight(0.6,90);
				}
				break;
				
			// Eject the cube
			case 3:
				m_arm.eject();
				autoCount++;
				
				if(autoCount > 50) {
					m_arm.stop();
					m_autoStep = 4;
					autoCount = 0;
				}
				break;
			
			// Back up against the wall
			case 4:
				driveStraight(-0.6, 90);
				autoCount++;
				
				if(autoCount > 100 || (RPM < 2 && autoCount > 25)) {
					m_drive.tankDrive(0,0);
					m_autoStep = 5;
					autoCount = 0;
				}
				break;
				
			// Lower the elevator
			case 5:
				m_elevator.setLevel(ElevatorSubsystem.liftLevels.GROUND);
				break;
			}
			
		}
			
		// Left Cross Scale
		if(m_autoMode == 6) {
			
			// If the Scale is on our side
			if(m_scaleLocation == "L") {
				switch(m_autoStep) {
				
				// Drive forward to the Scale
				case 0:
					m_elevator.setLevel(ElevatorSubsystem.liftLevels.EXCHANGE);
					if(m_encoders.getDistance() > 172) {
						speed = 0.45;
					}else {
						speed = 0.6;
					}
					if((m_encoders.getDistance() > 220)) {
						m_drive.tankDrive(0, 0);
						m_autoStep = 1;
						m_encoders.reset();
						autoCount = 0;
					}else {
						driveStraight(speed,0);
					}	
					break;
				
				// Turn towards the Scale and raise the lift
				case 1:
					if(m_ahrs.getYaw() > 25) {
						speed = 0.5;
					}else {
						speed = 0.55;
					}
					m_elevator.setLevel(ElevatorSubsystem.liftLevels.SCALEHIGH);
					if(m_ahrs.getYaw() > 30) {
						m_drive.tankDrive(0, 0);
						m_autoStep = 2;
						m_encoders.reset();
						autoCount = 0;
					}
					m_drive.tankDrive(speed,-speed);
					
					break;
					
				// Drive to scale
				case 2:
					driveStraight(0.5,24);
					if((m_encoders.getDistance() > 43)) {
						m_drive.tankDrive(0, 0);
						m_autoStep = 3;
						m_encoders.reset();
						autoCount = 0;
					}
					break;
				// Eject the Power Cube
				case 3:
					m_arm.eject();
					autoCount++;
					
					if(autoCount > 50) {
						m_arm.stop();
						m_autoStep = 4;
						autoCount = 0;
					}
					break;
				// Back Up
				case 4:

					autoCount++;
					driveStraight(-0.5, 0);
					if(autoCount > 75) {
						m_elevator.setLevel(ElevatorSubsystem.liftLevels.GROUND);
						m_autoStep = 5;
						autoCount = 0;
					}
					break;
				case 5:
				break;
				}	
			}
		}
		
		// Right Cross Scale
		if(m_autoMode == 7) {
			// Our Side
			if(m_scaleLocation == "R") {
				switch(m_autoStep) {
				// Drive forward
				case 0:
					
					if(m_encoders.getDistance() > 172) {
						speed = 0.45;
					}else {
						speed = 0.6;
					}
					if((m_encoders.getDistance() > 220)) {
						m_drive.tankDrive(0, 0);
						m_autoStep = 1;
						m_encoders.reset();
						autoCount = 0;
					}else {
						driveStraight(speed,0);
					}
					
					break;
					
				// Turn and raise the elevator	
				case 1:
					
					if(m_ahrs.getYaw() < -20) {
						speed = 0.45;
					}else {
						speed = 0.5;
					}
					m_elevator.setLevel(ElevatorSubsystem.liftLevels.SCALEHIGH);
					if(m_ahrs.getYaw() < -30) {
						m_drive.tankDrive(0, 0);
						m_autoStep = 2;
						m_encoders.reset();
						autoCount = 0;
					}
					m_drive.tankDrive(-speed,speed);
					
					break;
				// Drive to scale
				case 2:
					driveStraight(0.5,24);
					if((m_encoders.getDistance() > 46)) {
						m_drive.tankDrive(0, 0);
						m_autoStep = 3;
						m_encoders.reset();
						autoCount = 0;
					}
					break;
				// Shoot	
				case 3:
					m_arm.eject();
					autoCount++;
					
					if(autoCount > 50) {
						m_arm.stop();
						m_autoStep = 4;
						autoCount = 0;
					}
					break;
				// Back Up
				case 4:

					autoCount++;
					driveStraight(-0.5, 0);
					if(autoCount > 75) {
						m_elevator.setLevel(ElevatorSubsystem.liftLevels.GROUND);
						m_autoStep = 5;
						autoCount = 0;
					}
					break;
				case 5:
				break;
				}
			}
		
			// Opposite side (Cross)
			if(m_switchLocation == "L") {
				switch(m_autoStep) {
				case 0:
					m_elevator.setLevel(ElevatorSubsystem.liftLevels.EXCHANGE);
					if(m_encoders.getDistance() > 160) {
						speed = 0.55;
					}else {
						speed = 0.7;
					}
					// Aiming for 226-20
					if((m_encoders.getDistance() > 204)) {
						m_drive.tankDrive(0, 0);
						m_autoStep = 1;
						m_encoders.reset();
						autoCount = 0;
					}else {
						driveStraight(speed,0);
					}
					
					break;
					
				case 1:
					// turn
					if(m_ahrs.getYaw() < -70) {
						speed = 0.55;
					}else {
						speed = .6;
					}
	
					if(m_ahrs.getYaw() < -75) {
						m_drive.tankDrive(0, 0);
						m_autoStep = 2;
						m_encoders.reset();
						autoCount = 0;
					}
					m_drive.tankDrive(-speed,speed);
					
					break;
					// Drive over the bump and over to the other side of the Scale
				case 2:
					if(m_encoders.getDistance() > 200) {
						speed = 0.55;
					}else {
						speed = 0.7;
					}
					// Aiming for 226-20
					if((m_encoders.getDistance() > 230)) {
						m_drive.tankDrive(0, 0);
						m_autoStep = 3;
						m_encoders.reset();
						autoCount = 0;
					}else {
						driveStraight(speed,-90);
					}
					break;
					// Turn forward
				case 3:
					
					// turn
					if(m_ahrs.getYaw() > 16) {
						speed = 0.55;
					}else {
						speed = 0.60;
					}
					m_elevator.setLevel(ElevatorSubsystem.liftLevels.SCALEHIGH);
	
					if(m_ahrs.getYaw() > 20) {
						m_drive.tankDrive(0, 0);
						m_autoStep = 4;
						m_encoders.reset();
						autoCount = 0;
					}
					m_drive.tankDrive(speed,-speed);
					
					break;
					// Drive to scale
				case 4:
					driveStraight(0.5,30);
					if((m_encoders.getDistance() > 56)) {
						m_drive.tankDrive(0, 0);
						m_autoStep = 5;
						m_encoders.reset();
						autoCount = 0;
					}
					break;
				// Shoot	
				case 5:
					m_arm.eject();
					autoCount++;
					
					if(autoCount > 50) {
						m_arm.stop();
						m_autoStep = 6;
						autoCount = 0;
					}
					break;
				// Back Up
				case 6:
	
					autoCount++;
					driveStraight(-0.5, 0);
					if(autoCount > 75) {
						m_elevator.setLevel(ElevatorSubsystem.liftLevels.GROUND);
						m_autoStep = 7;
						autoCount = 0;
					}
					break;
				case 7:
				break;				
				}	
			}
		}
		
		runs++;
		if(runs > 4) {
			RPM = (Math.abs(m_encoders.getRaw()-encoderOffset)/1024)*600;
			runs = 0;
			encoderOffset = m_encoders.getRaw();
		}
	}

	// Code ran during the driver-operated period of the Match.
	public void teleopPeriodic() {

		m_drive.arcadeDrive(-m_joystick0.getRawAxis(1)*m_elevator.percent(), m_joystick0.getRawAxis(2)*m_elevator.returnTurn());

		
		if(m_joystick1.getRawButton(XBox.LBumper.value())) {
			m_elevator.setLevel(ElevatorSubsystem.liftLevels.GROUND);
		}
		if(m_joystick1.getRawButton(XBox.RBumper.value())) {
			m_elevator.setLevel(ElevatorSubsystem.liftLevels.EXCHANGE);
		}
		if(m_joystick1.getRawButton(XBox.A.value())) {
			m_elevator.setLevel(ElevatorSubsystem.liftLevels.SWITCH);
		}
		if(m_joystick1.getRawButton(XBox.B.value())) {
			m_elevator.setLevel(ElevatorSubsystem.liftLevels.SCALELOW);
		}
		if(m_joystick1.getRawButton(XBox.X.value())) {
			m_elevator.setLevel(ElevatorSubsystem.liftLevels.SCALEMID);
		}
		if(m_joystick1.getRawButton(XBox.Y.value())) {
			m_elevator.setLevel(ElevatorSubsystem.liftLevels.SCALEHIGH);
		}
		
		if(m_joystick0.getRawButton(2)) {
			m_arm.eject();
		}else {
			if(m_joystick0.getRawButton(1)) {
				m_arm.grab();
			}else {
				m_arm.stop();
			}
		}
	}
	
	// Method for controlling LED Lights (5v) and Dashboard
	// readings.
	public void robotPeriodic(){
	}

	// Class to manage Talon encoder feedback
	public class srxEncoders{	
		double cycles = 0;
		double reset = 0;
		double pulses_per_revolution = 0;
		double toInches = 55.245648042746041975368465658798;
		
		public srxEncoders(int pulses, int channels) {
			pulses_per_revolution = pulses*channels;
			
		}
	
		public void reset() {
			m_r1.setSelectedSensorPosition(0, 0, 50);
			m_l3.setSelectedSensorPosition(0, 0, 50);
		}
		
		public double getDistance() {
			cycles = (m_r1.getSelectedSensorPosition(0) + m_l3.getSelectedSensorPosition(0))/2;
			return (cycles/toInches);
		}
		
		public int getRaw() {
			return (m_r1.getSelectedSensorPosition(0) + m_l3.getSelectedSensorPosition(0))/2;
		}
	}	
	
	// Method to drive using camera values
	public void autoDrive(double speed, double deadzone, double input) {			
		if(input < -deadzone) {
       	 m_drive.tankDrive(0,speed);
        }else {
       	 if(input > deadzone) {
       		 m_drive.tankDrive(speed,0 );
       	 }else {
       		 if((input <= deadzone && input >= -deadzone) || input == -9000) {
       			 m_drive.tankDrive(speed,speed );
       		 }
       	 }
        }
	}
	
	public void driveStraight(double speed, double heading) {
		double gyro = m_ahrs.getYaw();
		double turn = 0;
		if(gyro > heading+0.5) {
			turn = -0.2;
		}
		if(gyro < heading-0.5) {
			turn = 0.32;	
		}
		if(gyro >= heading-0.5 && gyro <= heading + 0.5) {
			turn = 0;
		}
		
		m_drive.arcadeDrive(speed, turn);
	}

	public void setCurrentLimits(WPI_TalonSRX talon, int constantDraw, int spikeDraw, int maxSpikeTime) {
		talon.configContinuousCurrentLimit(constantDraw, 20);
		talon.configPeakCurrentLimit(spikeDraw, 20);
		talon.configPeakCurrentDuration(maxSpikeTime, 20);
		talon.enableCurrentLimit(true);
	}

	// Enum for XBox buttons
	public enum XBox {
		A(1), B(2), X(3), Y(4), LBumper(5), RBumper(6);	
		private int value;	
		private XBox(int val) {
			this.value = val;
		}
		public int value() {
			return this.value;
		}
	}
}