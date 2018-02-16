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
	//VisionSubsystem m_vision = new VisionSubsystem();
	
	// Begin Arduino Communication
	NeoStrip m_leds = new NeoStrip(30);
	
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
	
	// Autonomus Switch Statement stage
	int m_autoStep = 0;
	
	// String to hold the Auto mode
	int m_autoMode = 0;
	
	// Ran once on robot initalization
	public void robotInit() {
		
		// Configure the right drivetrain encoder
		m_r1.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 50);
		m_r1.setSensorPhase(true);
		
		// Configure the left drivetrain encoder
		m_l3.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 50);
		m_l3.setSensorPhase(true);
		
		// Reset encoders
		m_encoders.reset();
	}

	// Resets values to prepare for Auto
	public void autonomousInit() {
		
		// Reset Auto stage counter
		m_autoStep = 0;	
		
		// Reset Encoders
		m_encoders.reset();
		
		// Reset Gyro
		m_ahrs.reset();
		
		// Obtain Autonomus mode
		m_autoMode = m_ethernet.autoMode();
	}

	// Autonomus
	public void autonomousPeriodic() {

		// SWITCH CENTER AUTO
		if(m_autoMode == 0) {

		}
		
		// SCALE LEFT AUTO
		if(m_autoMode == 1) {
			
		}
		
		// SCALE RIGHT AUTO
		if(m_autoMode == 2) {
			
		}

		// CROSS AUTOLINE AUTO
		if(m_autoMode == 3) {
			
		}
		
	}
	
	public void disabledPeriodic() {
		m_elevator.setLevel(0);
	}

	// Code ran during the driver-operated period of the Match.
	public void teleopPeriodic() {
		// Drive
		//if(m_ethernet.driveType() == 1) {
		//	m_drive.tankDrive(-m_joystick0.getRawAxis(1), m_joystick1.getRawAxis(1));
		//}else {
			m_drive.arcadeDrive(-m_joystick1.getRawAxis(1), m_joystick1.getRawAxis(0));
		//}
		
		if(m_joystick1.getRawButton(7)) {
			m_elevator.setLevel(0);
		}
		if(m_joystick1.getRawButton(8)) {
			m_elevator.setLevel(1);
		}
		if(m_joystick1.getRawButton(9)) {
			m_elevator.setLevel(2);
		}
		if(m_joystick1.getRawButton(10)) {
			m_elevator.setLevel(3);
		}
		if(m_joystick1.getRawButton(11)) {
			m_elevator.setLevel(4);
		}
		if(m_joystick1.getRawButton(12)) {
			m_elevator.setLevel(5);
		}
		
		if(m_joystick1.getRawButton(3)) {
			m_arm.eject();
		}else {
			if(m_joystick1.getRawButton(4)) {
				m_arm.grab();
			}else {
				m_arm.stop();
			}
		}
	
	}
	
	// Method for controlling LED Lights (5v) and Dashboard
	// readings.
	public void robotPeriodic(){
		if(m_ethernet.alliance() == "blue") {
			m_leds.fill(0, 0, 8);
		}
		if(m_ethernet.alliance() == "red") {
			m_leds.fill(8, 0, 0);
		}
		if(m_ethernet.alliance() == "invalid") {
			m_leds.fill(6, 0, 6);
		}
		m_leds.show();
		SmartDashboard.putNumber("Angle", m_ahrs.getYaw());     
		SmartDashboard.putNumber("Auto Step", m_autoStep);
	}

	// Class to manage Talon encoder feedback
	public class srxEncoders{	
		double cycles = 0;
		double reset = 0;
		double pulses_per_revolution = 0;
		double toInches = 26.937960615884433690468921271645;
		
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
	}	
	
	// Method to drive using camera values
	public void autoDrive(double speed, double deadzone, double input) {			
		if(input < -deadzone) {
       	 m_drive.tankDrive(0,-speed);
        }else {
       	 if(input > deadzone) {
       		 m_drive.tankDrive(speed,0 );
       	 }else {
       		 if((input <= deadzone && input >= -deadzone) || input == -9000) {
       			 m_drive.tankDrive(speed,-speed );
       		 }
       	 }
        }
	}
	
	public class armControl {
		
	}

}