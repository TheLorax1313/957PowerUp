/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.swarm957.robot;

import edu.wpi.cscore.CvSink;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.cscore.CvSource;
import edu.wpi.cscore.MjpegServer;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.cscore.VideoMode;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.math.BigDecimal;
import java.util.ArrayList;
import java.util.List;

import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.swarm957.data.*;
import org.swarm957.vision.CubeFinder;
import org.swarm957.vision.VisionControl;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.kauailabs.navx.frc.AHRS;

public class Robot extends IterativeRobot {

	// Definitions of major non-drive robot systems (and lights!)
	NetworkData m_ethernet = new NetworkData();			// DS and Limelight communication
	VisionControl m_vision = new VisionControl();		// Non-Limelight vision
	NeoStrip m_leds = new NeoStrip(30);					// LEDS
	srxEncoders m_encoders = new srxEncoders(256,2);	// Encoders
	AHRS m_ahrs = new AHRS(I2C.Port.kMXP);				// Gyroscope (Nav-X)

	// Drive-train Initialization
	WPI_TalonSRX m_r1 = new WPI_TalonSRX(0);	// Talons
	WPI_TalonSRX m_r2 = new WPI_TalonSRX(1);
	WPI_TalonSRX m_r3 = new WPI_TalonSRX(2);
	WPI_TalonSRX m_l1 = new WPI_TalonSRX(3);
	WPI_TalonSRX m_l2 = new WPI_TalonSRX(4);
	WPI_TalonSRX m_l3 = new WPI_TalonSRX(5);
	SpeedControllerGroup m_right = new SpeedControllerGroup(m_r1, m_r2, m_r3);	// Left/Right Groups of controllers
	SpeedControllerGroup m_left = new SpeedControllerGroup(m_l1, m_l2, m_l3);
	DifferentialDrive m_drive = new DifferentialDrive(m_left, m_right);			// Drivetrain object
	
	// Joystick Initialization
	Joystick m_joystick0 = new Joystick(0);	
	Joystick m_joystick1 = new Joystick(1);
    
	// Autonomus Stage value holder (carries over between
	// program cycles).
	int m_autoStep = 0;
	
	// Ran once on robot initalization
	public void robotInit() {
		m_r1.setInverted(true);
		m_r2.setInverted(true);
		m_r3.setInverted(true);	
		m_r1.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 50);
		m_r1.setSensorPhase(true);
		m_l3.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 50);
		m_l3.setSensorPhase(true);
		m_encoders.reset();
	}

	// Resets values to prepare for Auto
	public void autonomousInit() {
		m_autoStep = 0;	
		m_encoders.reset();
	}

	// Incomplete. Check back Week 6.
	public void autonomousPeriodic() {

		SmartDashboard.putNumber("Auto Step", m_autoStep);
		switch(m_autoStep) {
		// Reset Encoders
	    case 0:
	    	m_encoders.reset();
	    	m_autoStep = 1;
	    	m_ahrs.reset();
	    	break;
	    
	    // Drive Forward
	    case 1:
			m_drive.tankDrive(.6,-.6);
			SmartDashboard.putNumber("Distance", m_encoders.getDistance());
	    	if(m_encoders.getDistance() > 12) {
	    		m_drive.tankDrive(0,0);
	    		if(m_ethernet.switchLocation()[0] == "R".toCharArray()[0]) {
	    			m_autoStep = 2;
	    		}else {
	    			m_autoStep = 3;
	    		}   		
	    	}
	    	break;
		// Turn right ***FMS!!!!!***
		case 2:
			m_drive.tankDrive(.45,.45);
			if(m_ahrs.getYaw() > 27){
				m_drive.tankDrive(0,0);
				m_autoStep = 4;
			}
			break;
		// Turn Left ***FMS!!!!!***
		case 3:
			m_drive.tankDrive(-.45,-.45);
			if(m_ahrs.getYaw() < -27){
				m_drive.tankDrive(0,0);
				m_autoStep = 4;
				m_encoders.reset();
			}
			break;
		case 4:
			if(m_encoders.getDistance() < 80) {
				m_drive.tankDrive(.6,-.6 );
			}else {
				m_drive.tankDrive(.4,-.4 );
			}
			
			SmartDashboard.putNumber("Right Sensor position", m_encoders.getDistance());
			if(m_encoders.getDistance() > 112) {
				m_drive.tankDrive(0,0);
	    		if(m_ethernet.switchLocation()[0] == "R".toCharArray()[0]) {
	    			m_autoStep = 5;
	    		}else {
	    			m_autoStep = 6;
	    		} 
			}
			break;
		// Face Forward ***FMS!!!!!***
		case 5:
			m_drive.tankDrive(-.45,-.45 );
			if(m_ahrs.getYaw() < 1) {
				m_drive.tankDrive(0,0);
				m_autoStep = 7;
			}
			break;
			
		case 6:
			m_drive.tankDrive(.4,.4 );
			if(m_ahrs.getYaw() > -1) {
				m_drive.tankDrive(0,0);
				m_autoStep = 7;

			}
			break;
		// TRACK
		case 7:
			autoDrive(0.6, 2, m_ethernet.xFinal());
			
			if(m_ethernet.targetArea() > 11.3){
				m_autoStep = 8;
				m_encoders.reset();
			}
			break;	
			
			// BACK UP
		case 8:
			m_drive.tankDrive(-.6,.6);
			SmartDashboard.putNumber("Right Sensor position", m_encoders.getDistance());
	    	if(m_encoders.getDistance() < -11) {
	    		m_drive.tankDrive(0,0);
	    		if(m_ethernet.switchLocation()[0] == "R".toCharArray()[0]) {
	    			m_autoStep = 9;
	    		}else {
	    			m_autoStep = 10;
	    		}   		
	    	}
			break;		
			
			// TURN RIGHT
		case 9:
			m_drive.tankDrive(-.45,-.45 );
			if(m_ahrs.getYaw() < -85) {
				m_drive.tankDrive(0,0);
				m_autoStep = 11;
				m_encoders.reset();
			}
			break;
		
		// TURN Left
		case 10:
			m_drive.tankDrive(-.45,-.45 );
			if(m_ahrs.getYaw() < -85) {
				m_drive.tankDrive(0,0);
				m_autoStep = 12;
				m_encoders.reset();
			}
			break;
		// Drive Forward
		case 12:
			m_drive.tankDrive(.6,-.6);
			SmartDashboard.putNumber("Right Sensor position", m_encoders.getDistance());
	    	if(m_encoders.getDistance() > 70) {
	    		m_drive.tankDrive(0,0);
	    		m_encoders.reset();
	    		if(m_ethernet.switchLocation()[0] == "R".toCharArray()[0]) {
	    			m_autoStep = 13;
	    		}else {
	    			m_autoStep = 14;
	    		}   		
	    	}
			break;
		
		// Turn Right
		case 14:
			
			if(m_ahrs.getYaw() < -10) {
				m_drive.tankDrive(.5,.5 );
			}else {
				m_drive.tankDrive(.4,.4 );
			}
			if(m_ahrs.getYaw() > -1) {
				m_drive.tankDrive(0,0);
				m_encoders.reset();
				m_autoStep = 15;
			}
			break;
		
		case 15:
			if(m_ahrs.getYaw() < -1) {
				m_drive.tankDrive(.5,-.4 );
			}
			if(m_ahrs.getYaw() > 1) {
				m_drive.tankDrive(.4,-.5 );
			}
			if(m_ahrs.getYaw() < 1 && m_ahrs.getYaw() > -1) {
				m_drive.tankDrive(.4,-.4 );
			}
			
			SmartDashboard.putNumber("distance", m_encoders.getDistance());
			if(m_encoders.getDistance() > 150) {
				m_autoStep = 16;
			}
			
			break;
			
			
		case 16:
			break;
			
		}
		
	}

	// Code ran during the driver-operated period of the Match.
	public void teleopPeriodic() {
		// Drive
		if(m_ethernet.driveType() == "Tank") {
			m_drive.tankDrive(-m_joystick0.getRawAxis(1), m_joystick1.getRawAxis(1));
		}else {
			m_drive.arcadeDrive(m_joystick0.getRawAxis(0), -m_joystick0.getRawAxis(1));
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
		
		SmartDashboard.putNumber("Angle", m_ahrs.getYaw());     
	}

	// Class to manage Talon encoder feedback
	public class srxEncoders{	
		double cycles = 0;
		double reset = 0;
		double pulses_per_revolution = 0;
		double toInches = 40.743665431525205956834243423364;
		
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
}
