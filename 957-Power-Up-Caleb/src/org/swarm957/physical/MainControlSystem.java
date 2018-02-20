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
	
	// Begin Arduino Communication
	//NeoStrip m_leds = new NeoStrip(30);
	
	// Enable Encoder tracking
	srxEncoders m_encoders = new srxEncoders(256,2);
	
	// Start Gyro
	AHRS m_ahrs = new AHRS(I2C.Port.kMXP);
	
	double RPM = 0;
	
	// Begin Elevator control
	ElevatorSubsystem m_elevator = new ElevatorSubsystem();
	
	// Begin Arm control
	ArmSubsystem m_arm = new ArmSubsystem(1);
	
	SensorSubsystem m_sensors = new SensorSubsystem();
	
	Thread m_rpm = new Thread(new calculateRPM());
	
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
	
	Thread m_updateAuto = new Thread(new obtainAutoData());
	// Autonomus Switch Statement stage
	int m_autoStep = 0;
	int autoCount = 0;
	// String to hold the Auto mode
	int m_autoMode = 0;
	double speed = 0;
	// Character array to hold switch/scale locations
	char[] m_gameData = new char[3];
	
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
		
		/*Set Talon ramping
		setCurrentLimits(m_r1,13,15,50);
		setCurrentLimits(m_r2,13,15,50);
		setCurrentLimits(m_r3,13,15,50);
		setCurrentLimits(m_l1,13,15,50);
		setCurrentLimits(m_l2,13,15,50);
		setCurrentLimits(m_l3,13,15,50);
		*/
		
		m_updateAuto.setDaemon(true);
		m_updateAuto.start();
		m_rpm.setDaemon(true);
		m_rpm.start();

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
		SmartDashboard.putNumber("m_autoMode", m_autoMode);
		// SWITCH CENTER AUTO
		if(m_autoMode == 0) {
			switch(m_autoStep) {
			case 0:
				m_elevator.setLevel(1);
				if(m_encoders.getDistance() > 6) {
					m_drive.tankDrive(0, 0);
					m_autoStep = 1;
				}else {
					driveStraight(0.6,0);
				}
				
				break;
				
			case 1:
				
				if(m_encoders.getDistance() > 11) {
					m_autoStep = 2;
				}
				break;
				
			case 2:
				
				if(m_gameData[0] == "L".toCharArray()[0]) {
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
				
				if(m_gameData[0] == "R".toCharArray()[0]) {
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
					if(m_gameData[0] == "R".toCharArray()[0])
					driveStraight(speed,45);
					if(m_gameData[0] == "L".toCharArray()[0])
						driveStraight(speed,-45);
				}
				break;
				
			case 4:
				
				if(m_gameData[0] == "L".toCharArray()[0]) {
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
				
				if(m_gameData[0] == "R".toCharArray()[0]) {
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
				
			case 5:
				m_elevator.setLevel(2);
				autoCount++;
				autoDrive(0.6,2,m_ethernet.xFinal());
				if(m_ethernet.targetArea() > 4.1 || (RPM < 2 && autoCount > 25)) {
					m_drive.tankDrive(0, 0);
					m_autoStep = 6;
					autoCount = 0;
				}
			break;
			
			case 6:
				
				m_arm.eject();
				autoCount++;
				
				if(autoCount > 50) {
					m_arm.stop();
					m_autoStep = 7;
					autoCount = 0;
				}
				break;
				
			case 7:
				
				autoCount++;
				
				if(autoCount > 100) {
					m_drive.tankDrive(0, 0);
					m_autoStep = 8;
					m_elevator.setLevel(0);
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
			if(m_gameData[1] == "R".toCharArray()[0]) {
				switch(m_autoStep) {
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
					// 304 inches
					
					if( m_encoders.getDistance() > 384) {
						m_drive.tankDrive(0, 0);
					}
					
					break;
					
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
				
				case 2:
					autoCount++;
					m_elevator.setLevel(1);
					driveStraight(-0.55, -90);
					if(RPM < 2 && autoCount > 25) {
						m_autoStep = 3;
						autoCount = 0;
					}
					
					break;
				case 3:
					autoCount++;
					m_elevator.setLevel(5);
					if(autoCount > 100) {
						m_autoStep = 4;
						m_encoders.reset();
					}
					break;
					
				case 4:
					driveStraight(0.5, -90);
					if(m_encoders.getDistance() > 24) {
						m_drive.tankDrive(0, 0);
						m_autoStep = 5;
						autoCount = 0;
					}
					break;
				case 5:
					m_arm.eject();
					autoCount++;
					
					if(autoCount > 50) {
						m_arm.stop();
						m_autoStep = 6;
						autoCount = 0;
					}
					break;
					
				case 6:
					driveStraight(-0.5, -90);
					autoCount++;
					
					if(autoCount > 100 || (RPM < 2 && autoCount > 25)) {
						m_drive.tankDrive(0,0);
						m_autoStep = 7;
						autoCount = 0;
					}
					break;
					
				case 7:
					
					m_elevator.setLevel(0);
					break;
				case 8:
					break;
				}
			}else {
				switch(m_autoStep) {
				case 0:
					
					if(m_encoders.getDistance() > 70) {
						speed = 0.4;
					}else {
						speed = 0.7;
					}
					
					
					if(m_encoders.getDistance() > 101) {
						m_drive.tankDrive(0, 0);
						//m_autoStep = 1;
					}else {
						driveStraight(speed,0);
					}
					
					break;
					
				case 1:
					
					break;
				}
			}	
		}
		
		// SCALE Left AUTO
		if(m_autoMode == 1) {
			if(m_gameData[1] == "L".toCharArray()[0]) {
				switch(m_autoStep) {
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
					// 304 inches
					
					if( m_encoders.getDistance() > 384) {
						m_drive.tankDrive(0, 0);
					}
					
					break;
					
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
				
				case 2:
					autoCount++;
					m_elevator.setLevel(1);
					driveStraight(-0.55, -90);
					if(RPM < 2 && autoCount > 25) {
						m_autoStep = 3;
						autoCount = 0;
					}
					
					break;
				case 3:
					autoCount++;
					m_elevator.setLevel(5);
					if(autoCount > 100) {
						m_autoStep = 4;
						m_encoders.reset();
					}
					break;
					
				case 4:
					driveStraight(0.5, -90);
					if(m_encoders.getDistance() > 24) {
						m_drive.tankDrive(0, 0);
						m_autoStep = 5;
						autoCount = 0;
					}
					break;
				case 5:
					m_arm.eject();
					autoCount++;
					
					if(autoCount > 50) {
						m_arm.stop();
						m_autoStep = 6;
						autoCount = 0;
					}
					break;
					
				case 6:
					driveStraight(-0.5, -90);
					autoCount++;
					
					if(autoCount > 100 || (RPM < 2 && autoCount > 25)) {
						m_drive.tankDrive(0,0);
						m_autoStep = 7;
						autoCount = 0;
					}
					break;
					
				case 7:
					
					m_elevator.setLevel(0);
					break;
				case 8:
					break;
				}
			}else {
				switch(m_autoStep) {
				case 0:
					
					if(m_encoders.getDistance() > 70) {
						speed = 0.4;
					}else {
						speed = 0.7;
					}
					
					
					if(m_encoders.getDistance() > 101) {
						m_drive.tankDrive(0, 0);
						//m_autoStep = 1;
					}else {
						driveStraight(speed,0);
					}
					
					break;
					
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
			
			case 0:
				m_elevator.setLevel(1);
				if(m_encoders.getDistance() > 125) {
					speed = 0.45;
				}else {
					speed = 0.7;
				}
				
				
				if(m_encoders.getDistance() > 155) {
					m_drive.tankDrive(0, 0);
					if(m_gameData[0] == "L".toCharArray()[0]) {
						m_autoStep = 1;
					}
					
				}else {
					driveStraight(speed,0);
				}
				break;
				
			case 1:
				if(m_ahrs.getYaw() > 80) {
					speed = 0.45;
				}else {
					speed = 0.7;
				}
				m_elevator.setLevel(2);
				if(m_ahrs.getYaw() > 88) {
					m_drive.tankDrive(0, 0);
					m_autoStep = 2;
					m_encoders.reset();
					autoCount = 0;
				}
				m_drive.tankDrive(speed,-speed);
				break;
				
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
				
			case 3:
				m_arm.eject();
				autoCount++;
				
				if(autoCount > 50) {
					m_arm.stop();
					m_autoStep = 4;
					autoCount = 0;
				}
				break;
				
			case 4:
				driveStraight(-0.6, 90);
				autoCount++;
				
				if(autoCount > 100 || (RPM < 2 && autoCount > 25)) {
					m_drive.tankDrive(0,0);
					m_autoStep = 5;
					autoCount = 0;
				}
				break;
				
			case 5:
				m_elevator.setLevel(0);
				break;
			}
		}
		
		// SWITCH RIGHT OR CROSS THE LINE
		if(m_autoMode == 5) {
			
			switch(m_autoStep) {
			
			case 0:
				m_elevator.setLevel(1);
				if(m_encoders.getDistance() > 125) {
					speed = 0.45;
				}else {
					speed = 0.7;
				}
				
				
				if(m_encoders.getDistance() > 155) {
					m_drive.tankDrive(0, 0);
					if(m_gameData[0] == "R".toCharArray()[0]) {
						m_autoStep = 1;
					}
					
				}else {
					driveStraight(speed,0);
				}
				break;
				
			case 1:
				if(m_ahrs.getYaw() < -80) {
					speed = 0.45;
				}else {
					speed = 0.7;
				}
				m_elevator.setLevel(2);
				if(m_ahrs.getYaw() < -88) {
					m_drive.tankDrive(0, 0);
					m_autoStep = 2;
					m_encoders.reset();
					autoCount = 0;
				}
				m_drive.tankDrive(-speed,speed);
				break;
				
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
				
			case 3:
				m_arm.eject();
				autoCount++;
				
				if(autoCount > 50) {
					m_arm.stop();
					m_autoStep = 4;
					autoCount = 0;
				}
				break;
				
			case 4:
				driveStraight(-0.6, 90);
				autoCount++;
				
				if(autoCount > 100 || (RPM < 2 && autoCount > 25)) {
					m_drive.tankDrive(0,0);
					m_autoStep = 5;
					autoCount = 0;
				}
				break;
				
			case 5:
				m_elevator.setLevel(0);
				break;
			}
			
		}
		
	}
	
	public void disabledPeriodic() {
		//m_elevator.setLevel(0);
	}
	
	public void teleopInit() {
		//m_elevator.setLevel(0);
	}

	// Code ran during the driver-operated period of the Match.
	public void teleopPeriodic() {
		// Drive
		//if(m_ethernet.driveType() == 1) {
		//	m_drive.tankDrive(-m_joystick0.getRawAxis(1), m_joystick1.getRawAxis(1));
		//}else {
			m_drive.arcadeDrive(-m_joystick0.getRawAxis(1)*m_elevator.percent(), m_joystick0.getRawAxis(2)*m_elevator.returnTurn());
		//}
		
		if(m_joystick1.getRawButton(5)) {
			m_elevator.setLevel(0);
		}
		if(m_joystick1.getRawButton(6)) {
			m_elevator.setLevel(1);
		}
		if(m_joystick1.getRawButton(1)) {
			m_elevator.setLevel(2);
		}
		if(m_joystick1.getRawButton(2)) {
			m_elevator.setLevel(3);
		}
		if(m_joystick1.getRawButton(3)) {
			m_elevator.setLevel(4);
		}
		if(m_joystick1.getRawButton(4)) {
			m_elevator.setLevel(5);
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
		
		//if(m_sensors.getDistance() > 0.8 && m_elevator.getLevel() == 0) {
		//	m_elevator.setLevel(1);
		//}
		
		
	
	}
	
	// Method for controlling LED Lights (5v) and Dashboard
	// readings.
	public void robotPeriodic(){

		SmartDashboard.putNumber("Angle", m_ahrs.getYaw());     
		SmartDashboard.putNumber("Auto Step", m_autoStep);
		SmartDashboard.putNumber("voltage", m_sensors.getDistance());
		SmartDashboard.putBoolean("lowSwitch", m_elevator.get());
		SmartDashboard.putNumber("distance", m_encoders.getDistance());
		SmartDashboard.putNumber("Pitch: ", m_ahrs.getPitch());
		SmartDashboard.putNumber("DRIVE RPM: ", RPM);
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
	
	public class obtainAutoData implements Runnable{
		public void run() {
			while(true) {
				m_gameData = m_ethernet.switchLocation();
				SmartDashboard.putString("Game Data", m_gameData.toString());
			}
		}
	}
	
	public class calculateRPM implements Runnable{
		
		double encoderOffset = 0;
		
		public void run() {
			while(true) {
				encoderOffset = m_encoders.getRaw();
				try {
					Thread.sleep(100);
				} catch (InterruptedException e) {}
				RPM = (Math.abs(m_encoders.getRaw()-encoderOffset)/1024)*600;
				
			}
		}
		
		
	}

}
