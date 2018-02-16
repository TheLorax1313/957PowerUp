package org.swarm957.data;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * 
 * @author Caleb Shilling
 * This class manages all data transactions between the Driver Station and
 * the robot. Robot.java interacts with this class to begin data transactions
 * and the like.
 */

public class DataControlSubsystem {
	
	public DriverStation ds = DriverStation.getInstance();
	
	SendableChooser<Integer> autoModes = new SendableChooser<Integer>();
	SendableChooser<Integer> driveModes = new SendableChooser<Integer>();
	
	NetworkTable limelight = NetworkTableInstance.getDefault().getTable("limelight");
	NetworkTableEntry xFinal = limelight.getEntry("tx");
	NetworkTableEntry yFinal = limelight.getEntry("ty");
	NetworkTableEntry targetArea = limelight.getEntry("ta");
	NetworkTableEntry light = limelight.getEntry("ledMode");
	
	public DataControlSubsystem(){
		// Initalizes auto modes
		autoModes.setName("AUTO");
		driveModes.setName("DRIVE");
		autoModes.addDefault("Switch: Center Start", 0);
		autoModes.addDefault("Scale: Left Start", 1);
		autoModes.addDefault("Scale: Right Start", 2);
		autoModes.addDefault("Cross Auto Line", 3);
		SmartDashboard.putData(autoModes);
		driveModes.addDefault("Arcade", 1);
		driveModes.addDefault("Tank", 2);
		SmartDashboard.putData(driveModes);
	}
	
	public String alliance(){
		if(ds.getAlliance() == DriverStation.Alliance.Blue){
			return "blue";
		}
		if(ds.getAlliance() == DriverStation.Alliance.Red){
			return "red";	
		}
		if(ds.getAlliance() == DriverStation.Alliance.Invalid){
			return "invalid";
		}
		return "invalid";
	}
	
	public int autoMode(){
		return autoModes.getSelected();
	}
	
	public char[] switchLocation(){
		return (ds.getGameSpecificMessage()).toCharArray();
	}
	
	public int driveType() {
		return driveModes.getSelected();
	}
	
	public double xFinal() {
		return xFinal.getDouble(0);
	}
	
	public double yFinal() {
		return yFinal.getDouble(0);
	}
	
	public double targetArea() {
		return targetArea.getDouble(0);
	}
	
	public void lightOn(boolean input) {
		if(input) {
			light.setDouble(0);
		}else {
			light.setDouble(1);
		}
	}
}


