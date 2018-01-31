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

public class NetworkData {
	
	public DriverStation ds = DriverStation.getInstance();
	
	SendableChooser<String> autoModes = new SendableChooser<String>();
	SendableChooser<String> driveModes = new SendableChooser<String>();
	
	NetworkTable limelight = NetworkTableInstance.getDefault().getTable("limelight");
	NetworkTableEntry xFinal = limelight.getEntry("tx");
	NetworkTableEntry yFinal = limelight.getEntry("ty");
	NetworkTableEntry targetArea = limelight.getEntry("ta");
	NetworkTableEntry light = limelight.getEntry("ledMode");
	
	public NetworkData(){
		// Initalizes auto modes
		autoModes.addDefault("Switch: Center Start", "center");
		autoModes.addDefault("Scale: Left Start", "left");
		autoModes.addDefault("Scale: Right Start", "right");
		SmartDashboard.putData(autoModes);
		driveModes.addDefault("Arcade", "Arcade");
		driveModes.addDefault("Tank", "Tank");
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
	
	public String autoMode(){
		return (String) autoModes.getSelected();
	}
	
	public char[] switchLocation(){
		return (ds.getGameSpecificMessage()).toCharArray();
	}
	
	public String driveType() {
		return (String) driveModes.getSelected();
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


