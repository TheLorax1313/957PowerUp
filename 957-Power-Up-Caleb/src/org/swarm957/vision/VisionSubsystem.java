package org.swarm957.vision;

import java.util.ArrayList;

import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import edu.wpi.cscore.CvSink;
import edu.wpi.cscore.CvSource;
import edu.wpi.cscore.MjpegServer;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.cscore.VideoMode;
import edu.wpi.first.wpilibj.Joystick;

public class VisionSubsystem {

	// Array for storing contour data
	ArrayList<MatOfPoint> contourData = new ArrayList<MatOfPoint>();
	
	// USB Cameras
	//UsbCamera cubeCamera = new UsbCamera("Cube Camera",0);
	//UsbCamera driveCamera = new UsbCamera("Drive Camera",1);
	//UsbCamera scaleCamera = new UsbCamera("scaleCamera", 2);
	
	// Camera Stream Servers
	//MjpegServer driveServer = new MjpegServer("Cube Camera Server",1180);
	//MjpegServer secondServer = new MjpegServer("Drive Camera Server",1181);

	
	// CV Sink+Source
	CvSink cubeSink = new CvSink("Cube Image Grabber");
	CvSource cubeImageSource = new CvSource
			("CV Image Source", VideoMode.PixelFormat.kMJPEG, 640, 480, 30);
	
	// MAT image file to process and overlay onto
	Mat cubeMat = new Mat();
	
	// Class to process MAT image
	CubeTracking ct = new CubeTracking();
	
	double hat = -1;
	
	// Variables for holding vision processing data
	double rectCenterX = 0;
	double rectCenterY = 0;
	double rectWidth = 0;
	double rectHeight = 0;
	
	public VisionSubsystem() {
		// Set resolutions of the cameras
		//cubeCamera.setResolution(160,120);
		//driveCamera.setResolution(160, 120);
		//cubeCamera.setFPS(30);
		//driveCamera.setFPS(30);
		//driveServer.setSource(driveCamera);
		//secondServer.setSource(cubeCamera);
		// Set sink source
		//cubeSink.setSource(cubeCamera);
		
		// Tell the cvStream to stream to CubeServer
		//cubeServer.setSource(cubeImageSource);
		
		// Launches a thread to process images from the cube camera
		//new Thread(new processCube()).start();
		
		// Launches camera switching thread
	}

	
}
