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
	UsbCamera cubeCamera = new UsbCamera("Cube Camera",0);
	UsbCamera driveCamera = new UsbCamera("Drive Camera",1);
	UsbCamera scaleCamera = new UsbCamera("scaleCamera", 2);
	
	// Camera Stream Servers
	MjpegServer driveServer = new MjpegServer("Cube Camera Server",1180);
	MjpegServer secondServer = new MjpegServer("Drive Camera Server",1181);

	
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
		cubeCamera.setResolution(160,120);
		driveCamera.setResolution(160, 120);
		cubeCamera.setFPS(30);
		driveCamera.setFPS(30);
		driveServer.setSource(driveCamera);
		secondServer.setSource(cubeCamera);
		// Set sink source
		//cubeSink.setSource(cubeCamera);
		
		// Tell the cvStream to stream to CubeServer
		//cubeServer.setSource(cubeImageSource);
		
		// Launches a thread to process images from the cube camera
		//new Thread(new processCube()).start();
		
		// Launches camera switching thread
	}
	
	public void sendHat(Joystick joy) {
		hat = joy.getPOV(0);
		if(hat == 90) {
			secondServer.setSource(cubeCamera);
		}
		if(hat == 270) {
			secondServer.setSource(scaleCamera);
		}
	}
	
	
	class processCube implements Runnable{

		public void run() { while(true) {
			
			// Sets cube finding variables to nil
			rectCenterX = 0;
            rectCenterY = 0;
            rectHeight = 0;
            rectWidth = 0;
            
            if (cubeSink.grabFrame(cubeMat) == 0)    //Skips whatever below if the framerate = 0
                continue;
                
            // Process the Mat
            ct.process(cubeMat);
        
            contourData = ct.filterContoursOutput();    //Grabs from the GripPipeline class the contour information
            int gpLength = contourData.size();          //Asks from gpArray how many contours were seen
		
            if(gpLength > 0){       //1 contour seen
                Rect r0 = Imgproc.boundingRect(ct.filterContoursOutput().get(0));       //Creates a rectangle based on information from a contour
                rectCenterX = r0.x+(r0.width/2);       //Puts information into a variable that will later be sent to the Rio regarding bounding box center, width, and height of the contour
                rectCenterY = r0.y+(r0.height/2);
                rectWidth = r0.width;
                rectHeight = r0.height;
                Imgproc.rectangle(cubeMat, new Point(r0.x,r0.y), new Point(r0.x+r0.width , r0.y+r0.height), new Scalar(0, 255, 0),5);        //Draws the rectangle on the image itself
            }else{
                rectCenterX = -9000;   //Sets networkTables variables to an easily recognizable error value
                rectCenterY = -9000;
                rectWidth = -9000;
                rectHeight = -9000;
        	}
            
            cubeImageSource.putFrame(cubeMat);
		}}
	}
}
