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

public class VisionSubsystem {

	// Enum values and associated array positions for cube tracking
	public enum Position{LEFT,RIGHT,TOP,BOTTOM,CENTER_X,CENTER_Y}
	int[] positionValues = new int[Position.values().length];
	
	// Array for storing contour data
	ArrayList<MatOfPoint> ctArray = new ArrayList<MatOfPoint>();
	Rect rect = new Rect();
	
	// USB Camera
	UsbCamera cubeCamera = new UsbCamera("Cube Camera",0);
	
	// Camera Stream Servers
	MjpegServer cubeServer = new MjpegServer("Cube Camera Server",1180);
	MjpegServer processedImage = new MjpegServer("GRIP Server", 1181);
	
	// CV Sink+Source
	CvSink cubeSink = new CvSink("Cube Image Grabber");
	CvSource cubeImageSource = new CvSource("CV Image Source", VideoMode.PixelFormat.kMJPEG, 640, 480, 30);
	
	// MAT image file to process and overlay onto
	Mat cubeMat = new Mat();
	
	// Class to process MAT image
	CubeTracking ct = new CubeTracking();
	
	// Frame rate of vision tracking camera
	long frametime = 0;
	
	// Variable to enable vision tracking
	boolean killThread = true;
	
	public VisionSubsystem() {
		// Set resolution of the camera
		cubeCamera.setResolution(160,120);
		cubeCamera.setFPS(30);
		
		// Set the server source to the camera
		cubeServer.setSource(cubeCamera);

		// Set sink source to the camera
		cubeSink.setSource(cubeCamera);
		
		// Tell the cvStream to stream to CubeServer
		processedImage.setSource(cubeImageSource);
		
		cubeCamera.setExposureManual(49);
		cubeCamera.setWhiteBalanceManual(4500);
		cubeCamera.setBrightness(43);
		
		// Launches a thread to process images from the cube camera when asked to at the start of Auto
		new Thread(new visionThread()).start();
	}
	
	// Returns a position value based on an ordinal
	public int getPosition(Position position) {
		return positionValues[position.ordinal()];
	}
	
	public void startThread() {
		killThread = false;
		new Thread(new visionThread()).start();
	}
	
	public void killThread() {
		killThread = true;
	}
	
	private class visionThread implements Runnable{

		@Override
		public void run() { 
			
			while(!killThread) {
					
				// Calculates frame rate and skips vision processing code if 0 (a camera issue)
				frametime = cubeSink.grabFrame(cubeMat);
				if(frametime == 0)
					continue;
				
				// Processes the image
				ct.process(cubeMat);
				
				// Obtains the array holding contour data
				ctArray = ct.filterContoursOutput();
				
				// Checks if the array holds any data whatsoever
				if(ctArray.size() > 0) {
					// Obtains the rectangle created by the image pipeline
					rect = Imgproc.boundingRect(ct.filterContoursOutput().get(0));
					// Assigns values based on the class enum
					// Rect.x or .y gives the lower left corner data, not the center
					positionValues[Position.LEFT.ordinal()] = (rect.x)-80;
					positionValues[Position.RIGHT.ordinal()] = (rect.x + rect.width)-80;
					positionValues[Position.TOP.ordinal()] = (rect.y + rect.height)-80;
					positionValues[Position.BOTTOM.ordinal()] = (rect.y)-80;
					positionValues[Position.CENTER_X.ordinal()] = (rect.x + (rect.width/2))-80;
					positionValues[Position.CENTER_Y.ordinal()] = (rect.y + (rect.height/2))-80;
					// Draws the rectangle onto the MAT
					Imgproc.rectangle(cubeMat, new Point(rect.x, rect.y), new Point(rect.x + rect.width, rect.y + rect.height), new Scalar(0,255,0),5);
				}else {
					// Defaults to 0 when nothing is seen
					positionValues[Position.LEFT.ordinal()] = 666;
					positionValues[Position.RIGHT.ordinal()] = 666;
					positionValues[Position.TOP.ordinal()] = 666;
					positionValues[Position.BOTTOM.ordinal()] = 666;
					positionValues[Position.CENTER_X.ordinal()] = 666;
					positionValues[Position.CENTER_Y.ordinal()] = 666;
				}
				
				// Updates the image stream on port 1181
				cubeImageSource.putFrame(cubeMat);
				
			}
		}	
	}
}
