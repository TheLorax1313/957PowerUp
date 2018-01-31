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

public class VisionControl {

	double m_cubeCenterX = 0;
	double m_cubeCenterY = 0;
	double m_cubeHeight = 0;
	double m_cubeWidth = 0;
	CubeFinder m_cubeFind = new CubeFinder();
	Mat m_cubeMat = new Mat();
	UsbCamera cubeCam = new UsbCamera("cube camera",0);
	CvSink m_cubeSink = new CvSink("sink");
	UsbCamera m_cubeCam = new UsbCamera("Front Camera",0);
	MjpegServer m_cubeServer = new MjpegServer("Front Stream", 1180);
	Mat m_mat = new Mat();
	CvSource m_cubeSource = new CvSource("CV Image Source", VideoMode.PixelFormat.kMJPEG, 640, 480, 30);     //Starts a CV Source to pass MATs into
    MjpegServer cvStream = new MjpegServer("CV Image Stream", 1181);        //Starts a camera server and sets it to stream what is passed into the CV Source, the processed image, most likely
	Thread m_cubeThread = new Thread(new cubeTrack());
    double[] returnArray;
    
    public VisionControl() {
    	m_cubeThread.setDaemon(true);
    	m_cubeThread.start();
    }
    
	class cubeTrack implements Runnable{
		public void run() {
			
			m_cubeCam.setResolution(160,120);
			m_cubeCam.setExposureManual(1);
			m_cubeCam.setBrightness(10);	
			m_cubeServer.setSource(m_cubeCam);
			m_cubeSink.setSource(m_cubeCam);
			cvStream.setSource(m_cubeSource);
			
			while(!Thread.currentThread().isInterrupted()) {
				long frameTime = m_cubeSink.grabFrame(m_cubeMat);       //Grabs latest frame from the CV sink and pastes it into the InputImage MAT for processing. Also grabs framerate of the stream
			     
			    if (frameTime == 0){    //Skips whatever below if the framerate = 0
			            return;
			    }
			     
			    m_cubeCenterX = 0;       //Resets all NetworkTables variables for sending vision data
			    m_cubeCenterY = 0;
			    m_cubeWidth = 0;
			    m_cubeHeight = 0;
			    m_cubeFind.process(m_cubeMat);
			    ArrayList<MatOfPoint> gpArray = m_cubeFind.filterContoursOutput();    //Grabs from the GripPipeline class the contour information
			    int gpLength = gpArray.size(); 
			     
			    if(gpLength > 0){       //1 contour seen
			        Rect r0 = Imgproc.boundingRect(m_cubeFind.filterContoursOutput().get(0));       //Creates a rectangle based on information from a contour
			        m_cubeCenterX = r0.x+(r0.width/2);       //Puts information into a variable that will later be sent to the Rio regarding bounding box center, width, and height of the contour
			        m_cubeCenterY = r0.y+(r0.height/2);
			        m_cubeWidth = r0.width;
			        m_cubeHeight = r0.height;
			        Imgproc.rectangle(m_cubeMat, new Point(r0.x,r0.y), new Point(r0.x+r0.width , r0.y+r0.height), new Scalar(0, 255, 0),5);        //Draws the rectangle on the image
			    }else{
			   	 	m_cubeCenterX = -9000+80;   //Sets networkTables variables to an easily recognizable error value
			   	 	m_cubeCenterY = -9000;
			        m_cubeWidth = -9000;
			        m_cubeHeight = -9000;
			    }
			    m_cubeCenterX = m_cubeCenterX - 80;  
			    m_cubeSource.putFrame(m_cubeMat);
			}
		}
	}
	
	public double[] cubeValues() {
		return new double[] {m_cubeCenterX, m_cubeCenterY, m_cubeHeight, m_cubeWidth};
	}
}
