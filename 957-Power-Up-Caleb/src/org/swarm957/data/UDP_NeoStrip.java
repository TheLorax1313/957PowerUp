package org.swarm957.data;

import java.net.DatagramPacket;
import java.net.DatagramSocket;
import java.net.InetAddress;
import java.util.Arrays;

public class UDP_NeoStrip {
	
	// UDP Socket 
	DatagramSocket controllerSocket;
	
	// Default UDP packet
	DatagramPacket controllerPacket;
	
	// Byte buffer array to hold data to send to a controller
	byte[] stripData;
	
	// Default byte to fill the buffer with
	byte defaultByte = 56;
	
	// Holder for the IP of the controller
	InetAddress controllerIp;
	
	// String used to identify the LED strip/controller in console
	String identifier;
	
	
	// Constructor Class
	public UDP_NeoStrip(int length, String identifier, byte[] ipAddress) {
		
		// Attempt to:
		try {
			
			// Configure the UDP socket that outputs packets
			controllerSocket = new DatagramSocket();
			
			// Set the IP of the target controller
			controllerIp = InetAddress.getByAddress(ipAddress);
			
		} catch (Exception e) {} 
		
		// Set the public identifier string
		this.identifier = identifier;
		
		// Set the length of the LED strip by setting the buffer length
		stripData = new byte[length*3];
		
		// Fill the LED strip buffer with the default byte
		Arrays.fill(stripData, defaultByte);  
	}
	 
	// Updates the strip by sending a packet to the connected controller
	public void show() {
		
		// Set destination and content of the UDP packet
		controllerPacket = new DatagramPacket(stripData, stripData.length, controllerIp, 5810);
		
		// Attempt to:
		try {
			
			// Send the packet
			controllerSocket.send(controllerPacket);
			
		} catch (Exception e) {
			
			// If the packet fails to send, print an error message
			System.err.println(identifier+ ": Microcontrollers don't quit!");
		}
	}
	
	// Sets the color of a single pixel
	public void setPixel(int pixel, int r, int g, int b) {
		
		// Gets rid of negative values passed into the function
		r = Math.abs(r);
		g = Math.abs(g);
		b = Math.abs(b);
		
		// Sets a max brightness level of 8 for every color
		if(r > 8) {
			r = 8;
		}
		if(g > 8) {
			g = 8;
		}
		if(b > 8) {
			b = 8;
		}
		
		// Prevents an error if the desired pixel doesn't exist
		if(pixel > stripData.length) {
			System.err.println(identifier+": Interface attempted with a non-existent pixel!");
			return;
		}
		
		// Sets the pixel's color
		// The buffer is updated by casting the number as a byte and adding 48
		// 48 is ASCII for 0, 49 is 1, etc.
		stripData[pixel*3] = (byte)(r+48);
		stripData[(pixel*3)+1] = (byte)(g+48);
		stripData[(pixel*3)+2] = (byte)(b+48);
	}
	
	public void fill(int r, int g, int b) {
		// Gets rid of negative values passed into the function
		r = Math.abs(r);
		g = Math.abs(g);
		b = Math.abs(b);
		
		// Sets a max brightness level of 8 for every color
		if(r > 8) {
			r = 8;
		}
		if(g > 8) {
			g = 8;
		}
		if(b > 8) {
			b = 8;
		}
		
		for(int i = 0; i < stripData.length/3; i++) {
			stripData[i*3] = (byte)(r+48);
			stripData[(i*3)+1] = (byte)(g+48);
			stripData[(i*3)+2] = (byte)(b+48);
		}
	}
}