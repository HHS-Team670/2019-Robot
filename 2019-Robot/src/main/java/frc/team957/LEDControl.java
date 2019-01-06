/**
 * This class performs two functions: be a container class for an LED
 * strip and communicate to an Arduino over Ethernet using a standard
 * sketch that is easily downloaded. This allows full control of the LED
 * strip by the RoboRio, and not just have the Rio select preset patterns
 * pre-programed on the Arduino. This will hopefully make robot LEDs plug
 * and play with minimal, cheap hardware.
 * 
 * Caleb Shilling
 * FRC Team 957
**/

package frc.team957;

import java.io.DataOutputStream;
import java.io.IOException;
import java.net.ServerSocket;
import java.net.Socket;
import java.util.Arrays;

public class LEDControl {
	
	// Creates the ServerSockets
	ServerSocket server = null;
	
	// Byte array to convert 0-8 integers to their ASCII counterparts
	byte[] intConvert = 
		{48,49,50,51,52,53,54,55,56};
	
	byte byte1 = 48;
	
	// Byte array for the LED strip--holds all values for r, g, and b between 0 and 8--with 0 
	// being un-lit and 8 being 100% bright for that color
	byte[] strip = new byte[1];
	
	// Buffer array to update all pixels in an animation at once
	byte[] stripBuffer = new byte[1];
	
	// Length of LED strip
	int length = 0;
	
	// Constructor class: Initiates communication on Rio port 5810 and obtains a client
	public LEDControl(int length){
		
		while(server == null){
			try {server = new ServerSocket(5810);} catch (IOException e) {server = null;}
		}
		
		strip = new byte[(length*3)];
		stripBuffer = new byte[(length*3)];
		Arrays.fill(strip, byte1);
		Arrays.fill(stripBuffer, byte1);
		this.length = length;
		
		new Thread(new clientAquisition()).start();

	}
	
	// Main loop for detecting clients and giving them individual
	// threads for communication.
	private class clientAquisition implements Runnable{

		Socket client = null;
		
		public void run() {
			while (true) {
	            try {
	                client = server.accept();
	            } catch (IOException e) {
	                // Do nothing
	            }
	            // new thread for a client
	            new Thread(new clientThread(client)).start();
	        }
		}	
	}
	
	// Thread to manage incoming clients, and allows for blackouts/disconnects
	private class clientThread implements Runnable{
		
		// Network Communication Objects
		Socket clientSocket = null;
		DataOutputStream dout = null;
		
		// Constructor for a new client management thread
		public clientThread(Socket clientSocket){
			this.clientSocket = clientSocket;
			
			// Runs until a dataOutputStream can be established
			while(dout == null){
				try {
					dout = new DataOutputStream(clientSocket.getOutputStream());
				} catch (IOException e) {
					dout = null;
				}
			}
			
			System.out.println(
					"Client aquired and DOS established. Starting NeoPixel control.");
		}
		
		// Writes to the Arduino and closes the socket once done
		public void run() {
			boolean running = true;
			
			while(running == true){
				System.out.println(length);
				try {
					dout.write(strip);
					dout.flush();
				} catch (IOException e) {
					running = false;
					return;
				}
				try {
					Thread.sleep(50);} catch (InterruptedException e) {}
			}
			
			try {
				clientSocket.close();
			} catch (IOException e) {}	
		}
	}
	
	// Sets the color of an individual pixel, from 0-8
	public void setPixel(int n, int r, int g, int b){	
		
		if(n > length-1)
			return;
		
		if(r > 8){
			r = 8;
		}
		if(g > 8){
			g = 8;
		}
		if(b > 8){
			b = 8;
		}
		
		stripBuffer[(n*3)] = intConvert[r];
		stripBuffer[(n*3)+1] = intConvert[g];
		stripBuffer[(n*3)+2] = intConvert[b];	
	}
	
	// Fills the entire LED strip with a specific color
	public void fill(int r, int g, int b){
		
		if(r > 8){
			r = 8;
		}
		if(g > 8){
			g = 8;
		}
		if(b > 8){
			b = 8;
		}
		
		for(int n = 0; n < length; n++){
			stripBuffer[(n*3)] = intConvert[r];
			stripBuffer[(n*3)+1] = intConvert[g];
			stripBuffer[(n*3)+2] = intConvert[b];	
		}
	}
	
	// Copies the byte buffer array into the main array, thus allowing the
	// Arduino to update the strip with new settings
	public void show(){
		strip = stripBuffer;
	}
}