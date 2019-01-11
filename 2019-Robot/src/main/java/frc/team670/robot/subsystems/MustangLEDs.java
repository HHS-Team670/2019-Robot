/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team670.robot.subsystems;

import java.io.DataOutputStream;
import java.io.IOException;
import java.net.ServerSocket;
import java.net.Socket;
import java.util.ArrayList;

import org.opencv.core.MatOfPoint;

import edu.wpi.first.wpilibj.DriverStation;

/**
 * Code for communicating LED strip changes to the RoboRIO. You will need to
 * modify this to interface with its superclass (LEDControl) so that it has an
 * enum. Based off Team 957's NeoPixel Library.
 */
public class MustangLEDs {

    /**
     * Represents the different patterns that can be displayed by the strip.
     */
    public enum LEDState{FORWARD, REVERSED, VISION_DRIVE, CLIMBING};

    private LEDState state;

    // Data Init. This is the data we end up passing to the arduino, so change it to match what it expects to read in.
	String alliance = "Invalid";
	String climbing = "false";
	String gearData = "false";
	
	int xFinal = 2;
	String data = "Invalid,false,false,2";
	
	// Socket Init
	ServerSocket serverSocket = null;
	// Thread init
	clientManagement client = new clientManagement();

	
	public void socketSetup(int server){ // Activates a single client once comms are established
		
		try {
			serverSocket = null;
			serverSocket = new ServerSocket(server);
			client.start();
		} catch (IOException e) {
			System.out.println("Unable to init Arduino server");
		}  
		
	}
	
	public void changeAlliance(boolean b) {
		if(b) {
			alliance = "blue   ";
		}
		else {
			alliance = "red    ";
		}
	}

	public void updateAlliance(){ // Updates the Alliance Color in data transmission
		
		DriverStation.Alliance color = DriverStation.getInstance().getAlliance();
		
		if(color == DriverStation.Alliance.Blue){
			alliance = "blue   ";
		}
		if(color == DriverStation.Alliance.Red){
			alliance = "red    ";
		}
		if(color == DriverStation.Alliance.Invalid){
			alliance = "invalid";	
		}
	}
	
	public void updateClimbingBoolean(boolean trigger){ // Updates if we are climbing
		if(trigger == true){
			climbing = "true ";
		}else{
			climbing = "false";
		}
	}
	
	public void updateGearData(boolean trigger){ // Updates if we have a gear
		if(trigger == true){
			gearData = "true ";
		}else{
			gearData = "false";
		}
	}
	
	public void update_xFinal(double acceptedXFinal){ // Updates xFinal in data
		if(acceptedXFinal > 10 && acceptedXFinal != -666){
			xFinal = 3;
		}else{
			if(acceptedXFinal < -10 && acceptedXFinal != -666){
				xFinal = 1;
			}else{
				xFinal = 2;
			}
		}
	}
	
	public class clientManagement extends Thread{
		
		public void run(){
			
			Socket socketClient = null;
			DataOutputStream output = null;
			ArrayList<MatOfPoint> gpgArray = new ArrayList<MatOfPoint>();
			while(socketClient == null){ // Attempts until client is found

				System.out.println("Socket Client null");
				try {
					socketClient = serverSocket.accept();
				} catch (IOException e) {
					socketClient = null;
				}
			}
			
			while(output == null){	// Attempts until Data Output Stream declared
				System.out.println("Output Null");
				try {
					output = new DataOutputStream(socketClient.getOutputStream());
					output.flush();
				} catch (IOException e) {
					output = null;
				}
			}
			
			while(true){
				
                // Compiles data into one nice neat string. Modify this to match with arduino-side stuff.
                // I suggest alliance + LEDState (forward, reversed, visiondrive, climb) + modifiers (turning, celebrating, etc.)
				data = alliance + "," + gearData + "," + climbing + "," + xFinal + "," + "false";
				
				System.out.println(data);
				
				// Sends data to the Arduino/Ethernet Shield
				try {
					output.writeUTF(data);
					output.flush();
				} catch (Exception e) {
					System.out.println("Unable to send Arduino data");
					
					try{
						output.close();
						socketClient.close();
						serverSocket = null;
						socketClient = null;
						socketSetup(5801);
					}catch(Exception E){
					}
					
					socketSetup(5801);
					return;
				}  
				
				// Sleeps to attempt to prevent a DOS overload
                try {
					Thread.sleep(250);
				} catch (Exception e) {
				}
  
			}
			
		}			
	}	

    /**
     * Returns the current state of the LED strip (the pattern currently being displayed)
     */
    public LEDState getCurrentState() {
        return state;
    }

    public void setLEDState(LEDState state) {
        this.state = state;
    }

}
