package frc.team670.robot.subsystems;

import java.io.DataOutputStream;
import java.io.IOException;
import java.net.ServerSocket;
import java.net.Socket;
import java.util.ArrayList;

import org.opencv.core.MatOfPoint;

import edu.wpi.first.wpilibj.DriverStation;

/**
 * Modify this class to collect data to send to the Arduino to control LEDs based on this year's game.
 * 
 * Credit to team 957 for the sample code.
 * 
 * @author ctchen, shaylandias
 */
public class MustangLEDs {

 	/**
     * Represents the different patterns that can be displayed by the strip.
     */
    public enum LEDState{FORWARD, REVERSED, VISION_DRIVE, CLIMBING};

    private LEDState state = LEDState.FORWARD;

	// Data Init. Change this to work with what we want to send to the Arduino.
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
			client = new clientManagement();
			client.start();
		} catch (IOException e) {
			System.out.println("Unable to init Arduino server");
		}  
		
	}
	
	/**
	 * Changes alliance. True = blue, False = red.
	 */
	public void changeAlliance(boolean b) {
		if(b) {
			alliance = "blue   ";
		}
		else {
			alliance = "red    ";
		}
	}

	/**
	 * Updates the alliance for the LEDs using the DriverStation given alliance
	 */
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
	
	/**
	 * @deprecated
	 */
	public void updateClimbingBoolean(boolean trigger){ // Updates if we are climbing
		if(trigger == true){
			climbing = "true ";
		}else{
			climbing = "false";
		}
	}
	
	/**
	 * @deprecated
	 */
	public void updateGearData(boolean trigger){ // Updates if we have a gear
		if(trigger == true){
			gearData = "true ";
		}else{
			gearData = "false";
		}
	}
	
	/**
	 * @deprecated
	 */
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
				
				// Compiles data into one nice neat string. Change this to match what we want to display and change the arduino to match.
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
	 /**
	  * Sets the LEDState to the given LEDState.
	  */
    public void setLEDState(LEDState state) {
        this.state = state;
    }

}