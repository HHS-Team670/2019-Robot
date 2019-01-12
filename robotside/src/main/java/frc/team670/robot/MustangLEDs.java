package frc.team670.robot;

import java.io.DataOutputStream;
import java.io.IOException;
import java.net.ServerSocket;
import java.net.Socket;
import java.util.ArrayList;

import org.opencv.core.MatOfPoint;

import edu.wpi.first.wpilibj.DriverStation;

/**
 * Modify this class to collect data to send based on this year's game.
 * 
 * Credit to team 957 for the sample code.
 * 
 * @author ctchen, shaylandias
 */
public class MustangLEDs {
	// Data Init
	String alliance = "Invalid";
	String climbing = "false  ";
	String forwardDrive="false  ";
	String reverseDrive="false  ";
	String still="false  ";
	String visionLock="false  ";
	int xFinal = 2;
	String data = "Invalid,false,false,";
	
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
			climbing = "true   ";
		}else{
			climbing = "false  ";
		}
	}
	
	public void updateVisionData(boolean trigger){ //updates if we lock onto a vision target
		if(trigger == true){
			visionLock = "true   ";
		}else{
			visionLock = "false  ";
		}
	}
	public void updateForwardDrive(boolean trigger){//update if we are driving forward
		if(trigger=true){
			forwardDrive="true   ";
		}
		else{
			forwardDrive="false  ";
		}
	}
	public void updateReverseDrive(boolean trigger){
		if(trigger=true){
			reverseDrive="true   ";
		}
		else{
			reverseDrive="false  ";
		}
	}
	public void updateStillDrive(boolean trigger){
		if(trigger=true){
			still="true   ";
		}
		else{
			still="false  ";
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
				
				// Compiles data into one nice neat string
				
				data = alliance + "," + climbing + "," + forwardDrive+ "," +reverseDrive + "," + still+ "," +visionLock+","+xFinal;
	
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
}