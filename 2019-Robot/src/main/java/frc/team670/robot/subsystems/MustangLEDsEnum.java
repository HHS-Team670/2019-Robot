package frc.team670.robot.subsystems;

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

public class MustangLEDsEnum {
	// Data Init
	// String alliance = "Invalid";
	// String climbing = "false  ";
	// String forwardDrive="false  ";
	// String reverseDrive="false  ";
	// String still="false  ";
	// String visionLock="false  ";
	//String xFinal = "2";
	
	
	final String blueAlliance = "1A";
	final String redAlliance = "2A";
	String stateData="3R";
	String allianceData="0A";
	String xFinalData="2X";
	String data;

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
			System.out.println(e.toString());
		}  
		
	}
	
	public void changeAlliance(boolean b) {
		if(b) {
			allianceData=blueAlliance;
		}
		else {
			allianceData=redAlliance;
		}
	}

	public void updateAlliance(){ // Updates the Alliance Color in data transmission
		
		DriverStation.Alliance color = DriverStation.getInstance().getAlliance();
		
		if(color == DriverStation.Alliance.Blue){
			allianceData=blueAlliance;
		}
		if(color == DriverStation.Alliance.Red){
			allianceData=redAlliance;
		}
		if(color == DriverStation.Alliance.Invalid){
			allianceData="0A";	
		}
	}
	
	public void updateClimbingBoolean(boolean trigger){ // Updates if we are climbing
		if(trigger == true){
			stateData="4R";
		}
	}
	
	public void updateVisionData(boolean trigger){ //updates if we lock onto a vision target
		if(trigger == true){
			stateData="2R";
		}
	}
	public void updateForwardDrive(boolean trigger){//update if we are driving forward
		if(trigger=true){
			stateData="0R";
		}
		
	}
	public void updateReverseDrive(boolean trigger){
		if(trigger=true){
			stateData="1R";
		}
		
	}
	public void updateStillDrive(boolean trigger){
		if(trigger=true){
			stateData = "3R";
		}
	}
	
	public void update_xFinal(double acceptedXFinal){ // Updates xFinal in data
		if(acceptedXFinal > 10 && acceptedXFinal != -666){
			xFinalData="3X";
		}
		else{
			if(acceptedXFinal < -10 && acceptedXFinal != -666){
				xFinalData="1X";
			}
			else{
				xFinalData="2X";
			}
		}
	}
	
	public class clientManagement extends Thread{
		
		public void run(){
			
			while(true){
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
					} catch (IOException e) {
						output = null;
					}
				}
			
				
				System.out.println(stateData+""+allianceData+""+xFinalData);
				data = stateData + allianceData + xFinalData;
				
				// Sends data to the Arduino/Ethernet Shield
				try {
					output.writeUTF(data);
					output.flush();
					output.close();
				} catch (Exception e) {
					System.out.println("Unable to send Arduino data");
					System.out.println(e.toString());
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
  