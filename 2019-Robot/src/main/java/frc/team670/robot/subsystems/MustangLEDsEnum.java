package frc.team670.robot.subsystems;

import java.io.DataOutputStream;
import java.io.IOException;
import java.net.ServerSocket;
import java.net.Socket;
import java.util.ArrayList;


import org.opencv.core.MatOfPoint;

import edu.wpi.first.wpilibj.DriverStation;
enum RobotState{
	DEFAULT_STATE,FORWARD_DRIVE,REVERSE_DRIVE,VISION_LOCK,STILL_DRIVE,CLIMBING;
}
enum XFinal{
	XFINAL1,XFINAL2,XFINAL3;
}
enum AllianceColor{
	RED,BLUE,INVALID;
}
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
	
	private RobotState robotData=RobotState.DEFAULT_STATE;
	private XFinal xFinal=XFinal.XFINAL2;
	private AllianceColor allianceColor=AllianceColor.INVALID;
	String stateData;
	String allianceData;
	String xFinalData;
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
			allianceColor=AllianceColor.BLUE;
		}
		else {
			allianceColor=AllianceColor.RED;
		}
	}

	public void updateAlliance(){ // Updates the Alliance Color in data transmission
		
		DriverStation.Alliance color = DriverStation.getInstance().getAlliance();
		
		if(color == DriverStation.Alliance.Blue){
			allianceColor=AllianceColor.BLUE;
		}
		if(color == DriverStation.Alliance.Red){
			allianceColor=AllianceColor.RED;
		}
		if(color == DriverStation.Alliance.Invalid){
			allianceColor=AllianceColor.INVALID;	
		}
	}
	
	public void updateClimbingBoolean(boolean trigger){ // Updates if we are climbing
		if(trigger == true){
			robotData=RobotState.CLIMBING;
		}
	}
	
	public void updateVisionData(boolean trigger){ //updates if we lock onto a vision target
		if(trigger == true){
			robotData=RobotState.VISION_LOCK;
		}
	}
	public void updateForwardDrive(boolean trigger){//update if we are driving forward
		if(trigger=true){
			robotData=RobotState.FORWARD_DRIVE;
		}
		
	}
	public void updateReverseDrive(boolean trigger){
		if(trigger=true){
			robotData=RobotState.REVERSE_DRIVE;
		}
		
	}
	public void updateStillDrive(boolean trigger){
		if(trigger=true){
			robotData=RobotState.STILL_DRIVE;
		}
	}
	
	public void update_xFinal(double acceptedXFinal){ // Updates xFinal in data
		if(acceptedXFinal > 10 && acceptedXFinal != -666){
			xFinal=XFinal.XFINAL3;
		}else{
			if(acceptedXFinal < -10 && acceptedXFinal != -666){
				xFinal=XFinal.XFINAL1;
			}else{
				xFinal=XFinal.XFINAL2;
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
				
				//converts enum values for robot state into strings because .writeUTF is only compatible with strings
				if(robotData==RobotState.CLIMBING){
					stateData="5R";
				}
				if(robotData==RobotState.FORWARD_DRIVE){
					stateData="1R";
				}
				if(robotData==RobotState.REVERSE_DRIVE){
					stateData="2R";
				}
				if(robotData==RobotState.STILL_DRIVE){
					stateData="4R";
				}
				if(robotData==RobotState.VISION_LOCK){
					stateData="3R";
				}
				else{
					stateData="0R";
				}
				//converts enum values for alliance color into strings 
				if(allianceColor==AllianceColor.BLUE){
					allianceData="1A";
				}
				if(allianceColor==AllianceColor.RED){
					allianceData="2A";
				}
				else{
					allianceData="0A";
				}
				//converts enum values for xFinal into strings
				if(xFinal==XFinal.XFINAL1){
					xFinalData="1X";
				}
				if(xFinal==XFinal.XFINAL3){
					xFinalData="3X";
				}
				else{
					xFinalData="2X";
				}


	
				System.out.println(stateData+""+allianceData+""+xFinalData);
				
				// Sends data to the Arduino/Ethernet Shield
				try {
					output.writeUTF(stateData+""+allianceData+""+xFinalData);
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
  