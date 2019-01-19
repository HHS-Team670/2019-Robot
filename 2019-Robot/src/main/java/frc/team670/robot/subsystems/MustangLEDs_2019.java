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

public class MustangLEDs_2019 {

	// string representations for alliances
	final byte[] blueAlliance = { '1', 'A' };
	final byte[] redAlliance = { '2', 'A' };
	// final String invalidAlliance="3A";

	// string representations for robot states
	final byte[] climbing = { '4', 'R' };
	final byte[] visionLock = { '2', 'R' };
	final byte[] forwardDrive = { '0', 'R' };
	final byte[] reverseDrive = { '1', 'R' };
	final byte[] stillDrive = { '3', 'R' };
	final byte[] visionLockLost={'5','R'};
	// variables for data which will be sent over server
	byte[] stateData = stillDrive;
	byte[] allianceData = blueAlliance;

	// string representations for xFinal
	// final String xFinalLeft="1X";
	// final String xFinalCenter="2X";
	// final String xFinalRight="3X";
	// String xFinalData=xFinalCenter;

	// Socket Init
	ServerSocket serverSocket = null;
	// Thread init
	clientManagement client = new clientManagement();

	public void socketSetup(int server) { // Activates a single client once comms are established

		try {
			serverSocket = null;
			serverSocket = new ServerSocket(server);
			client.start();
		} catch (IOException e) {
			System.out.println("Unable to init Arduino server");
			System.out.println(e.toString());
		}

	}

	public void changeAlliance(boolean b) { // true for blue, false for red
		if (b) {
			allianceData = blueAlliance;
		} else {
			allianceData = redAlliance;
		}

	}

	// public void updateAlliance(){ // Updates the Alliance Color in data
	// transmission

	// DriverStation.Alliance color = DriverStation.getInstance().getAlliance();

	// if(color == DriverStation.Alliance.Blue){
	// allianceData=blueAlliance;
	// }
	// if(color == DriverStation.Alliance.Red){
	// allianceData=redAlliance;
	// }
	// if(color == DriverStation.Alliance.Invalid){
	// allianceData=invalidAlliance;
	// }
	// }

	public void setClimbingData(boolean trigger) { // Updates if we are climbing
		if (trigger == true) {
			stateData = climbing;
		}
	}

	public void setVisionData(boolean trigger) { // updates if we lock onto a vision target
		if (trigger == true) {
			stateData = visionLock;
		}
	}
	public void visionLockLost(boolean trigger){
		if(trigger == true){
			stateData = visionLockLost;
		}
	}
	public void setForwardData(boolean trigger) {// updates if we are driving forward
		if (trigger = true) {
			stateData = forwardDrive;
		}

	}

	public void setReverseData(boolean trigger) {// updates if we are driving in reverse
		if (trigger = true) {
			stateData = reverseDrive;
		}

	}

	public void setStillDrive(boolean trigger) {// updates it robot is not moving
		if (trigger = true) {
			stateData = stillDrive;
		}
	}

	// public void update_xFinal(double acceptedXFinal){ // Updates xFinal in data
	// if(acceptedXFinal > 10 && acceptedXFinal != -666){
	// xFinalData=xFinalRight;
	// }
	// else{
	// if(acceptedXFinal < -10 && acceptedXFinal != -666){
	// xFinalData=xFinalLeft;
	// }
	// else{
	// xFinalData=xFinalCenter;
	// }
	// }
	// }

	public class clientManagement extends Thread {

		public void run() {

			while (true) {
				Socket socketClient = null;
				DataOutputStream output = null;
				ArrayList<MatOfPoint> gpgArray = new ArrayList<MatOfPoint>();
				while (socketClient == null) { // Attempts until client is found

					System.out.println("Socket Client null");
					try {
						socketClient = serverSocket.accept();
					} catch (IOException e) {
						socketClient = null;
					}
				}

				while (output == null) { // Attempts until Data Output Stream declared
					System.out.println("Output Null");
					try {
						output = new DataOutputStream(socketClient.getOutputStream());
					} catch (IOException e) {
						output = null;
					}
				}

				System.out.println(stateData + "" + allianceData);

				// Sends data to the Arduino/Ethernet Shield
				try {
					output.write(stateData);
					output.write(allianceData);
					// output.write(toSend);
					// output.write(stateData); // data.getBytes());
					// output.write(allianceData);
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