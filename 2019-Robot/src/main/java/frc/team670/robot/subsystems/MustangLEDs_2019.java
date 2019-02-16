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

	// final byte[] runningAllianceColors = {'0','L'};
	// final byte[] solidGreen = {'1','L'};
	// final byte[] solidRed = {'2','L'};
	// final byte[] solidPurple = {'3','L'};
	// final byte[] GreenLights = {'4','L'};
	// final byte[] Strobe = {'5','L'};
	// final byte[] randomStrobe = {'6','L'};
	// final byte[] bounceBackground = {'7','L'};
	// final byte[] cylonBounce = {'8','L'};
	// final byte[] rainbow = {'9','L'};
	

	// variables for data which will be sent over server
	byte[] stateData = stillDrive;
	byte[] allianceData = blueAlliance;
	// byte[] lightShowData = runningAllianceColors;

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

	public void changeAlliance(boolean b) { //sets alliance data, is the first byte array sent over server
		if (b) {
			allianceData = blueAlliance;
		} else {
			allianceData = redAlliance;
		}

	}
	public void setClimbingData(boolean trigger) { // Updates if we are climbing
		if (trigger == true) {
			stateData = climbing;
		} else {
			setStillDrive(true);
		}
	}

	public void setVisionData(boolean trigger) { // updates if we lock onto a vision target
		if (trigger == true) {
			stateData = visionLock;
		} else {
			setStillDrive(true);
		}
	}

	public void setForwardData(boolean trigger) {// updates if we are driving forward

		if (trigger = true) {
			stateData = forwardDrive;
		} else {
			setStillDrive(true);
		}
	}

	public void setReverseData(boolean trigger) {// updates if we are driving in reverse
		if (trigger = true) {
			stateData = reverseDrive;
		} else {
			setStillDrive(true);
		}
	}

	public void setStillDrive(boolean trigger) {// updates it robot is not moving
		if (trigger = true) {
			stateData = stillDrive;
		}
	}

	public class clientManagement extends Thread {

		public void run() {

			while (true) {
				Socket socketClient = null;
				DataOutputStream output = null;
				ArrayList<MatOfPoint> gpgArray = new ArrayList<MatOfPoint>();
				while (socketClient == null) { // Attempts until client is found

					System.out.println("Waiting for LED client connection");
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

				System.out.println(allianceData + "" + stateData);

				// Sends data to the Arduino/Ethernet Shield
				try {
					output.write(allianceData);
					output.write(stateData);
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