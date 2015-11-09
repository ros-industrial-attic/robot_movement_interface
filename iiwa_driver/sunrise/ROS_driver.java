/*
	Copyright 2015 Fraunhofer IPA

	Licensed under the Apache License, Version 2.0 (the "License");
	you may not use this file except in compliance with the License.
	You may obtain a copy of the License at

	   http://www.apache.org/licenses/LICENSE-2.0

	Unless required by applicable law or agreed to in writing, software
	distributed under the License is distributed on an "AS IS" BASIS,
	WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
	See the License for the specific language governing permissions and
	limitations under the License.
	
	Contributors:
	- Pablo Quilez Velilla - First release
*/

// ----------------------------------------------------------------------------
// TCP Driver for ROS
// ----------------------------------------------------------------------------
// Important notice: Clicking stop button in the pad controller produces an 
// exception which is catched with Exception e
// ----------------------------------------------------------------------------
// Incoming messages and responses (don't forget /r/n, or /n)
// Units: mm, rad, % for lin, ptp
// ----------------------------------------------------------------------------
// If SmartServo and DirectServo are not available (Kuka Connectivity package is required)
// it is possible to disallow those functions which use them, such as smart ptp move
// TODO: split into two versions of this program with/without connectivity package
// ----------------------------------------------------------------------------

package application;

import static com.kuka.roboticsAPI.motionModel.BasicMotions.lin;
import static com.kuka.roboticsAPI.motionModel.BasicMotions.positionHold;
import static com.kuka.roboticsAPI.motionModel.BasicMotions.ptp;

import java.io.BufferedReader;
import java.io.IOException;
import java.io.InputStreamReader;
import java.io.PrintWriter;
import java.net.ServerSocket;
import java.net.Socket;
import java.util.concurrent.TimeUnit;

import com.kuka.roboticsAPI.applicationModel.RoboticsAPIApplication;
import com.kuka.roboticsAPI.conditionModel.ForceComponentCondition;
import com.kuka.roboticsAPI.controllerModel.Controller;
import com.kuka.roboticsAPI.controllerModel.sunrise.SunriseController;
import com.kuka.roboticsAPI.deviceModel.JointPosition;
import com.kuka.roboticsAPI.deviceModel.LBR;
import com.kuka.roboticsAPI.deviceModel.LBRE1Redundancy;
import com.kuka.roboticsAPI.geometricModel.CartDOF;
import com.kuka.roboticsAPI.geometricModel.Frame;
import com.kuka.roboticsAPI.geometricModel.Tool;
import com.kuka.roboticsAPI.geometricModel.World;
import com.kuka.roboticsAPI.geometricModel.math.CoordinateAxis;
import com.kuka.roboticsAPI.geometricModel.math.Transformation;
import com.kuka.roboticsAPI.geometricModel.math.Vector;
import com.kuka.roboticsAPI.motionModel.DirectServo;
import com.kuka.roboticsAPI.motionModel.IDirectServoRuntime;
import com.kuka.roboticsAPI.motionModel.IMotionContainer;
import com.kuka.roboticsAPI.motionModel.ISmartServoRuntime;
import com.kuka.roboticsAPI.motionModel.PositionHold;
import com.kuka.roboticsAPI.motionModel.SmartServo;
import com.kuka.roboticsAPI.motionModel.controlModeModel.CartesianImpedanceControlMode;
import com.kuka.roboticsAPI.motionModel.controlModeModel.JointImpedanceControlMode;

// The TCP driver as robot application class
// Port and simulation options can be configured through the Process data directly in the pad
public class ROS_driver_v2 extends RoboticsAPIApplication {
	
	public enum RobotMode {unknown, normal, impedance, smart, direct} // robot behaviour differs between modes
	
	RobotMode lastRobotMode = RobotMode.unknown; // Stores current robot move
	
	private Controller controller;
	private LBR robot;
	
	private IMotionContainer motionContainer = null;
	private Tool tool;

	ServerSocket serverSocket = null;
	Socket clientSocket = null;
	
	SmartServo smartServo = null;
	ISmartServoRuntime smartMotion = null;
	
	DirectServo directServo = null;
	IDirectServoRuntime directMotion = null;
	
	JointPosition simulation_joints = null;
	
	public static void main(String[] args) {
		ROS_driver_v2 app = new ROS_driver_v2();
		app.runApplication();
	}

	@Override
	public void dispose(){
		System.out.println("Stoping motion... ");
		if (motionContainer != null) motionContainer.cancel();
		System.out.println("Closing the sockets... ");
		try { clientSocket.close(); } catch (Exception e) { }
		try { serverSocket.close(); } catch (Exception e) { }
	}
	
	public void initialize() {
		controller = (SunriseController) getContext().getDefaultController();
		robot = (LBR) getRobot(controller, "LBR_iiwa_7_R800_1");
		tool = createFromTemplate("Tool");
		tool.attachTo(robot.getFlange()); // Attach the tool
	}
	
	// ----------------------------------------------------------------------------------
	// Connection manager
	// ----------------------------------------------------------------------------------
	public void run() {
		
		System.out.println("Initializing tcp server... ");
		
		int port = getApplicationData().getProcessData("port").getValue();
		
		try {
			
			serverSocket = new ServerSocket(port); // In case of error, close the application
			
			while (true){ // Until stop is clicked in the pad
				
				try {
					// ------------------ Connection acceptance -------------------
					System.out.println("Waiting for incoming connection...");
					clientSocket = serverSocket.accept();
					System.out.println("Connection accepted");
					// ------------------------------------------------------------
					
					BufferedReader socketReader = new BufferedReader(new InputStreamReader(clientSocket.getInputStream()));
					PrintWriter out = new PrintWriter(clientSocket.getOutputStream(), true);
					
					// ----------------------- Command loop -----------------------
					while (true){
						
						// v2 processing format
						// command, parameters
						
						String command = "";
						String[] parameters = new String[0];
						String line = socketReader.readLine(); // Blocked until line received
						
						if (line == null) break; // Restart socket server
						
						String[] processedLine = line.split(":");
						
						if (processedLine.length == 1){
							command = processedLine[0].trim();
						} else if (processedLine.length == 2){
							command = processedLine[0].trim();
							parameters = processedLine[1].trim().split("\\s+");
						}
						
						if (command.compareToIgnoreCase("bye") == 0){
							out.println("bye");
							out.flush();
							clientSocket.close();
							break;
						} else {
							out.println(process(command, parameters));
							out.flush();
						}
					}
					// ------------------------------------------------------------
					
				} catch (IOException e) { }
				
				// Prepare robot for a new connection
				if (motionContainer != null) motionContainer.cancel(); // Stop the robot
				Thread.sleep(1000); // Wait 1 second before retry
				
			}
			
		} catch (Exception e){
			// Stop button clicked in the control pad or critical error
			// Sockets are close in the dispose function
			// Some events like a broken connection could reach this Exception Handler,
			// but I considered better to start again the program than don't stop the robot
			if (motionContainer != null) motionContainer.cancel();
			e.printStackTrace(); // Trace in red in the pad
		}
	}
	
	void robotModeChange(RobotMode newMode){
		if (motionContainer != null){
			if (lastRobotMode == RobotMode.smart){
				if (newMode != RobotMode.smart) smartMotion.stopMotion();
			}
			if (lastRobotMode == RobotMode.direct){
				if (newMode != RobotMode.direct) directMotion.stopMotion();
			}
		}
		lastRobotMode = newMode;
	}
	
// ----------------------------------------------------------------------------------
// Command manager
// ----------------------------------------------------------------------------------

	String process(String command, String[] parameters){
		
		boolean simulation = getApplicationData().getProcessData("simulation").getValue();
		
// --------------------------------------------------------------------------------------
// Smart / Direct Mode (joints)
// --------------------------------------------------------------------------------------
		
	if (command.compareToIgnoreCase("smart joint move") == 0){
		
		if (smartServo == null) smartServo = new SmartServo(robot.getCurrentJointPosition());
		
		if (lastRobotMode != RobotMode.smart){
			if (motionContainer != null) motionContainer.cancel();
			motionContainer = robot.moveAsync(smartServo);
			smartMotion = smartServo.getRuntime();
		}
		
		try {
		
			JointPosition jointPosition = new JointPosition(
					Double.parseDouble(parameters[0]), 
					Double.parseDouble(parameters[1]),
					Double.parseDouble(parameters[2]),
					Double.parseDouble(parameters[3]),
					Double.parseDouble(parameters[4]),
					Double.parseDouble(parameters[5]),
					Double.parseDouble(parameters[6]));
			
			JointPosition jointSpeed = new JointPosition(
					Double.parseDouble(parameters[7]), 
					Double.parseDouble(parameters[8]),
					Double.parseDouble(parameters[9]),
					Double.parseDouble(parameters[10]),
					Double.parseDouble(parameters[11]),
					Double.parseDouble(parameters[12]),
					Double.parseDouble(parameters[13]));

			if (!simulation){ 
				smartMotion.setDestination(jointPosition, jointSpeed);
			} else {
				simulation_joints = jointPosition;
			}
			
			lastRobotMode = RobotMode.smart;
			
			return "done";
			
		} catch (Exception e){
			return "error";
		}
		
	} else if (command.compareToIgnoreCase("direct joint move") == 0){ 
		
		if (directServo == null) directServo = new DirectServo(robot.getCurrentJointPosition());
		
		if (lastRobotMode != RobotMode.direct){
			if (motionContainer != null) motionContainer.cancel();
			motionContainer = robot.moveAsync(directServo);
			directMotion = directServo.getRuntime();
		}
		
		try {
		
			JointPosition jointPosition = new JointPosition(
					Double.parseDouble(parameters[0]), 
					Double.parseDouble(parameters[1]),
					Double.parseDouble(parameters[2]),
					Double.parseDouble(parameters[3]),
					Double.parseDouble(parameters[4]),
					Double.parseDouble(parameters[5]),
					Double.parseDouble(parameters[6]));
			
			JointPosition jointSpeed = new JointPosition(
					Double.parseDouble(parameters[7]), 
					Double.parseDouble(parameters[8]),
					Double.parseDouble(parameters[9]),
					Double.parseDouble(parameters[10]),
					Double.parseDouble(parameters[11]),
					Double.parseDouble(parameters[12]),
					Double.parseDouble(parameters[13]));
	
			if (!simulation){
				directServo.setJointVelocityRel(jointSpeed.get());
				directMotion.setDestination(jointPosition);
			} else {
				simulation_joints = jointPosition;
			}
			
			lastRobotMode = RobotMode.direct;
			
			return "done";
	
		} catch (Exception e){
			return "error";
		}

// --------------------------------------------------------------------------------------
// Smart / Direct Mode (frames)
// --------------------------------------------------------------------------------------
		
	} else if (command.compareToIgnoreCase("smart cartesian move") == 0){
		
		if (smartServo == null) smartServo = new SmartServo(robot.getCurrentJointPosition());
		
		if (lastRobotMode != RobotMode.smart){
			if (motionContainer != null) motionContainer.cancel();
			motionContainer = robot.moveAsync(smartServo);
			smartMotion = smartServo.getRuntime();
		}
		
		try {
			
			Frame frame = new Frame(World.Current.getRootFrame());
			frame.setX(Double.parseDouble(parameters[0]));
			frame.setY(Double.parseDouble(parameters[1]));
			frame.setZ(Double.parseDouble(parameters[2]));
			frame.setAlphaRad(Double.parseDouble(parameters[3]));
			frame.setBetaRad(Double.parseDouble(parameters[4]));
			frame.setGammaRad(Double.parseDouble(parameters[5]));
			double speed = Double.parseDouble(parameters[6]);

			if (!simulation){ 
				smartServo.setJointVelocityRel(speed);
				smartMotion.setDestination(frame);
			}
			
			lastRobotMode = RobotMode.smart;
			
			return "done";
			
		} catch (Exception e){
			return "error";
		}
		
	} else if (command.compareToIgnoreCase("direct cartesian move") == 0){ 
		
		if (directServo == null) directServo = new DirectServo(robot.getCurrentJointPosition());
		
		if (lastRobotMode != RobotMode.direct){
			if (motionContainer != null) motionContainer.cancel();
			motionContainer = robot.moveAsync(directServo);
			directMotion = directServo.getRuntime();
		}
		
		try {
		
			Frame frame = new Frame(World.Current.getRootFrame());
			frame.setX(Double.parseDouble(parameters[0]));
			frame.setY(Double.parseDouble(parameters[1]));
			frame.setZ(Double.parseDouble(parameters[2]));
			frame.setAlphaRad(Double.parseDouble(parameters[3]));
			frame.setBetaRad(Double.parseDouble(parameters[4]));
			frame.setGammaRad(Double.parseDouble(parameters[5]));
			double speed = Double.parseDouble(parameters[6]);
	
			if (!simulation){
				directServo.setJointVelocityRel(speed);
				directMotion.setDestination(frame);
			}
			
			lastRobotMode = RobotMode.direct;
			
			return "done";
	
		} catch (Exception e){
			return "error";
		}
		
// --------------------------------------------------------------------------------------
// Simple moves
// --------------------------------------------------------------------------------------
		
		// ------------------------------------------------------------------------------
		// Joint move without force collision checking
		// ------------------------------------------------------------------------------
		} else if (command.compareToIgnoreCase("joint move") == 0){

			robotModeChange(RobotMode.normal);

			try {
			
				JointPosition jointPosition = new JointPosition(
						Double.parseDouble(parameters[0]), 
						Double.parseDouble(parameters[1]),
						Double.parseDouble(parameters[2]),
						Double.parseDouble(parameters[3]),
						Double.parseDouble(parameters[4]),
						Double.parseDouble(parameters[5]),
						Double.parseDouble(parameters[6]));
				
				JointPosition relativeSpeed = new JointPosition(
						Double.parseDouble(parameters[7]), 
						Double.parseDouble(parameters[8]),
						Double.parseDouble(parameters[9]),
						Double.parseDouble(parameters[10]),
						Double.parseDouble(parameters[11]),
						Double.parseDouble(parameters[12]),
						Double.parseDouble(parameters[13]));
						
				// Driver Relative Speed DEPRECATED -> you will need to set this according to the current
				// indicated speed % in the smartpad if you wish to work with the real speed.
				// Sunrise library doesn't allow easily to work with speed real values. To do this
				// the speed in m/s or rad/s must be transformed. 
				// driver_speed_percentage was removed from xml and moved to ROS side. This was the original concept:
				//double relative_speed = getApplicationData().getProcessData("driver_speed_percentage").getValue(); // 0.0..1.0
				
				// Calculate from absolute joint velocity to relative
				// PTP motion = ptp(jointPosition);
				
				// http://www.kuka-robotics.com/res/sps/f776ebab-f613-4818-9feb-527612db8dc4_db_LBRiiwa_en.pdf
				//double maxSpeed[] = {1.71042267, 1.71042267, 1.74532925, 2.26892803, 2.44346095, 3.14159265, 3.14159265}; // radians per second 
				//	for (int i=0; i<7; i++) maxSpeed[i] = maxSpeed[i] * relative_speed; 
				//double absoluteSpeed[] = new double[7]; 
				//	for (int i=0; i<7; i++) absoluteSpeed[i] = Double.parseDouble(parameters[7 + i]); // in rad / s
				//double relativeSpeed[] = new double[7];
				//	for (int i=0; i<7; i++) relativeSpeed[i] = absoluteSpeed[i] / maxSpeed[i];
				//	for (int i=0; i<7; i++) if (relativeSpeed[i] > 1.0) relativeSpeed[i] = 1.0;
	
				motionContainer = tool.moveAsync(ptp(jointPosition).setJointVelocityRel(relativeSpeed.get()).setBlendingRel(0.1));		
				
				return "done";
			
			} catch(Exception e){
				return "error";
			}
		
		// ------------------------------------------------------------------------------
		// LIN / PTP move without force collision checking
		// ------------------------------------------------------------------------------
		} else if ((command.compareToIgnoreCase("lin move") == 0) || (command.compareToIgnoreCase("ptp move") == 0)){
			
			robotModeChange(RobotMode.normal);
			
			try {
			
				Frame frame = new Frame(World.Current.getRootFrame());
				frame.setX(Double.parseDouble(parameters[0]));
				frame.setY(Double.parseDouble(parameters[1]));
				frame.setZ(Double.parseDouble(parameters[2]));
				frame.setAlphaRad(Double.parseDouble(parameters[3]));
				frame.setBetaRad(Double.parseDouble(parameters[4]));
				frame.setGammaRad(Double.parseDouble(parameters[5]));
				double speed = Double.parseDouble(parameters[6]);
				double blending = Double.parseDouble(parameters[7]);	
				if ((command.compareToIgnoreCase("lin move") == 0)){ 
					motionContainer = tool.moveAsync(lin(frame).setJointVelocityRel(speed).setBlendingCart(blending));
				} else {
					motionContainer = tool.moveAsync(ptp(frame).setJointVelocityRel(speed).setBlendingCart(blending));
				}
				return "done";
			
			} catch (Exception e) {
				return "error";
			}
			
// --------------------------------------------------------------------------------------
// Redundancy parameter
// --------------------------------------------------------------------------------------
		
		} else if ((command.compareToIgnoreCase("linr move") == 0) || (command.compareToIgnoreCase("ptpr move") == 0)){
			
			robotModeChange(RobotMode.normal);
			
			try {
			
				Frame frame = new Frame(World.Current.getRootFrame());
				frame.setX(Double.parseDouble(parameters[0]));
				frame.setY(Double.parseDouble(parameters[1]));
				frame.setZ(Double.parseDouble(parameters[2]));
				frame.setAlphaRad(Double.parseDouble(parameters[3]));
				frame.setBetaRad(Double.parseDouble(parameters[4]));
				frame.setGammaRad(Double.parseDouble(parameters[5]));
				double speed = Double.parseDouble(parameters[6]);
				double blending = Double.parseDouble(parameters[7]);	
				double redundancy = Double.parseDouble(parameters[8]); // Redundancy in radians
				
				LBRE1Redundancy newRedundancyInformation = new LBRE1Redundancy().setE1(redundancy);
				
				frame.setRedundancyInformation(robot, newRedundancyInformation);
				
				if ((command.compareToIgnoreCase("linr move") == 0)){ 
					motionContainer = tool.moveAsync(lin(frame).setJointVelocityRel(speed).setBlendingCart(blending));	
				} else {
					motionContainer = tool.moveAsync(ptp(frame).setJointVelocityRel(speed).setBlendingCart(blending));
				}
				
				return "done";
			
			} catch(Exception e){
				return "error";
			}
			
// --------------------------------------------------------------------------------------
// Force ruled moves
// --------------------------------------------------------------------------------------
			
		// ------------------------------------------------------------------------------
		// Ptp and Lin move with force collision checking in Z
		// ------------------------------------------------------------------------------		
		} else if ((command.compareToIgnoreCase("ptpforcez move") == 0) || (command.compareToIgnoreCase("linforcez move") == 0)){
			
			robotModeChange(RobotMode.normal);
			
			try {
			
				Frame frame = new Frame(World.Current.getRootFrame());
				frame.setX(Double.parseDouble(parameters[0]));
				frame.setY(Double.parseDouble(parameters[1]));
				frame.setZ(Double.parseDouble(parameters[2]));
				frame.setAlphaRad(Double.parseDouble(parameters[3]));
				frame.setBetaRad(Double.parseDouble(parameters[4]));
				frame.setGammaRad(Double.parseDouble(parameters[5]));
				double speed = Double.parseDouble(parameters[6]);
				double blending = Double.parseDouble(parameters[7]);
				double thresholdZ = Double.parseDouble(parameters[8]); // Force threshold X
				double offsetZ = robot.getExternalForceTorque(tool.getDefaultMotionFrame()).getForce().getZ(); // Force offset Z
	
				ForceComponentCondition conditionZ = new ForceComponentCondition(tool.getDefaultMotionFrame(), CoordinateAxis.Z, offsetZ - thresholdZ, offsetZ + thresholdZ); 
				
				if ((command.compareToIgnoreCase("ptpforcez move") == 0)){
					motionContainer = tool.moveAsync(ptp(frame).setJointVelocityRel(speed).setBlendingCart(blending).breakWhen(conditionZ));
				} else {
					motionContainer = tool.moveAsync(lin(frame).setJointVelocityRel(speed).setBlendingCart(blending).breakWhen(conditionZ));
				}
				
				return "done";
			
			} catch(Exception e) {
				return "error";
			}
			
		// ------------------------------------------------------------------------------
		// Ptp and Lin move with force collision checking in X, Y, Z
		// ------------------------------------------------------------------------------		
		} else if ((command.compareToIgnoreCase("ptpforce move") == 0) || (command.compareToIgnoreCase("linforce move") == 0)){	
			
			robotModeChange(RobotMode.normal);
			
			try {
			
				Frame frame = new Frame(World.Current.getRootFrame());
				frame.setX(Double.parseDouble(parameters[0]));
				frame.setY(Double.parseDouble(parameters[1]));
				frame.setZ(Double.parseDouble(parameters[2]));
				frame.setAlphaRad(Double.parseDouble(parameters[3]));
				frame.setBetaRad(Double.parseDouble(parameters[4]));
				frame.setGammaRad(Double.parseDouble(parameters[5]));
				double speed = Double.parseDouble(parameters[6]);
				double blending = Double.parseDouble(parameters[7]);
				double thresholdX = Double.parseDouble(parameters[8]); // Force threshold X
				double thresholdY = Double.parseDouble(parameters[9]); // Force threshold Y
				double thresholdZ = Double.parseDouble(parameters[10]); // Force threshold Z
				double offsetX = robot.getExternalForceTorque(tool.getDefaultMotionFrame()).getForce().getX(); // Force offset X
				double offsetY = robot.getExternalForceTorque(tool.getDefaultMotionFrame()).getForce().getY(); // Force offset Y
				double offsetZ = robot.getExternalForceTorque(tool.getDefaultMotionFrame()).getForce().getZ(); // Force offset Z
				
				ForceComponentCondition conditionX = new ForceComponentCondition(tool.getDefaultMotionFrame(), CoordinateAxis.X, offsetX - thresholdX, offsetX + thresholdX); 
				ForceComponentCondition conditionY = new ForceComponentCondition(tool.getDefaultMotionFrame(), CoordinateAxis.Y, offsetY - thresholdY, offsetY + thresholdY); 
				ForceComponentCondition conditionZ = new ForceComponentCondition(tool.getDefaultMotionFrame(), CoordinateAxis.Z, offsetZ - thresholdZ, offsetZ + thresholdZ); 
				
				if ((command.compareToIgnoreCase("ptpforce move") == 0)){
					motionContainer = tool.moveAsync(ptp(frame).setJointVelocityRel(speed).setBlendingCart(blending).breakWhen(conditionX).breakWhen(conditionY).breakWhen(conditionZ));
				} else {
					motionContainer = tool.moveAsync(lin(frame).setJointVelocityRel(speed).setBlendingCart(blending).breakWhen(conditionX).breakWhen(conditionY).breakWhen(conditionZ));
				}
				
				return "done";
			
			} catch(Exception e) {
				return "error";
			}
		
		// ------------------------------------------------------------------------------
		// 	ptpstiff and linstiff
		// ------------------------------------------------------------------------------
		} else if ((command.compareToIgnoreCase("ptpstiff move") == 0) || (command.compareToIgnoreCase("linstiff move") == 0)){

			robotModeChange(RobotMode.impedance);
			
			try {
			
				Frame frame = new Frame(World.Current.getRootFrame());
				frame.setX(Double.parseDouble(parameters[0]));
				frame.setY(Double.parseDouble(parameters[1]));
				frame.setZ(Double.parseDouble(parameters[2]));
				frame.setAlphaRad(Double.parseDouble(parameters[3]));
				frame.setBetaRad(Double.parseDouble(parameters[4]));
				frame.setGammaRad(Double.parseDouble(parameters[5]));
				double speed = Double.parseDouble(parameters[6]);
				double stiffness = Double.parseDouble(parameters[7]);
				double forceX = Double.parseDouble(parameters[8]);
				double forceY = Double.parseDouble(parameters[9]);
				double forceZ = Double.parseDouble(parameters[10]);
				double torqueX = Double.parseDouble(parameters[11]);
				double torqueY = Double.parseDouble(parameters[12]);
				double torqueZ = Double.parseDouble(parameters[13]);
				
				CartesianImpedanceControlMode impedance = new CartesianImpedanceControlMode();
				impedance.parametrize(CartDOF.TRANSL).setStiffness(stiffness);
				
				if (motionContainer != null) motionContainer.cancel(); 
				
				impedance.setAdditionalControlForce(-forceX, -forceY, -forceZ, -torqueX, -torqueY, -torqueZ);
				
				if ((command.compareToIgnoreCase("ptpstiff move") == 0)){
					motionContainer = tool.moveAsync(ptp(frame).setJointVelocityRel(speed).setMode(impedance));	
				} else {
					motionContainer = tool.moveAsync(lin(frame).setJointVelocityRel(speed).setMode(impedance));
				}
				return "done";
			
			} catch(Exception e) {
				return "error";
			}
		
// --------------------------------------------------------------------------------------
// Position holder
// --------------------------------------------------------------------------------------
		
		// ------------------------------------------------------------------------------
		// 	Start gravity compensation
		//  Next command after gravity start must be stop
		//  Otherwise the robot will not move and will block until program ends
		// ------------------------------------------------------------------------------
		} else if (command.compareToIgnoreCase("start gravity") == 0){	
			
			robotModeChange(RobotMode.impedance);
			
			if (motionContainer != null) motionContainer.cancel();
			
			try {
			
				double stiffness = Double.parseDouble(parameters[0]);
				
				PositionHold posHoldGravComp = positionHold(new JointImpedanceControlMode(stiffness,stiffness,stiffness,stiffness,stiffness,stiffness,stiffness),-1,TimeUnit.SECONDS);
				motionContainer=tool.moveAsync(posHoldGravComp);  
				
			return "done";	
			
			} catch (Exception e){
				return "error";
			}
			
// --------------------------------------------------------------------------------------
// Publishers
// --------------------------------------------------------------------------------------
			
		// ------------------------------------------------------------------------------
		// Frames publishing, tool and flange
		// ------------------------------------------------------------------------------
		} else if (command.compareToIgnoreCase("get tool frame") == 0){
			Transformation t = tool.getDefaultMotionFrame().transformationFromWorld();
			return t.getX() + " " + t.getY() + " " + t.getZ() + " " + t.getAlphaRad() + " " + t.getBetaRad() + " " + t.getGammaRad();
		} else if (command.compareToIgnoreCase("get flange frame") == 0){
			Transformation t = robot.getFlange().transformationFromWorld();
			return t.getX() + " " + t.getY() + " " + t.getZ() + " " + t.getAlphaRad() + " " + t.getBetaRad() + " " + t.getGammaRad();	
		// ------------------------------------------------------------------------------
		// Move info publishing (stopped or moving)
		// ------------------------------------------------------------------------------
		} else if (command.compareToIgnoreCase("get status") == 0){
 			if (motionContainer == null){
 				return "stopped";
 			} else {
 				if (motionContainer.isFinished()) return "stopped"; else return "moving";
 			}
 		// ------------------------------------------------------------------------------
		// Force publishing
		// ------------------------------------------------------------------------------
 		} else if (command.compareToIgnoreCase("get cartesian force") == 0){
 			Vector force = robot.getExternalForceTorque(tool.getDefaultMotionFrame()).getForce();
 			Vector torque = robot.getExternalForceTorque(tool.getDefaultMotionFrame()).getTorque();
 			return force.getX() + " " + force.getY() + " " + force.getZ() + " " + torque.getX() + " " + torque.getY() + " " + torque.getZ();
 		// ------------------------------------------------------------------------------
		// 	Joints publishing
		// ------------------------------------------------------------------------------
 		} else if (command.compareToIgnoreCase("get joint position") == 0){
 			double j[] = robot.getCurrentJointPosition().get();
 			if (simulation && simulation_joints != null) j = simulation_joints.get();
 			String result = String.valueOf(j[0]);
 			for (int i = 1; i <= 6; i++) result = result + " " + String.valueOf(j[i]);
 			return result;
 		// ------------------------------------------------------------------------------
		// 	Joints torque publishing
		// ------------------------------------------------------------------------------
 		} else if (command.compareToIgnoreCase("get joint torque") == 0){
 			double t[] = robot.getExternalTorque().getTorqueValues();
 			//TorqueSensorData torque = robot.getMeasuredTorque();
 			String result = String.valueOf(t[0]);
 			for (int i = 1; i <= 6; i++) result = result + " " + String.valueOf(t[i]);
 			return result;
 			
// --------------------------------------------------------------------------------------
// Motion cancel (required after gravity start)
// --------------------------------------------------------------------------------------
		} else if (command.compareToIgnoreCase("stop") == 0){
			if (motionContainer != null) motionContainer.cancel();
			return "done";
// --------------------------------------------------------------------------------------
// Ack
// --------------------------------------------------------------------------------------
		} else if (command.compareToIgnoreCase("hello") == 0){
			return "hello";
// --------------------------------------------------------------------------------------
// command 
// --------------------------------------------------------------------------------------	
		} else {
			return "unknown command";
		}
	}
	
}
