package application;

import com.kuka.roboticsAPI.applicationModel.RoboticsAPIApplication;
import static com.kuka.roboticsAPI.motionModel.BasicMotions.*;

import com.kuka.roboticsAPI.deviceModel.LBR;
import com.kuka.roboticsAPI.deviceModel.LBRE1Redundancy;
import com.kuka.roboticsAPI.geometricModel.CartDOF;
import com.kuka.roboticsAPI.geometricModel.Frame;
import com.kuka.roboticsAPI.motionModel.LIN;
import com.kuka.roboticsAPI.motionModel.PTP;
import com.kuka.roboticsAPI.motionModel.controlModeModel.CartesianImpedanceControlMode;
import com.kuka.roboticsAPI.uiModel.ApplicationDialogType;

public class test extends RoboticsAPIApplication {
	private LBR lbr;
	
	private static final int stiffnessZ = 2500;                
	private static final int stiffnessY = 700;
	private static final int stiffnessX = 1500;

	final static double radiusOfCircMove=120;
	final static int nullSpaceAngle = 80;

	final static double offsetAxis2And4=Math.toRadians(20);
	final static double offsetAxis4And6=Math.toRadians(-40);

	double[] startPosition = new double[]{Math.toRadians(-1.28), Math.toRadians(40.86), Math.toRadians(1.69), Math.toRadians(-83.23), Math.toRadians(1.77), Math.toRadians(-32.53), Math.toRadians(-8.8)};

	private final static String informationText=
			"This application is intended for floor mounted robots!"+ "\n" +
			"\n" +
			"The robot moves to the start position and based on this position, a motion that " +
			"describes the symbol of lemniscate (a 'horizontal eight') will be executed." + "\n" +
			"In a next step the robot will move in nullspace by "+nullSpaceAngle+"? in both directions.";

	public void initialize() {		
		lbr = getContext().getDeviceFromType(LBR.class);
	}

	public void run() {		
		getLogger().info("Show modal dialog and wait for user to confirm");
        int isCancel = getApplicationUI().displayModalDialog(ApplicationDialogType.QUESTION, informationText, "OK", "Cancel");
        if (isCancel == 1)
        {
            return;
        }

		getLogger().info("Move to start position");	
		PTP ptpToStart = ptp(startPosition);
		ptpToStart.setJointVelocityRel(0.25);
		lbr.move(ptpToStart);
		
		getLogger().info("Set impedance");
		CartesianImpedanceControlMode impedanceControlMode = 	new CartesianImpedanceControlMode();
		impedanceControlMode.parametrize(CartDOF.X).setStiffness(stiffnessX);
		impedanceControlMode.parametrize(CartDOF.Y).setStiffness(stiffnessY);
		impedanceControlMode.parametrize(CartDOF.Z).setStiffness(stiffnessZ);
	
		Frame startFrame = lbr.getCurrentCartesianPosition(lbr.getFlange());
		int runLoop = 0;
		
		while(runLoop <= 2) {
			getLogger().info("Move to first position");
			Frame firstFrame=(new Frame(startFrame)).setY(startFrame.getY() + 100);
			LIN linToFirstFrame = lin(firstFrame);
			linToFirstFrame.setJointVelocityRel(0.25);
			linToFirstFrame.setMode(impedanceControlMode);
			lbr.move(linToFirstFrame);
			
			getLogger().info("Move in nullspace -"+nullSpaceAngle+"?");		
			Frame firstFrameWithChangedE1_1 = createChildFrameAndSetE1Offset(firstFrame,Math.toRadians(-nullSpaceAngle));
			LIN linToFirstFrameWithE1_1 = lin(firstFrameWithChangedE1_1);
			linToFirstFrameWithE1_1.setJointVelocityRel(0.25);
			linToFirstFrameWithE1_1.setMode(impedanceControlMode);
			lbr.move(linToFirstFrameWithE1_1);

			getLogger().info("Move in nullspace "+nullSpaceAngle+"?");
			Frame firstFrameWithChangedE1_2 = createChildFrameAndSetE1Offset(firstFrame,Math.toRadians(nullSpaceAngle));
			LIN linToFirstFrameWithE1_2 = lin(firstFrameWithChangedE1_2);
			linToFirstFrameWithE1_2.setJointVelocityRel(0.25);
			linToFirstFrameWithE1_2.setMode(impedanceControlMode);
			lbr.move(linToFirstFrameWithE1_2);
			
			getLogger().info("Move to second position");
			Frame secondFrame=(new Frame(startFrame)).setY(startFrame.getY() - 100);
			LIN linToSecondFrame = lin(secondFrame);
			linToSecondFrame.setJointVelocityRel(0.25);
			linToSecondFrame.setMode(impedanceControlMode);
			lbr.move(linToSecondFrame);
			
			getLogger().info("Move in nullspace -"+nullSpaceAngle+"?");		
			Frame secondFrameWithChangedE1_1 = createChildFrameAndSetE1Offset(secondFrame,Math.toRadians(-nullSpaceAngle));
			LIN linToSecondFrameWithE1_1 = lin(secondFrameWithChangedE1_1);
			linToSecondFrameWithE1_1.setJointVelocityRel(0.25);
			linToSecondFrameWithE1_1.setMode(impedanceControlMode);
			lbr.move(linToSecondFrameWithE1_1);

			getLogger().info("Move in nullspace "+nullSpaceAngle+"?");
			Frame secondFrameWithChangedE1_2 = createChildFrameAndSetE1Offset(secondFrame,Math.toRadians(nullSpaceAngle));
			LIN linToSecondFrameWithE1_2 = lin(secondFrameWithChangedE1_2);
			linToSecondFrameWithE1_2.setJointVelocityRel(0.25);
			linToSecondFrameWithE1_2.setMode(impedanceControlMode);
			lbr.move(linToSecondFrameWithE1_2);
			
			runLoop = runLoop + 1;
		}
		
		getLogger().info("Move to start position");
		LIN linToStartFrame = lin(startFrame);
		linToStartFrame.setJointVelocityRel(0.25);
		linToStartFrame.setMode(impedanceControlMode);
		lbr.move(linToStartFrame);
	}

	

	
	private Frame createChildFrameAndSetE1Offset( Frame parent, double offset) {

		// Create a new frame
		Frame childFrame = new Frame(parent);

		// Create new redundancy information
		LBRE1Redundancy newRedundancyInformation = new LBRE1Redundancy().setE1(offset);

		// Add new redundancy information to new frame
		childFrame.setRedundancyInformation(lbr, newRedundancyInformation);
		return childFrame;
	}
	

}
