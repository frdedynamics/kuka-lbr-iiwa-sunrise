package application;

import javax.inject.Inject;

import com.kuka.grippertoolbox.api.gripper.AbstractGripper;
import com.kuka.roboticsAPI.applicationModel.RoboticsAPIApplication;
import static com.kuka.roboticsAPI.motionModel.BasicMotions.*;

import com.kuka.roboticsAPI.deviceModel.LBR;
import com.kuka.roboticsAPI.geometricModel.CartDOF;
import com.kuka.roboticsAPI.geometricModel.Frame;
import com.kuka.roboticsAPI.motionModel.controlModeModel.CartesianImpedanceControlMode;
import com.kuka.roboticsAPI.motionModel.IMotionContainer;
import com.kuka.roboticsAPI.motionModel.LIN;
import com.kuka.roboticsAPI.motionModel.PositionHold;
import com.kuka.roboticsAPI.uiModel.ApplicationDialogType;
//import com.kuka.roboticsAPI.motionModel.PTP;



public class RedundancyImpedanceMove extends RoboticsAPIApplication {
	@Inject
	private AbstractGripper gripper;
	
	private static double[] startPosition=new double[]{Math.toRadians(-1.28), Math.toRadians(40.86), Math.toRadians(1.69), Math.toRadians(-83.23), Math.toRadians(1.77), Math.toRadians(-32.53), -0.57};
	private LBR lbr;
	//private AbstractGripper gripper;

	public void initialize() {
		lbr = getContext().getDeviceFromType(LBR.class);
		boolean gripper_init = gripper.initialize();
		if (gripper_init) {
			getLogger().info("Gripper initialized");
		}
		else {
			getLogger().info("Gripper initialization failed");
		}
	}

	public void run() {
		
		
		gripper.attachTo(lbr.getFlange());
		
		// move to forward starting pose
		getLogger().info("Moving to start position");
		//lbr.move(ptp(0, Math.toRadians(10), 0, Math.toRadians(-80), 0, Math.toRadians(90), 0));
		lbr.move(ptp(startPosition));
		
		// set up impedance control
		// high translational/rotational stiffness, low null-space stiffness
		getLogger().info("Hold position in impedance control mode");
		final CartesianImpedanceControlMode controlMode = new CartesianImpedanceControlMode();

		final double stiffnessTrans = 5000.0; // N
		final double stiffnessRot = 300.0; // Nm
		final double stiffnessNull = 5.0;

		controlMode.parametrize(CartDOF.TRANSL).setStiffness(stiffnessTrans); //cartStiffnessTrans
		controlMode.parametrize(CartDOF.ROT).setStiffness(stiffnessRot); //cartStiffnessRot
		controlMode.setNullSpaceStiffness(stiffnessNull);
		controlMode.setNullSpaceDamping(0.3);

		// hold impedance control until dialog is closed by user
//		final IMotionContainer motionContainer = lbr.moveAsync((new PositionHold(controlMode, -1, null)));
//		getLogger().info("Show modal dialog while executing position hold");
//		getApplicationUI().displayModalDialog(ApplicationDialogType.INFORMATION, "Press ok to finish the application.",
//				"OK");
//		motionContainer.cancel();
		
		
		Frame startFrame = lbr.getCurrentCartesianPosition(gripper.getDefaultMotionFrame()); 
		//Frame gripperFrame = gripper.getDefaultMotionFrame();
		Frame firstFrame=(new Frame(startFrame)).setY(startFrame.getY() + 300);
		Frame secondFrame=(new Frame(startFrame)).setY(startFrame.getY() - 300);
		
		int runLoop = 0;
		
		while(runLoop <= 4) {
			LIN linToFirstFrame = lin(firstFrame);
			linToFirstFrame.setJointVelocityRel(0.5);
			linToFirstFrame.setMode(controlMode);
			gripper.move(linToFirstFrame);
			gripper.gripAsync();
			
			LIN linToSecondFrame = lin(secondFrame);
			linToSecondFrame.setJointVelocityRel(0.5);
			linToSecondFrame.setMode(controlMode);
			gripper.move(linToSecondFrame);
			gripper.releaseAsync();
			
			runLoop++;
		}
		
		
		
		
		
		getLogger().info("App finished");
	}
}
