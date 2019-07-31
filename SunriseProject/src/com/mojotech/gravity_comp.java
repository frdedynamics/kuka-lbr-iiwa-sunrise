package com.mojotech;

import javax.inject.Inject;

import com.kuka.grippertoolbox.api.gripper.AbstractGripper;
import com.kuka.roboticsAPI.applicationModel.RoboticsAPIApplication;
import static com.kuka.roboticsAPI.motionModel.BasicMotions.ptp;

import com.kuka.roboticsAPI.deviceModel.LBR;
import com.kuka.roboticsAPI.geometricModel.CartDOF;
import com.kuka.roboticsAPI.motionModel.IMotionContainer;
import com.kuka.roboticsAPI.motionModel.PTP;
import com.kuka.roboticsAPI.motionModel.PositionHold;

import com.kuka.roboticsAPI.motionModel.controlModeModel.CartesianImpedanceControlMode;
import com.kuka.roboticsAPI.uiModel.ApplicationDialogType;

public class gravity_comp extends RoboticsAPIApplication {
	@Inject
	private AbstractGripper gripper;

	private LBR lbr;
	
	private static final int stiffnessZ = 50;
	private static final int stiffnessY = 50;  // 
	private static final int stiffnessX = 50;
	private static final int stiffnessA = 10;  // Z-axis  set low impedance in xyz
	private static final int stiffnessB = 10;  // Y-axis
	private static final int stiffnessC = 10;  // X-axis

	private static double[] startPosition=new double[]{0,Math.toRadians(50),0,Math.toRadians(-90),0,Math.toRadians(-50),Math.toRadians(60)};
	

	public void initialize() {
		lbr = getContext().getDeviceFromType(LBR.class);
	}

	public void run() {
		
		gripper.attachTo(lbr.getFlange());
		
		getLogger().info("Show modal dialog and wait for user to confirm");
        int isCancel = getApplicationUI().displayModalDialog(ApplicationDialogType.QUESTION, "Ok to move to start?", "OK", "Cancel");
        if (isCancel == 1)
        {
            return;
        }

		getLogger().info("Move to start position");
		PTP ptpToStartPosition = ptp(startPosition);
		ptpToStartPosition.setJointVelocityRel(0.2);
		lbr.move(ptpToStartPosition);
		
		getLogger().info("Hold position in impedance control mode");
		CartesianImpedanceControlMode impedanceControlMode = new CartesianImpedanceControlMode();
		impedanceControlMode.parametrize(CartDOF.X).setStiffness(stiffnessX);
		impedanceControlMode.parametrize(CartDOF.Y).setStiffness(stiffnessY);
		impedanceControlMode.parametrize(CartDOF.Z).setStiffness(stiffnessZ);
		
		impedanceControlMode.parametrize(CartDOF.A).setStiffness(stiffnessA);
		impedanceControlMode.parametrize(CartDOF.B).setStiffness(stiffnessB);
		impedanceControlMode.parametrize(CartDOF.C).setStiffness(stiffnessC);
		
		impedanceControlMode.parametrize(CartDOF.ALL).setDamping(0.95);

		// The robot is set to position hold and impedance control mode gets activated without a timeout. 
		IMotionContainer positionHoldContainer = gripper.moveAsync((new PositionHold(impedanceControlMode, -1, null)));

		getLogger().info("Show modal dialog while executing position hold");
		getApplicationUI().displayModalDialog(ApplicationDialogType.INFORMATION, "Press ok to finish the application.", "OK");

		// As soon as the modal dialog returns, the motion container will be cancelled. This finishes the position hold. 
		positionHoldContainer.cancel();
	}

}
