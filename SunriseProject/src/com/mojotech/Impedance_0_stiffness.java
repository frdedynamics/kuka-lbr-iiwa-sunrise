package com.mojotech;

import com.kuka.roboticsAPI.applicationModel.RoboticsAPIApplication;
import static com.kuka.roboticsAPI.motionModel.BasicMotions.*;

import com.kuka.roboticsAPI.deviceModel.LBR;
import com.kuka.roboticsAPI.motionModel.IMotionContainer;
import com.kuka.roboticsAPI.motionModel.PTP;
import com.kuka.roboticsAPI.motionModel.PositionHold;
import com.kuka.roboticsAPI.motionModel.controlModeModel.JointImpedanceControlMode;
import com.kuka.roboticsAPI.uiModel.ApplicationDialogType;

public class Impedance_0_stiffness extends RoboticsAPIApplication {

	final static double offsetAxis2And4=Math.toRadians(10);

	private LBR lbr;
	private static double[] startPosition=new double[]{0,offsetAxis2And4,0,offsetAxis2And4-Math.toRadians(90),0,Math.toRadians(90),0};

	public void initialize() {
		lbr = getContext().getDeviceFromType(LBR.class);
	}

	public void run() {
		getLogger().info("Show modal dialog and wait for user to confirm");
        int isCancel = getApplicationUI().displayModalDialog(ApplicationDialogType.QUESTION, "Move to start position?", "OK", "Cancel");
        if (isCancel == 1)
        {
            return;
        }

		getLogger().info("Move to start position");
		PTP ptpToStartPosition = ptp(startPosition);
		ptpToStartPosition.setJointVelocityRel(0.25);
		lbr.move(ptpToStartPosition);
		
		isCancel = getApplicationUI().displayModalDialog(ApplicationDialogType.QUESTION, "Do you feel lucky?", "OK", "Cancel");
        if (isCancel == 1)
        {
            return;
        }
		
		getLogger().info("Hold position in 0 impedance control mode");
		JointImpedanceControlMode ctrMode = new JointImpedanceControlMode(200, 200, 200, 200, 200, 200, 0);
		IMotionContainer positionHoldContainer = lbr.moveAsync((new PositionHold(ctrMode, -1, null)));

		getLogger().info("Show modal dialog while executing position hold");
		getApplicationUI().displayModalDialog(ApplicationDialogType.INFORMATION, "Press ok to finish the application.", "OK");

		positionHoldContainer.cancel();
	}

}
