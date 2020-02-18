package com.mojotech;

import javax.inject.Inject;

import com.kuka.generated.ioAccess.MediaFlangeIOGroup;
import com.kuka.grippertoolbox.api.gripper.AbstractGripper;
import com.kuka.roboticsAPI.applicationModel.RoboticsAPIApplication;
import static com.kuka.roboticsAPI.motionModel.BasicMotions.*;

import com.kuka.roboticsAPI.deviceModel.LBR;
import com.kuka.roboticsAPI.geometricModel.CartDOF;
import com.kuka.roboticsAPI.motionModel.IMotionContainer;
import com.kuka.roboticsAPI.motionModel.PTP;
import com.kuka.roboticsAPI.motionModel.PositionHold;
import com.kuka.roboticsAPI.motionModel.controlModeModel.CartesianImpedanceControlMode;
import com.kuka.roboticsAPI.uiModel.ApplicationDialogType;

public class Handshake extends RoboticsAPIApplication {
	
	@Inject
	private AbstractGripper gripper;
    
    @Inject
    private MediaFlangeIOGroup mediaFlange;


	private static final int stiffnessZ = 50;
	private static final int stiffnessY = 2500;
	private static final int stiffnessX = 2500;
	private static final int stiffnessA = 20;
	private static final int stiffnessB = 1;
	private static final int stiffnessC = 20;
	
	private LBR lbr;
	
	private final static String informationText=
			"Come say hi! Let me move to shake hands first.";


	public void initialize() {
		lbr = getContext().getDeviceFromType(LBR.class);
		
		mediaFlange = new MediaFlangeIOGroup(lbr.getController());
        gripper.attachTo(lbr.getFlange());
        
        mediaFlange.setLEDBlue(true);
        
        // yellow
        mediaFlange.setOutputX3Pin12(true);
        mediaFlange.setOutputX3Pin2(false);
        
        // De-activate grip
        mediaFlange.setOutputX3Pin1(false);
        mediaFlange.setOutputX3Pin11(false);
	}

	public void run() {
		getLogger().info("Show modal dialog and wait for user to confirm");
        int isCancel = getApplicationUI().displayModalDialog(ApplicationDialogType.QUESTION, informationText, "OK", "Cancel");
        if (isCancel == 1)
        {
            return;
        }

		getLogger().info("Move to start position");
		PTP ptpToStartPosition = ptp(Math.toRadians(0.0),
		                              Math.toRadians(50.0),
		                              Math.toRadians(0.0),
		                              Math.toRadians(-80.0),
		                              Math.toRadians(0.0),
		                              Math.toRadians(-45.0),
		                              Math.toRadians(60.0));
		ptpToStartPosition.setJointVelocityRel(0.25);
		gripper.move(ptpToStartPosition);

		getLogger().info("Hold position in impedance control mode");
		CartesianImpedanceControlMode impedanceControlMode = 	new CartesianImpedanceControlMode();
		impedanceControlMode.parametrize(CartDOF.X).setStiffness(stiffnessX);
		impedanceControlMode.parametrize(CartDOF.Y).setStiffness(stiffnessY);
		impedanceControlMode.parametrize(CartDOF.Z).setStiffness(stiffnessZ);
		impedanceControlMode.parametrize(CartDOF.A).setStiffness(stiffnessA);
		impedanceControlMode.parametrize(CartDOF.B).setStiffness(stiffnessB);
		impedanceControlMode.parametrize(CartDOF.C).setStiffness(stiffnessC);
		
		// The robot is set to position hold and impedance control mode gets activated without a timeout. 
		IMotionContainer positionHoldContainer = gripper.moveAsync((new PositionHold(impedanceControlMode, -1, null)));

		mediaFlange.setOutputX3Pin12(false);
        mediaFlange.setOutputX3Pin2(true); // green
		getLogger().info("Show modal dialog while executing position hold");
		getApplicationUI().displayModalDialog(ApplicationDialogType.INFORMATION, "Press ok to finish the application.", "OK");

		// As soon as the modal dialog returns, the motion container will be cancelled. This finishes the position hold. 
		positionHoldContainer.cancel();
	}

}
