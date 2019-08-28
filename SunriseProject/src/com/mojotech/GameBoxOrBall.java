package com.mojotech;


import com.kuka.generated.ioAccess.MediaFlangeIOGroup;
import com.kuka.roboticsAPI.applicationModel.RoboticsAPIApplication;
import static com.kuka.roboticsAPI.motionModel.BasicMotions.ptp;

import com.kuka.roboticsAPI.deviceModel.LBR;
import com.kuka.roboticsAPI.geometricModel.CartDOF;
import com.kuka.roboticsAPI.geometricModel.Frame;
import com.kuka.roboticsAPI.motionModel.controlModeModel.CartesianImpedanceControlMode;
import com.kuka.roboticsAPI.uiModel.ApplicationDialogType;

public class GameBoxOrBall extends RoboticsAPIApplication {
	private LBR lbr;
	private MediaFlangeIOGroup media_flange;
	
	private static final int lowStiffnessZ = 100;
	private static final int lowStiffnessY = 100;
	private static final int lowStiffnessX = 100;
	
	private static final int highStiffnessZ = 1500;
	
	private static final double boxHeight = 200.0; // [mm]


	final static double offsetAxis2And4=Math.toRadians(10);
	private static double[] startPosition=new double[]{-Math.toRadians(90), offsetAxis2And4, 0, offsetAxis2And4-Math.toRadians(90), 0, Math.toRadians(90), 0};


	private final static String informationText=
			"Would you like to play a game?"+ "\n" +
			"\n" +
			"The robot moves to the start position and from here " +
			"you move the robot by hand to " + "\n" +
			"guess if there`s an imaginary box or ball on the table.";

	public void initialize() {		
		lbr = getContext().getDeviceFromType(LBR.class);
		media_flange = new MediaFlangeIOGroup(lbr.getController());
	}

	public void run() {		
		getLogger().info("Show modal dialog and wait for user to confirm");
        int isCancel = getApplicationUI().displayModalDialog(ApplicationDialogType.QUESTION, informationText, "OK", "Cancel");
        if (isCancel == 1)
        {
            return;
        }

		getLogger().info("Set low impedance");
		CartesianImpedanceControlMode impedanceControlMode = new CartesianImpedanceControlMode();
		impedanceControlMode.parametrize(CartDOF.X).setStiffness(lowStiffnessX);
		impedanceControlMode.parametrize(CartDOF.Y).setStiffness(lowStiffnessY);
		impedanceControlMode.parametrize(CartDOF.Z).setStiffness(lowStiffnessZ);
		
		getLogger().info("Move to start position.");	
		lbr.move(ptp(startPosition).setJointVelocityRel(0.2));
		
		getLogger().info("Press green button to end." + "\n" + "Hold white button half way in to move robot by hand." + "\n" + "Press white button all the way in to panic");
		while (!media_flange.getUserButton()){
			Frame currentCartesianPosition = lbr.getCurrentCartesianPosition(lbr.getFlange());
			if (currentCartesianPosition.getZ() < boxHeight){
				impedanceControlMode.parametrize(CartDOF.Z).setStiffness(highStiffnessZ);
			}
		}
		
		getLogger().info("End");
		
	}

	


}
