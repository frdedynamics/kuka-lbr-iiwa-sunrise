package com.mojotech;


import com.kuka.roboticsAPI.applicationModel.RoboticsAPIApplication;
import static com.kuka.roboticsAPI.motionModel.BasicMotions.*;

import com.kuka.roboticsAPI.deviceModel.LBR;
import com.kuka.roboticsAPI.uiModel.ApplicationDialogType;

public class GoToJointTargets extends RoboticsAPIApplication {
	private LBR lbr;
	private final static String informationText=
			"This application is intended for floor mounted robots!"+ "\n" +
			"\n" +
			"The robot will move to the joint targets enter in the process data screen.";

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
        
        double tj1 = getApplicationData().getProcessData("tj1").getValue();
        double tj2 = getApplicationData().getProcessData("tj2").getValue();
        double tj3 = getApplicationData().getProcessData("tj3").getValue();
        double tj4 = getApplicationData().getProcessData("tj4").getValue();
        double tj5 = getApplicationData().getProcessData("tj5").getValue();
        double tj6 = getApplicationData().getProcessData("tj6").getValue();
        double tj7 = getApplicationData().getProcessData("tj7").getValue();
        
        getLogger().info("Show modal dialog and wait for user to confirm");
        isCancel = getApplicationUI().displayModalDialog(ApplicationDialogType.QUESTION, String.format("joint targets are \n %s \n %s \n %s \n %s \n %s \n %s \n %s",
        		Double.toString(tj1),
        		Double.toString(tj2),
        		Double.toString(tj3),
        		Double.toString(tj4),
        		Double.toString(tj5),
        		Double.toString(tj6),
        		Double.toString(tj7)),
        		"Go to targets!", "Cancel");
        if (isCancel == 1)
        {
            return;
        }
        
		getLogger().info("Move to targets");
		lbr.move(ptp(
				Math.toRadians(tj1),
				Math.toRadians(tj2),
				Math.toRadians(tj3),
				Math.toRadians(tj4),
				Math.toRadians(tj5),
				Math.toRadians(tj6),
				Math.toRadians(tj7)
				).setJointVelocityRel(0.2));
	}


}
