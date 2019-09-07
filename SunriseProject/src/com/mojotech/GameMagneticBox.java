package com.mojotech;


import javax.inject.Inject;

import com.kuka.generated.ioAccess.MediaFlangeIOGroup;
import com.kuka.grippertoolbox.api.gripper.AbstractGripper;
import com.kuka.roboticsAPI.applicationModel.RoboticsAPIApplication;
import com.kuka.roboticsAPI.motionModel.PositionHold;
import static com.kuka.roboticsAPI.motionModel.BasicMotions.ptp;


import com.kuka.roboticsAPI.deviceModel.LBR;
import com.kuka.roboticsAPI.geometricModel.CartDOF;
import com.kuka.roboticsAPI.geometricModel.Frame;
import com.kuka.roboticsAPI.motionModel.IMotionContainer;
import com.kuka.roboticsAPI.motionModel.controlModeModel.CartesianImpedanceControlMode;
import com.kuka.roboticsAPI.uiModel.ApplicationDialogType;

public class GameMagneticBox extends RoboticsAPIApplication {
	
	@Inject
	private AbstractGripper gripper;
	
	private LBR lbr;
	private MediaFlangeIOGroup media_flange;
	
	private static final int lowStiffnessZ = 0;
	private static final int lowStiffnessY = 0;
	private static final int lowStiffnessX = 0;
	
	private static final int mediumStiffnessZ = 100;
	private static final int mediumStiffnessY = 100;
	private static final int mediumStiffnessX = 100;
	
	private static final int highStiffnessX = 1200;
	private static final int highStiffnessY = 1200;
	private static final int highStiffnessZ = 1200;
	
	
	private static final double boxHeight = 300.0; // [mm]
	private static final double boxWidth = 175.0;
	
	private static double boxXorigo = 0.0;
	private static final double boxYorigo = -600.0;
	
	private static final double marginLength = 5.0;


	final static double offsetAxis2And4=Math.toRadians(10);
	private static double[] startPosition=new double[]{-Math.toRadians(90), offsetAxis2And4, 0, offsetAxis2And4-Math.toRadians(90), 0, Math.toRadians(90), 0};
	private static double[] preStartPosition=new double[]{0,offsetAxis2And4,0,offsetAxis2And4-Math.toRadians(90),0,Math.toRadians(90),0};


	private final static String informationText=
			"Would you like to play a game?"+ "\n" +
			"\n" +
			"The robot moves to the start position and from here " +
			"you move the robot by hand to find" + "\n" +
			"an invisible magnetic box." + "\n" +
			"\n" +
			"Press the green button when you found the box";

	public void initialize() {		
		lbr = getContext().getDeviceFromType(LBR.class);
		media_flange = new MediaFlangeIOGroup(lbr.getController());
		gripper.attachTo(lbr.getFlange());
		
		media_flange.setLEDBlue(false);
		media_flange.setOutputX3Pin12(true);
		media_flange.setOutputX3Pin2(false); // yellow
	}

	public void run() {		
		boxXorigo = 600*Math.random() - 300.0;
		getLogger().info(Double.toString(boxXorigo));
		getLogger().info("Show modal dialog and wait for user to confirm");

        int isCancel = getApplicationUI().displayModalDialog(ApplicationDialogType.QUESTION, informationText, "OK", "Cancel");
        if (isCancel == 1)
        {
            return;
        }

		getLogger().info("Set medium impedance");
		media_flange.setLEDBlue(false);
		media_flange.setOutputX3Pin12(true);
		media_flange.setOutputX3Pin2(true); // red

		CartesianImpedanceControlMode impedanceControlMode = new CartesianImpedanceControlMode();
		impedanceControlMode.parametrize(CartDOF.X).setStiffness(mediumStiffnessX);
		impedanceControlMode.parametrize(CartDOF.Y).setStiffness(mediumStiffnessY);
		impedanceControlMode.parametrize(CartDOF.Z).setStiffness(mediumStiffnessZ);
		
		getLogger().info("Move to zero position (point to the sky)");
		gripper.move(ptp(0,0,0,0,0,0,0).setJointVelocityRel(0.2).setMode(impedanceControlMode));
		getLogger().info("Move to start position.");	
		gripper.moveAsync(ptp(0,Math.toRadians(10),0,Math.toRadians(-30),0,Math.toRadians(30),0).setJointVelocityRel(0.2));
		gripper.moveAsync(ptp(preStartPosition).setJointVelocityRel(0.2).setMode(impedanceControlMode).setBlendingRel(0.2));
		gripper.move(ptp(startPosition).setJointVelocityRel(0.2).setMode(impedanceControlMode));
		
		media_flange.setOutputX3Pin12(false);
		media_flange.setOutputX3Pin2(true); // green
		isCancel = getApplicationUI().displayModalDialog(ApplicationDialogType.QUESTION, "Grab the robot!", "OK", "Cancel");
        if (isCancel == 1)
        {
            return;
        }
        impedanceControlMode.parametrize(CartDOF.X).setStiffness(lowStiffnessX);
		impedanceControlMode.parametrize(CartDOF.Y).setStiffness(lowStiffnessY);
		impedanceControlMode.parametrize(CartDOF.Z).setStiffness(lowStiffnessZ);
		IMotionContainer positionHoldContainer = gripper.moveAsync((new PositionHold(impedanceControlMode, -1, null)));
		boolean isLowImpedance = true;
		
		getLogger().info("Press green button to end.");
		getLogger().info(gripper.getDefaultMotionFrame().toString());
		getLogger().info(lbr.getCurrentCartesianPosition(gripper.getDefaultMotionFrame()).toString());
		
		while (!media_flange.getUserButton()){
			Frame currentCartesianPosition = lbr.getCurrentCartesianPosition(gripper.getDefaultMotionFrame());
			if (currentCartesianPosition.getZ() < boxHeight &&
					(Math.abs(currentCartesianPosition.getX() - boxXorigo) < boxWidth/2) &&
					(Math.abs(currentCartesianPosition.getY() - boxYorigo) < boxWidth/2)){

				if (isLowImpedance){
					getLogger().info("Set high impedance");
					impedanceControlMode.parametrize(CartDOF.X).setStiffness(highStiffnessX);
					impedanceControlMode.parametrize(CartDOF.Y).setStiffness(highStiffnessY);
					impedanceControlMode.parametrize(CartDOF.Z).setStiffness(highStiffnessZ);
					positionHoldContainer.cancel();
					positionHoldContainer = gripper.moveAsync((new PositionHold(impedanceControlMode, -1, null)));
					media_flange.setLEDBlue(true);
					isLowImpedance = false;
				}
			}else if (currentCartesianPosition.getZ() > boxHeight + marginLength ||
					(Math.abs(currentCartesianPosition.getX() - boxXorigo) > boxWidth/2 + marginLength) ||
					(Math.abs(currentCartesianPosition.getY() - boxYorigo) > boxWidth/2 + marginLength)){
				
				if (!isLowImpedance){
					getLogger().info("Set low impedance");
					impedanceControlMode.parametrize(CartDOF.X).setStiffness(lowStiffnessX);
					impedanceControlMode.parametrize(CartDOF.Y).setStiffness(lowStiffnessY);
					impedanceControlMode.parametrize(CartDOF.Z).setStiffness(lowStiffnessZ);
					positionHoldContainer.cancel();
					positionHoldContainer.await();
					positionHoldContainer = gripper.moveAsync((new PositionHold(impedanceControlMode, -1, null)));
					media_flange.setLEDBlue(false);
					isLowImpedance = true;
				}
			}
		}
		
		positionHoldContainer.cancel();
		getLogger().info("Set medium impedance");
		media_flange.setLEDBlue(false);
		media_flange.setOutputX3Pin12(true);
		media_flange.setOutputX3Pin2(true); // red

		impedanceControlMode.parametrize(CartDOF.X).setStiffness(mediumStiffnessX);
		impedanceControlMode.parametrize(CartDOF.Y).setStiffness(mediumStiffnessY);
		impedanceControlMode.parametrize(CartDOF.Z).setStiffness(mediumStiffnessZ);
		
		getLogger().info("Move to pre start position");
		gripper.moveAsync(ptp(preStartPosition).setJointVelocityRel(0.2).setMode(impedanceControlMode).setBlendingRel(0.2));
		
		getLogger().info("Move to zero position (point to the sky)");
		gripper.move(ptp(0,Math.toRadians(10),0,Math.toRadians(-30),0,Math.toRadians(30),0).setJointVelocityRel(0.2).setMode(impedanceControlMode));
		gripper.move(ptp(0,0,0,0,0,0,0).setJointVelocityRel(0.2));
		
		if (!isLowImpedance){
			getLogger().info("Yeeeeeeaaaah!");	
			
			lbr.move(ptp(0,Math.toRadians(10),0,Math.toRadians(30),0,Math.toRadians(-30),0).setJointVelocityRel(0.2));
			gripper.moveAsync(ptp(getFrame("/Tiger_Woods_Swing_Bottom")).setJointVelocityRel(0.2).setMode(impedanceControlMode).setBlendingRel(0.2));
			gripper.releaseAsync();
			
			for (int i=0; i < 2; i++){
				gripper.move(ptp(getFrame("/Tiger_Woods_Swing_Top")).setJointVelocityRel(0.4).setMode(impedanceControlMode));
				gripper.gripAsync();
				gripper.move(ptp(getFrame("/Tiger_Woods_Swing_Bottom")).setJointVelocityRel(0.4).setMode(impedanceControlMode));
				gripper.releaseAsync();
			}
			
			
			getLogger().info("Move to zero position (point to the sky)");
			gripper.moveAsync(ptp(0,Math.toRadians(10),0,Math.toRadians(-30),0,Math.toRadians(30),0).setJointVelocityRel(0.2).setMode(impedanceControlMode).setBlendingRel(0.1));
			gripper.move(ptp(0,Math.toRadians(10),0,Math.toRadians(-30),0,Math.toRadians(30),0).setJointVelocityRel(0.2));
			gripper.move(ptp(0,0,0,0,0,0,0).setJointVelocityRel(0.2));
		}
		
		
		media_flange.setOutputX3Pin12(true);
		media_flange.setOutputX3Pin2(false); // yellow
		getLogger().info("End");
		
	}

	


}
