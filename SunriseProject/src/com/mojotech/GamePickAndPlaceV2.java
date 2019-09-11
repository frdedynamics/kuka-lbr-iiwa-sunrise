package com.mojotech;


import java.util.ArrayList;
import java.util.ListIterator;

import javax.inject.Inject;

import com.kuka.common.params.IParameterSet;
import com.kuka.generated.ioAccess.MediaFlangeIOGroup;
import com.kuka.grippertoolbox.api.gripper.AbstractGripper;
import com.kuka.roboticsAPI.applicationModel.RoboticsAPIApplication;

import static com.kuka.roboticsAPI.motionModel.BasicMotions.ptp;

import com.kuka.roboticsAPI.deviceModel.LBR;
import com.kuka.roboticsAPI.geometricModel.CartDOF;
import com.kuka.roboticsAPI.geometricModel.Frame;
import com.kuka.roboticsAPI.motionModel.IMotionContainer;
import com.kuka.roboticsAPI.motionModel.PositionHold;
import com.kuka.roboticsAPI.motionModel.controlModeModel.CartesianImpedanceControlMode;
import com.kuka.roboticsAPI.uiModel.ApplicationDialogType;

public class GamePickAndPlaceV2 extends RoboticsAPIApplication {
	
	@Inject
	private AbstractGripper gripper;
	
	private LBR lbr;
	private MediaFlangeIOGroup media_flange;
	
	private static final double relSpeed = 0.2;
	private static final int mediumStiffness = 300;

	final static double offsetAxis2And4=Math.toRadians(10);
	private static double[] startPosition=new double[]{0,offsetAxis2And4,0,offsetAxis2And4-Math.toRadians(90),0,Math.toRadians(90),0};
	


	private final static String informationText=
			"Would you like to play a game?"+ "\n" +
			"\n" +
			"You will guide the robot by hand." + "\n" +
			"You will guide the gripper by hand." + "\n" +
			"Click the green button to store positions." + "\n" +
			"\n" +
			"Long press the green button to replay our positions.";

	public void initialize() {		
		lbr = getContext().getDeviceFromType(LBR.class);
		media_flange = new MediaFlangeIOGroup(lbr.getController());
		gripper.attachTo(lbr.getFlange());
		
		media_flange.setLEDBlue(false);
		media_flange.setOutputX3Pin12(true);
		media_flange.setOutputX3Pin2(false); // yellow
	}

	public void run() {		

		getLogger().info("Show modal dialog and wait for user to confirm");

        int isCancel = getApplicationUI().displayModalDialog(ApplicationDialogType.QUESTION, informationText, "OK", "Cancel");
        if (isCancel == 1)
        {
            return;
        }
        getLogger().info("Open gripper");
        gripper.releaseAsync();
        
		getLogger().info("Set medium impedance");
		media_flange.setLEDBlue(false);
		media_flange.setOutputX3Pin12(true);
		media_flange.setOutputX3Pin2(true); // red

		CartesianImpedanceControlMode impedanceControlMode = new CartesianImpedanceControlMode();
		impedanceControlMode.parametrize(CartDOF.ALL).setStiffness(mediumStiffness);
		impedanceControlMode.setNullSpaceDamping(0.7);
		impedanceControlMode.setNullSpaceStiffness(90);
		
		getLogger().info("Move to start position.");	
		gripper.move(ptp(startPosition).setJointVelocityRel(relSpeed).setMode(impedanceControlMode));
		
		media_flange.setOutputX3Pin12(false);
		media_flange.setOutputX3Pin2(true); // green
		isCancel = getApplicationUI().displayModalDialog(ApplicationDialogType.QUESTION, "Grab the robot!", "OK", "Cancel");
        if (isCancel == 1)
        {
            return;
        }
        		
        getLogger().info("De-activate gripper power.");
        media_flange.setOutputX3Pin1(false);
        media_flange.setOutputX3Pin11(false);
        
		getLogger().info("Long press green button to end, click to store position.");
		
		boolean previousGetUserButton = media_flange.getUserButton();
		boolean currentGetUserButton = media_flange.getUserButton();
		
		long clickedTimeMillis = (long) (System.nanoTime()/1e6);
		long nowTimeMillis = (long) (System.nanoTime()/1e6);
		
		ArrayList<Frame> recordedFrames = new ArrayList<Frame>();
		ArrayList<Boolean> recordedGripStates = new ArrayList<Boolean>();
		
		impedanceControlMode.parametrize(CartDOF.ALL).setStiffness(0);
		PositionHold posHold = new PositionHold(impedanceControlMode, -1, null);
		
		IParameterSet posHoldParams = posHold.getParams();
		IMotionContainer positionHoldContainer = lbr.moveAsync(posHold);
		getLogger().info(posHoldParams.toString());
		while (true){
			
			//if (lbr.getSafetyState().getEnablingDeviceState() == EnablingDeviceState.NONE ){
				nowTimeMillis = (long) (System.nanoTime()/1e6);
				currentGetUserButton = media_flange.getUserButton();
				
				if (currentGetUserButton && !previousGetUserButton){ // button clicked
					clickedTimeMillis = (long) (System.nanoTime()/1e6);	
				}
				
				if (currentGetUserButton && previousGetUserButton && (nowTimeMillis - clickedTimeMillis > 2e3) ){ // button pressed
					getLogger().info("Long pressed, ending..");
					break;
				}
				
				if (!currentGetUserButton && previousGetUserButton){ // button released
					getLogger().info(lbr.getCurrentCartesianPosition(gripper.getDefaultMotionFrame()).toString());
					recordedFrames.add(lbr.getCurrentCartesianPosition(gripper.getDefaultMotionFrame()));
					recordedGripStates.add(media_flange.getInputX3Pin16());
					
					media_flange.setLEDBlue(!media_flange.getInputX3Pin16());
				}
				
			//}
			
			previousGetUserButton = currentGetUserButton;
		}
		
		positionHoldContainer.cancel();
		impedanceControlMode.parametrize(CartDOF.ALL).setStiffness(mediumStiffness);
		
		// replay, maybe backwards?
        gripper.releaseAsync();
		getLogger().info("Move to start position.");
		media_flange.setLEDBlue(false);
		media_flange.setOutputX3Pin12(true);
		media_flange.setOutputX3Pin2(true); // red
		
		gripper.move(ptp(startPosition).setJointVelocityRel(relSpeed).setMode(impedanceControlMode));
		
		ListIterator<Frame> frameIter = recordedFrames.listIterator(recordedFrames.size());
		ListIterator<Boolean> gripStateIter = recordedGripStates.listIterator(recordedGripStates.size());
		
		while (frameIter.hasPrevious()){ // try previous for backwards
			gripper.move(ptp(frameIter.previous()).setJointVelocityRel(relSpeed).setMode(impedanceControlMode));
			if (gripStateIter.previous()){
				gripper.releaseAsync();
				media_flange.setLEDBlue(false);
			}else{
				gripper.gripAsync();
				media_flange.setLEDBlue(true);
			}
		}
		
		gripper.move(ptp(startPosition).setJointVelocityRel(relSpeed).setMode(impedanceControlMode));
		gripper.releaseAsync();
		media_flange.setLEDBlue(false);
		
		media_flange.setOutputX3Pin12(true);
		media_flange.setOutputX3Pin2(false); // yellow
		getLogger().info("End");
		
	}

	


}
