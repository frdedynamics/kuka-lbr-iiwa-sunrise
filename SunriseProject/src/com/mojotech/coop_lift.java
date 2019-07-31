package com.mojotech;

import javax.inject.Inject;

import com.kuka.roboticsAPI.applicationModel.RoboticsAPIApplication;
import static com.kuka.roboticsAPI.motionModel.BasicMotions.ptp;
import static com.kuka.roboticsAPI.motionModel.BasicMotions.lin;

import com.kuka.roboticsAPI.deviceModel.LBR;
import com.kuka.roboticsAPI.geometricModel.CartDOF;
import com.kuka.roboticsAPI.geometricModel.Frame;
import com.kuka.roboticsAPI.motionModel.IMotionContainer;
import com.kuka.roboticsAPI.motionModel.PTP;
import com.kuka.roboticsAPI.motionModel.PositionHold;
import com.kuka.roboticsAPI.motionModel.controlModeModel.CartesianImpedanceControlMode;
import com.kuka.roboticsAPI.sensorModel.ForceSensorData;
import com.kuka.roboticsAPI.sensorModel.TorqueSensorData;
import com.kuka.roboticsAPI.uiModel.ApplicationDialogType;

import com.kuka.generated.ioAccess.MediaFlangeIOGroup;
import com.kuka.grippertoolbox.api.gripper.AbstractGripper;
import com.kuka.grippertoolbox.api.state.GripperState;

public class coop_lift extends RoboticsAPIApplication {
	@Inject
	private AbstractGripper gripper;

	private LBR lbr;
	
	private static final int max_z_cartesian = 500;
	private static final int min_z_cartesian = 300;
	
	private static final int A6_joint_index = 5;
	
	private static final int stiffnessZ = 3000;
	private static final int stiffnessY = 3000;  // 
	private static final int stiffnessX = 3000;
	private static final int stiffnessA = 300;  // Z-axis
	private static final int stiffnessB = 25;  // Y-axis
	private static final int stiffnessC = 300;  // X-axis

	private MediaFlangeIOGroup media_flange;
	private static double[] startPosition=new double[]{0,Math.toRadians(50),0,Math.toRadians(-90),0,Math.toRadians(-50),Math.toRadians(60)};
	//private static Frame startCartesianPosition=new double[]{730, 0, 320, 0, Math.toRadians(90), 0};
	private static Frame startCartesianFrame = new Frame(730, 0, 320, 0, Math.toRadians(90), 0);
	// private static Frame topCartesianFrame = new Frame(730, 0, 390, 0, Math.toRadians(90), 0);
	// private static Frame bottomCartesianFrame = new Frame(730, 0, 310, 0, Math.toRadians(90), 0);
	

	public void initialize() {
		lbr = getContext().getDeviceFromType(LBR.class);
		media_flange = new MediaFlangeIOGroup(lbr.getController());
	}

	public void run() {
				
		gripper.attachTo(lbr.getFlange());

		getLogger().info("Show modal dialog and wait for user to confirm");
        int isCancel = getApplicationUI().displayModalDialog(ApplicationDialogType.QUESTION, "Open gripper?", "Yes", "No");
        if (isCancel == 1)
        {
        	getLogger().info("no then..");
        }else{
        	gripper.releaseAsync();
        }
        
        
		getLogger().info("Show modal dialog and wait for user to confirm");
        isCancel = getApplicationUI().displayModalDialog(ApplicationDialogType.QUESTION, "Ok to move to start?", "OK", "Cancel");
        if (isCancel == 1)
        {
            return;
        }

		getLogger().info("Move to start position");
		PTP ptpToStartPosition = ptp(startPosition);
		ptpToStartPosition.setJointVelocityRel(0.2);
		lbr.move(ptpToStartPosition);
		
		gripper.move(lin(startCartesianFrame).setJointVelocityRel(0.2));


		getLogger().info("Hold position in impedance control mode");
		CartesianImpedanceControlMode impedanceControlMode = new CartesianImpedanceControlMode();
		impedanceControlMode.parametrize(CartDOF.X).setStiffness(stiffnessX);
		impedanceControlMode.parametrize(CartDOF.Y).setStiffness(stiffnessY);
		impedanceControlMode.parametrize(CartDOF.Z).setStiffness(stiffnessZ);
		
		impedanceControlMode.parametrize(CartDOF.A).setStiffness(stiffnessA);
		impedanceControlMode.parametrize(CartDOF.B).setStiffness(stiffnessB);
		impedanceControlMode.parametrize(CartDOF.C).setStiffness(stiffnessC);
		
		impedanceControlMode.parametrize(CartDOF.B).setDamping(0.7);

		PositionHold hold_it = new PositionHold(impedanceControlMode, -1, null);
		
		// The robot is set to position hold and impedance control mode gets activated without a timeout. 
		IMotionContainer positionHoldContainer = gripper.moveAsync(hold_it);
		
		
		getLogger().info("You must click green button to grip.");
		while (!media_flange.getUserButton()){

		}
		gripper.gripAsync();
		gripper.waitForGripperState(GripperState.UNDEFINED);
		
		
		getLogger().info("Show modal dialog and wait for user to confirm");
        isCancel = getApplicationUI().displayModalDialog(ApplicationDialogType.QUESTION, "Get tare torque or die.", "Get it", "Die");
        if (isCancel == 1)
        {
            return;
        }
        TorqueSensorData tare_torque = lbr.getExternalTorque();
        getLogger().info("tare torque was: " + tare_torque.toString());
        ForceSensorData tare_ft = lbr.getExternalForceTorque(gripper.getDefaultMotionFrame());
        getLogger().info("tare ft was: " + tare_ft.toString());
        
        Frame tmp_cart_frame = lbr.getCurrentCartesianPosition(lbr.getFlange());
        getLogger().info("frame was: " + tmp_cart_frame.toString());
        getLogger().info(String.format("in degrees was: A: %s, B: %s, C: %s",
        		Double.toString(Math.toDegrees(tmp_cart_frame.getAlphaRad())), 
				Double.toString(Math.toDegrees(tmp_cart_frame.getBetaRad())), 
				Double.toString(Math.toDegrees(tmp_cart_frame.getGammaRad()))));
        

        double current_torque_A6 = lbr.getExternalTorque().getTorqueValues()[A6_joint_index] - tare_torque.getTorqueValues()[A6_joint_index];
        if (current_torque_A6 > 3 || current_torque_A6 < -3){
        	getLogger().info("current_torque_A6 is out of bounds: " + Double.toString(current_torque_A6));
        	return;
        }
        
        double current_force_downwards = lbr.getExternalForceTorque(gripper.getDefaultMotionFrame()).getForce().getX() - tare_ft.getForce().getX();
        if (current_force_downwards > 2 || current_force_downwards < -2){
        	getLogger().info("current_force_downwards is out of bounds: " + Double.toString(current_force_downwards));
        	return;
        }
        Frame current_command = lbr.getCommandedCartesianPosition(lbr.getFlange());
        Frame next_command = current_command.copyWithRedundancy();
        
		getLogger().info("Click green button to finish the application.");
		positionHoldContainer.cancel();
		while (!media_flange.getUserButton()){
			if (lbr.getCurrentCartesianPosition(lbr.getFlange()).getZ() < min_z_cartesian  || lbr.getCurrentCartesianPosition(lbr.getFlange()).getZ() > max_z_cartesian){
				getLogger().info("Out of bounds.");
				return;
			}
			
			current_torque_A6 = lbr.getExternalTorque().getTorqueValues()[A6_joint_index] - tare_torque.getTorqueValues()[A6_joint_index];
			current_force_downwards = lbr.getExternalForceTorque(gripper.getDefaultMotionFrame()).getForce().getX() - tare_ft.getForce().getX();
			current_command = lbr.getCommandedCartesianPosition(lbr.getFlange());
			next_command = current_command.copyWithRedundancy();
			
			
			if (current_force_downwards > 4){
				next_command.setZ(current_command.getZ() - 20);
			}else if (current_force_downwards < -4){
				next_command.setZ(current_command.getZ() + 20);
			}else{
				next_command.setZ(lbr.getCurrentCartesianPosition(lbr.getFlange()).getZ());
			}
			lbr.move(lin(next_command).setMode(impedanceControlMode).setJointVelocityRel(0.05));
						
		}

		// As soon as the modal dialog returns, the motion container will be cancelled. This finishes the position hold. 
		positionHoldContainer.cancel();
	}

}
