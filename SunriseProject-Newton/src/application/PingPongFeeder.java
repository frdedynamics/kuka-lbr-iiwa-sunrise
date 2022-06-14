package application;

import javax.inject.Inject;

import com.kuka.grippertoolbox.api.gripper.AbstractGripper;
import com.kuka.grippertoolbox.api.state.GripperState;
import com.kuka.roboticsAPI.applicationModel.RoboticsAPIApplication;
import static com.kuka.roboticsAPI.motionModel.BasicMotions.*;

import com.kuka.roboticsAPI.deviceModel.LBR;
import com.kuka.roboticsAPI.geometricModel.CartDOF;
import com.kuka.roboticsAPI.geometricModel.Frame;
import com.kuka.roboticsAPI.motionModel.controlModeModel.CartesianImpedanceControlMode;
import com.kuka.roboticsAPI.motionModel.IMotionContainer;
import com.kuka.roboticsAPI.motionModel.LIN;
import com.kuka.roboticsAPI.motionModel.PTP;
import com.kuka.roboticsAPI.motionModel.CartesianPTP;
import com.kuka.roboticsAPI.motionModel.PositionHold;
import com.kuka.roboticsAPI.uiModel.ApplicationDialogType;
import com.kuka.roboticsAPI.motionModel.Spline;
import com.kuka.roboticsAPI.motionModel.SPL;
import com.kuka.roboticsAPI.motionModel.MotionBatch;
import com.kuka.roboticsAPI.motionModel.RobotMotion;
import com.kuka.roboticsAPI.uiModel.ApplicationDialogType;



public class PingPongFeeder extends RoboticsAPIApplication {
	@Inject
	private AbstractGripper gripper;
	private LBR lbr;
	
	private static double vel_1 = 0.25;
	final double stiffnessTrans = 5000.0; // N
	final double stiffnessRot = 300.0; // Nm
	final double stiffnessNull = 5.0;
	
	public void initialize() {
		lbr = getContext().getDeviceFromType(LBR.class);
		
		// Initializing the gripper and ensuring it is in released state
		boolean gripper_init = gripper.initialize();
		if (gripper_init) {
			getLogger().info("Gripper initialized");
		}
		else {
			getLogger().info("Gripper initialization failed");
		}
		gripper.releaseAsync();
	}

	public void run() {
		// Extend the robot transformation chain with the gripper frame		
		gripper.attachTo(lbr.getFlange());
		
		// Move to starting pose
		getLogger().info("Moving to start position");
		CartesianPTP ptpToStartPosition = ptp(getApplicationData().getFrame("/PingPongFeeder/startPosition"));
		ptpToStartPosition.setJointVelocityRel(vel_1);
		lbr.move(ptpToStartPosition);
		
		// Set up impedance control
		// High translational/rotational stiffness, low null-space stiffness
		final CartesianImpedanceControlMode controlMode = new CartesianImpedanceControlMode();
		controlMode.parametrize(CartDOF.TRANSL).setStiffness(stiffnessTrans); //cartStiffnessTrans
		controlMode.parametrize(CartDOF.ROT).setStiffness(stiffnessRot); //cartStiffnessRot
		controlMode.setNullSpaceStiffness(stiffnessNull);
		//controlMode.setNullSpaceDamping(0.3);
		
		
		RobotMotion pickFeeder1_first;
		RobotMotion pickFeeder1;
		RobotMotion placeFeeder1;
		RobotMotion pickFeeder2;
		RobotMotion placeFeeder2;
		//int response = getApplicationUI().displayModalDialog(ApplicationDialogType.QUESTION, "Do you want to use spline movements", "Yes", "No");
		//Currently the non spline movements are not working correctly therefore we only allow the spline movements
		int response = 0;
		if (response == 0) {
			pickFeeder1_first = new Spline(
					spl(getApplicationData().getFrame("/PingPongFeeder/feeder1/approach_pick")),
					spl(getApplicationData().getFrame("/PingPongFeeder/feeder1/pick")));
			pickFeeder1 = new Spline(
					lin(getApplicationData().getFrame("/PingPongFeeder/feeder1/approach_place")),
					spl(getApplicationData().getFrame("/PingPongFeeder/feeder1/approach_pick")),
					lin(getApplicationData().getFrame("/PingPongFeeder/feeder1/pick")));
			placeFeeder1 = new Spline(
					lin(getApplicationData().getFrame("/PingPongFeeder/feeder2/approach_pick")),
					spl(getApplicationData().getFrame("/PingPongFeeder/feeder1/approach_place")),
					lin(getApplicationData().getFrame("/PingPongFeeder/feeder1/place")));
			pickFeeder2 = new Spline(
					lin(getApplicationData().getFrame("/PingPongFeeder/feeder2/approach_place")),
					spl(getApplicationData().getFrame("/PingPongFeeder/feeder2/approach_pick")),
					lin(getApplicationData().getFrame("/PingPongFeeder/feeder2/pick")));
			placeFeeder2 = new Spline(
					lin(getApplicationData().getFrame("/PingPongFeeder/feeder1/approach_pick")),
					spl(getApplicationData().getFrame("/PingPongFeeder/feeder2/approach_place")),
					spl(getApplicationData().getFrame("/PingPongFeeder/feeder2/place")));	
		}
		else {
			pickFeeder1_first = new MotionBatch(
				ptp(getApplicationData().getFrame("/PingPongFeeder/feeder1/approach_pick")),
				lin(getApplicationData().getFrame("/PingPongFeeder/feeder1/pick")));
			pickFeeder1 = new MotionBatch(
					lin(getApplicationData().getFrame("/PingPongFeeder/feeder1/approach_place")),
					ptp(getApplicationData().getFrame("/PingPongFeeder/feeder1/approach_pick")),
					lin(getApplicationData().getFrame("/PingPongFeeder/feeder1/pick")));
			placeFeeder1 = new MotionBatch(
					lin(getApplicationData().getFrame("/PingPongFeeder/feeder2/approach_pick")),
					ptp(getApplicationData().getFrame("/PingPongFeeder/feeder1/approach_place")),
					lin(getApplicationData().getFrame("/PingPongFeeder/feeder1/place")));
			pickFeeder2 = new MotionBatch(
					lin(getApplicationData().getFrame("/PingPongFeeder/feeder2/approach_place")),
					ptp(getApplicationData().getFrame("/PingPongFeeder/feeder2/approach_pick")),
					lin(getApplicationData().getFrame("/PingPongFeeder/feeder2/pick")));
			placeFeeder2 = new MotionBatch(
					lin(getApplicationData().getFrame("/PingPongFeeder/feeder1/approach_pick")),
					spl(getApplicationData().getFrame("/PingPongFeeder/feeder2/approach_place")),
					spl(getApplicationData().getFrame("/PingPongFeeder/feeder2/place")));
		}

		pickFeeder1_first.setJointVelocityRel(vel_1);
		pickFeeder1_first.setMode(controlMode);
		pickFeeder1.setJointVelocityRel(vel_1);
		pickFeeder1.setMode(controlMode);
		placeFeeder1.setJointVelocityRel(vel_1);
		placeFeeder1.setMode(controlMode);	
		pickFeeder2.setJointVelocityRel(vel_1);
		pickFeeder2.setMode(controlMode);
		placeFeeder2.setJointVelocityRel(vel_1);
		placeFeeder2.setMode(controlMode);
		
		
		int runLoop = 0;
		
		while(runLoop <= 4) {
			if (runLoop == 0){
				lbr.move(pickFeeder1_first);
			}
			else lbr.move(pickFeeder1);
			gripper.gripAsync();
			if (gripper.getGripperState() != GripperState.GRIPPED && gripper.getGripperState() != GripperState.GRIPPED_ITEM) getLogger().info("Waiting for Grip");
			while (gripper.getGripperState() != GripperState.GRIPPED && gripper.getGripperState() != GripperState.GRIPPED_ITEM);
			
			lbr.move(placeFeeder2);
			gripper.releaseAsync();
			
			lbr.move(pickFeeder2);
			gripper.gripAsync();
			if (gripper.getGripperState() != GripperState.GRIPPED && gripper.getGripperState() != GripperState.GRIPPED_ITEM) getLogger().info("Waiting for Grip");
			while (gripper.getGripperState() != GripperState.GRIPPED && gripper.getGripperState() != GripperState.GRIPPED_ITEM);
			
			lbr.move(placeFeeder1);
			gripper.releaseAsync();
			
			runLoop++;
		}		
		
		getLogger().info("App finished");
	}
}
