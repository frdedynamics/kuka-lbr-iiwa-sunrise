package application;

import javax.inject.Inject;
import java.util.concurrent.TimeUnit;

import com.kuka.grippertoolbox.api.gripper.AbstractGripper;
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



public class PingPongFeeder extends RoboticsAPIApplication {
	@Inject
	private AbstractGripper gripper;
	private LBR lbr;
	
	private static double vel_1 = 0.25;
	final double stiffnessTrans = 5000.0; // N
	final double stiffnessRot = 300.0; // Nm
	final double stiffnessNull = 5.0;
	
//	private static double[] startPosition=new double[]{Math.toRadians(-1.28), Math.toRadians(40.86), Math.toRadians(1.69), Math.toRadians(-83.23), Math.toRadians(1.77), Math.toRadians(-32.53), -0.57};
//	private static double[] feeder1_approach_pick = new double[]{};
//	private static double[] feeder1_pick = new double[]{};
//	private static double[] feeder1_approach_place = new double[]{};
//	
//	private static double[] feeder2_approach_pick = new double[]{};
//	private static double[] feeder2_pick = new double[]{};
//	private static double[] feeder2_approach_place = new double[]{};

	public void initialize() {
		lbr = getContext().getDeviceFromType(LBR.class);
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
		
		
		gripper.attachTo(lbr.getFlange());
		
		// move to forward starting pose
		getLogger().info("Moving to start position");
		//lbr.move(ptp(0, Math.toRadians(10), 0, Math.toRadians(-80), 0, Math.toRadians(90), 0));
		//lbr.move(ptp(startPosition));
		
//		lbr.move(ptp(getApplicationData().getFrame("/PickAndPlace1/Start")).setBlendingRel(.5).setJointVelocityRel(0.4));
		CartesianPTP ptpToStartPosition = ptp(getApplicationData().getFrame("/PingPongFeeder/startPosition"));
		ptpToStartPosition.setJointVelocityRel(vel_1);
		lbr.move(ptpToStartPosition);
		
		// set up impedance control
		// high translational/rotational stiffness, low null-space stiffness
		//getLogger().info("Hold position in impedance control mode");
		final CartesianImpedanceControlMode controlMode = new CartesianImpedanceControlMode();
		controlMode.parametrize(CartDOF.TRANSL).setStiffness(stiffnessTrans); //cartStiffnessTrans
		controlMode.parametrize(CartDOF.ROT).setStiffness(stiffnessRot); //cartStiffnessRot
		controlMode.setNullSpaceStiffness(stiffnessNull);
		//controlMode.setNullSpaceDamping(0.3);

		// hold impedance control until dialog is closed by user
//		final IMotionContainer motionContainer = lbr.moveAsync((new PositionHold(controlMode, -1, null)));
//		getLogger().info("Show modal dialog while executing position hold");
//		getApplicationUI().displayModalDialog(ApplicationDialogType.INFORMATION, "Press ok to finish the application.",
//				"OK");
//		motionContainer.cancel();
		
		Spline pickFeeder1_first = new Spline(
				spl(getApplicationData().getFrame("/PingPongFeeder/feeder1/approach_pick")),
				spl(getApplicationData().getFrame("/PingPongFeeder/feeder1/pick")));
//		MotionBatch pickFeeder1_first = new MotionBatch(
//				ptp(getApplicationData().getFrame("/PingPongFeeder/feeder1/approach_pick")),
//				lin(getApplicationData().getFrame("/PingPongFeeder/feeder1/pick")));
		pickFeeder1_first.setJointVelocityRel(vel_1);
		pickFeeder1_first.setMode(controlMode);
		
		Spline pickFeeder1 = new Spline(
				lin(getApplicationData().getFrame("/PingPongFeeder/feeder1/approach_place")),
				spl(getApplicationData().getFrame("/PingPongFeeder/feeder1/approach_pick")),
				lin(getApplicationData().getFrame("/PingPongFeeder/feeder1/pick")));	
		pickFeeder1.setJointVelocityRel(vel_1);
		pickFeeder1.setMode(controlMode);
		
		Spline placeFeeder1 = new Spline(
				lin(getApplicationData().getFrame("/PingPongFeeder/feeder2/approach_pick")),
				spl(getApplicationData().getFrame("/PingPongFeeder/feeder1/approach_place")),
				lin(getApplicationData().getFrame("/PingPongFeeder/feeder1/place")));	
		placeFeeder1.setJointVelocityRel(vel_1);
		placeFeeder1.setMode(controlMode);
		
		Spline pickFeeder2 = new Spline(
				lin(getApplicationData().getFrame("/PingPongFeeder/feeder2/approach_place")),
				spl(getApplicationData().getFrame("/PingPongFeeder/feeder2/approach_pick")),
				lin(getApplicationData().getFrame("/PingPongFeeder/feeder2/pick")));	
		pickFeeder2.setJointVelocityRel(vel_1);
		pickFeeder2.setMode(controlMode);
		
		Spline placeFeeder2 = new Spline(
				lin(getApplicationData().getFrame("/PingPongFeeder/feeder1/approach_pick")),
				spl(getApplicationData().getFrame("/PingPongFeeder/feeder2/approach_place")),
				spl(getApplicationData().getFrame("/PingPongFeeder/feeder2/place")));	
		placeFeeder2.setJointVelocityRel(vel_1);
		placeFeeder2.setMode(controlMode);
		
		
		int runLoop = 0;
		
		while(runLoop <= 4) {
			if (runLoop == 0){
				lbr.move(pickFeeder1_first);
//				lbr.move(ptp(getApplicationData().getFrame("/PingPongFeeder/feeder1/approach_pick")).setMode(controlMode).setBlendingRel(0.3).setJointVelocityRel(vel_1));
//				lbr.move(lin(getApplicationData().getFrame("/PingPongFeeder/feeder1/pick")).setMode(controlMode).setJointVelocityRel(vel_1));
				
			}
			else lbr.move(pickFeeder1);
			gripper.gripAsync();
			
			lbr.move(placeFeeder2);
//			lbr.move(lin(getApplicationData().getFrame("/PingPongFeeder/feeder1/approach_pick")).setMode(controlMode).setBlendingRel(0.3).setJointVelocityRel(vel_1));
//			lbr.move(ptp(getApplicationData().getFrame("/PingPongFeeder/feeder2/approach_place")).setMode(controlMode).setBlendingRel(0.3).setJointVelocityRel(vel_1));
//			lbr.move(lin(getApplicationData().getFrame("/PingPongFeeder/feeder2/place")).setMode(controlMode).setJointVelocityRel(vel_1));
			gripper.releaseAsync();
			
			lbr.move(pickFeeder2);
			gripper.gripAsync();
			
			lbr.move(placeFeeder1);
			gripper.releaseAsync();
			
			runLoop++;
		}
		
		
		
		
		
//		Frame startFrame = lbr.getCurrentCartesianPosition(gripper.getDefaultMotionFrame()); 
//		//Frame gripperFrame = gripper.getDefaultMotionFrame();
//		Frame firstFrame=(new Frame(startFrame)).setY(startFrame.getY() + 300);
//		Frame secondFrame=(new Frame(startFrame)).setY(startFrame.getY() - 300);
//		
//		int runLoop = 0;
//		
//		while(runLoop <= 4) {
//			LIN linToFirstFrame = lin(firstFrame);
//			linToFirstFrame.setJointVelocityRel(0.5);
//			linToFirstFrame.setMode(controlMode);
//			gripper.move(linToFirstFrame);
//			gripper.gripAsync();
//			
//			LIN linToSecondFrame = lin(secondFrame);
//			linToSecondFrame.setJointVelocityRel(0.5);
//			linToSecondFrame.setMode(controlMode);
//			gripper.move(linToSecondFrame);
//			gripper.releaseAsync();
//			
//			runLoop++;
//		}
		
		
		
		
		
		getLogger().info("App finished");
	}
}
