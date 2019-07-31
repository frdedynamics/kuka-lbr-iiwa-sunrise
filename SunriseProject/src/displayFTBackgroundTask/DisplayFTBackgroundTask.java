package displayFTBackgroundTask;


import javax.inject.Inject;
import java.util.concurrent.TimeUnit;

import com.kuka.grippertoolbox.api.gripper.AbstractGripper;
import com.kuka.roboticsAPI.applicationModel.tasks.CycleBehavior;
import com.kuka.roboticsAPI.applicationModel.tasks.RoboticsAPICyclicBackgroundTask;
import com.kuka.roboticsAPI.deviceModel.LBR;
import com.kuka.roboticsAPI.sensorModel.ForceSensorData;

/**
 * Implementation of a cyclic background task.
 * <p>
 * It provides the {@link RoboticsAPICyclicBackgroundTask#runCyclic} method 
 * which will be called cyclically with the specified period.<br>
 * Cycle period and initial delay can be set by calling 
 * {@link RoboticsAPICyclicBackgroundTask#initializeCyclic} method in the 
 * {@link RoboticsAPIBackgroundTask#initialize()} method of the inheriting 
 * class.<br>
 * The cyclic background task can be terminated via 
 * {@link RoboticsAPICyclicBackgroundTask#getCyclicFuture()#cancel()} method or 
 * stopping of the task.
 * @see UseRoboticsAPIContext
 * 
 */
public class DisplayFTBackgroundTask extends RoboticsAPICyclicBackgroundTask {

	@Inject
	private LBR lbr;
	
	@Inject
	private AbstractGripper gripper;
	
	private long currentTimeMillis = System.currentTimeMillis();
	private long previousTimeMillis = System.currentTimeMillis() - 100;
	private double deltaTimeSeconds = 0.1;

	@Override
	public void initialize() {
		// initialize your task here
		initializeCyclic(0, 100, TimeUnit.MILLISECONDS,
				CycleBehavior.BestEffort);
		
		lbr = getContext().getDeviceFromType(LBR.class);
		gripper.attachTo(lbr.getFlange());
	}

	@Override
	public void runCyclic() {
		// your task execution starts here
		currentTimeMillis = System.currentTimeMillis();
		deltaTimeSeconds = ((currentTimeMillis - previousTimeMillis)/1000.0);
		previousTimeMillis = currentTimeMillis;
		
		getApplicationData().getProcessData("dt").setValue(deltaTimeSeconds);
		
		ForceSensorData ft = lbr.getExternalForceTorque(gripper.getDefaultMotionFrame());
		
		getApplicationData().getProcessData("Fx").setValue(ft.getForce().getX());
		getApplicationData().getProcessData("Fy").setValue(ft.getForce().getY());
		getApplicationData().getProcessData("Fz").setValue(ft.getForce().getZ());
		
		getApplicationData().getProcessData("Taux").setValue(ft.getTorque().getX());
		getApplicationData().getProcessData("Tauy").setValue(ft.getTorque().getY());
		getApplicationData().getProcessData("Tauz").setValue(ft.getTorque().getZ());
	}
}