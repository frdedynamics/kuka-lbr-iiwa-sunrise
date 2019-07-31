package grippedBackgroundTask;


import javax.inject.Inject;
import java.util.concurrent.TimeUnit;

import com.kuka.grippertoolbox.api.gripper.AbstractGripper;
import com.kuka.grippertoolbox.api.state.GripperState;
import com.kuka.roboticsAPI.applicationModel.tasks.CycleBehavior;
import com.kuka.roboticsAPI.applicationModel.tasks.RoboticsAPICyclicBackgroundTask;

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
public class GrippedBackgroundTask extends RoboticsAPICyclicBackgroundTask {
	
	@Inject
	private AbstractGripper gripper;
	
	private boolean gripped_workpiece_estimate_state = false;
	private long currentTimeMillis = System.currentTimeMillis();
	private GripperState currentGripperState;
	
	private static boolean isClosing = false;
	private static long statusDelayMillis = 500;
	private static long closingTimeMillis = System.currentTimeMillis();
	private static GripperState previousGripperState;
	
	@Override
	public void initialize() {
		initializeCyclic(0, 100, TimeUnit.MILLISECONDS,
				CycleBehavior.BestEffort);
	}

	@Override
	public void runCyclic() {
		// your task execution starts here
		
		currentTimeMillis = System.currentTimeMillis();
		currentGripperState = gripper.getGripperState();

		if (currentGripperState == GripperState.GRIPPED || currentGripperState == GripperState.RELEASED || currentGripperState == GripperState.GRIPPED_ITEM){
			isClosing = false;
		}
		if (currentGripperState == GripperState.UNDEFINED){
			if (previousGripperState == GripperState.RELEASED){
				isClosing = true;
				closingTimeMillis = currentTimeMillis;
			}
		}
		
		// delay true state
		if (isClosing && (currentTimeMillis-closingTimeMillis) > statusDelayMillis){
			gripped_workpiece_estimate_state = true;
		}else{
			gripped_workpiece_estimate_state = false;
		}
		
		
		getApplicationData().getProcessData("grpdWpcs").setValue(gripped_workpiece_estimate_state);
		
		previousGripperState = currentGripperState;
	}
}