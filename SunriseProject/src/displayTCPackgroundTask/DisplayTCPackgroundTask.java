package displayTCPackgroundTask;


import javax.inject.Inject;
import java.util.concurrent.TimeUnit;

import com.kuka.grippertoolbox.api.gripper.AbstractGripper;
import com.kuka.roboticsAPI.applicationModel.tasks.CycleBehavior;
import com.kuka.roboticsAPI.applicationModel.tasks.RoboticsAPICyclicBackgroundTask;
import com.kuka.roboticsAPI.deviceModel.LBR;
import com.kuka.roboticsAPI.geometricModel.Frame;

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
public class DisplayTCPackgroundTask extends RoboticsAPICyclicBackgroundTask {

	@Inject
	private LBR lbr;
	
	@Inject
	private AbstractGripper gripper;

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
		
		Frame currentFrame = lbr.getCurrentCartesianPosition(gripper.getDefaultMotionFrame());

		getApplicationData().getProcessData("TcpX").setValue(Math.round(currentFrame.getX())/1000.0);
		getApplicationData().getProcessData("TcpY").setValue(Math.round(currentFrame.getY())/1000.0);
		getApplicationData().getProcessData("TcpZ").setValue(Math.round(currentFrame.getZ())/1000.0);
		
		getApplicationData().getProcessData("TcpA").setValue(Math.round(100.0*currentFrame.getAlphaRad()*180.0/Math.PI)/100.0);
		getApplicationData().getProcessData("TcpB").setValue(Math.round(100.0*currentFrame.getBetaRad()*180.0/Math.PI)/100.0);
		getApplicationData().getProcessData("TcpC").setValue(Math.round(100.0*currentFrame.getGammaRad()*180.0/Math.PI)/100.0);

		getApplicationData().getProcessData("TcpE").setValue(lbr.getCurrentJointPosition().get(2)*180.0/Math.PI);
		
	}
}