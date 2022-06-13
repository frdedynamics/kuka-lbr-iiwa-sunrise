package application;

import javax.inject.Inject;

import com.kuka.grippertoolbox.api.gripper.AbstractGripper;
import com.kuka.roboticsAPI.applicationModel.RoboticsAPIApplication;
import com.kuka.roboticsAPI.deviceModel.LBR;

public class Gripper extends RoboticsAPIApplication {
	@Inject
	private AbstractGripper gripper;
	private LBR lbr;
	
	public void initialize() {
		lbr = getContext().getDeviceFromType(LBR.class);
		boolean gripper_init = gripper.initialize();
		if (gripper_init) {
			getLogger().info("Gripper initialized");
		}
		else {
			getLogger().info("Gripper initialization failed");
		}
	}

	public void run() {
		gripper.attachTo(lbr.getFlange());
		gripper.gripAsync();
		gripper.releaseAsync();
		
	}
	
}
