package com.kuka.led;

import javax.inject.Inject;
import java.util.concurrent.TimeUnit;

import com.kuka.generated.ioAccess.MediaFlangeIOGroup;
import com.kuka.roboticsAPI.applicationModel.tasks.CycleBehavior;
import com.kuka.roboticsAPI.applicationModel.tasks.RoboticsAPICyclicBackgroundTask;
import com.kuka.roboticsAPI.controllerModel.Controller;
import com.kuka.roboticsAPI.controllerModel.ControllerStateListenerAdapter;
import com.kuka.roboticsAPI.controllerModel.DispatchedEventData;
import com.kuka.roboticsAPI.controllerModel.IControllerStateListener;
import com.kuka.roboticsAPI.controllerModel.StatePortData;
import com.kuka.roboticsAPI.controllerModel.sunrise.ISafetyState;
import com.kuka.roboticsAPI.controllerModel.sunrise.SunriseSafetyState.EmergencyStop;
import com.kuka.roboticsAPI.controllerModel.sunrise.SunriseSafetyState.EnablingDeviceState;
import com.kuka.roboticsAPI.controllerModel.sunrise.SunriseSafetyState.OperatorSafety;
import com.kuka.roboticsAPI.controllerModel.sunrise.SunriseSafetyState.SafetyStopType;
import com.kuka.roboticsAPI.deviceModel.Device;
import com.kuka.roboticsAPI.deviceModel.JointEnum;
import com.kuka.roboticsAPI.deviceModel.LBR;
import com.kuka.roboticsAPI.deviceModel.OperationMode;
import com.kuka.roboticsAPI.motionModel.controlModeModel.HandGuidingControlMode;
import com.kuka.sunrise.common.task.categories.BackgroundTaskCategory;
import com.kuka.common.ThreadUtil;

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
 * 
 * @see UseRoboticsAPIContext
 * 
 */
@BackgroundTaskCategory(autoStart = true)
public class LedTask extends RoboticsAPICyclicBackgroundTask {
	@Inject
	private Controller kUKA_Sunrise_Cabinet_1;
	@Inject
	private MediaFlangeIOGroup mfIOGroup;
	@Inject
	private LBR lbr;

	@Override
	public void initialize() {
		kUKA_Sunrise_Cabinet_1 = getController("KUKA_Sunrise_Cabinet_1");
		lbr = (LBR) getDevice(kUKA_Sunrise_Cabinet_1, "LBR_iiwa_14_R820_1");
		mfIOGroup = new MediaFlangeIOGroup(kUKA_Sunrise_Cabinet_1);
		// initialize your task here
		initializeCyclic(0, 500, TimeUnit.MILLISECONDS,
				CycleBehavior.BestEffort);
		//MediaFlangeIOGroup mfIOGroup = new MediaFlangeIOGroup(kUKA_Sunrise_Cabinet_1);

	}

	@Override
	public void runCyclic() {
		// your task execution starts here
	    if (lbr.isReadyToMove()) {
		 
			mfIOGroup.setLEDGreen(true);

		} else {
			mfIOGroup.setLEDGreen(false);
			mfIOGroup.setLEDRed(true);

		}

		if (lbr.getSafetyState().getEmergencyStopInt()
				.compareTo(EmergencyStop.ACTIVE) == 0
				|| lbr.getSafetyState().getEmergencyStopEx()
						.compareTo(EmergencyStop.ACTIVE) == 0) {
			mfIOGroup.setLEDRed(true);
		} else {
			mfIOGroup.setLEDRed(false);
		}
		if (lbr.getSafetyState().getEnablingDeviceState()
				.compareTo(EnablingDeviceState.HANDGUIDING) == 0
				&& lbr.getSafetyState().getSafetyStopSignal()
						.compareTo(SafetyStopType.NOSTOP) == 0) {
			mfIOGroup.setLEDBlue(true);
			mfIOGroup.setLEDGreen(false);
			mfIOGroup.setLEDRed(false);
		} else {
			mfIOGroup.setLEDBlue(false);
		}
		
	}
}