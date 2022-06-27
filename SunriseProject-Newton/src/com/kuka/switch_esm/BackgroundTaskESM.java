package com.kuka.switch_esm;

import java.util.concurrent.TimeUnit;

import com.kuka.common.ThreadUtil;
import com.kuka.roboticsAPI.applicationModel.tasks.CycleBehavior;
import com.kuka.roboticsAPI.applicationModel.tasks.RoboticsAPICyclicBackgroundTask;
import com.kuka.roboticsAPI.controllerModel.Controller;
import com.kuka.roboticsAPI.deviceModel.LBR;
import com.kuka.roboticsAPI.uiModel.userKeys.IUserKey;
import com.kuka.roboticsAPI.uiModel.userKeys.IUserKeyBar;
import com.kuka.roboticsAPI.uiModel.userKeys.IUserKeyListener;
import com.kuka.roboticsAPI.uiModel.userKeys.UserKeyAlignment;
import com.kuka.roboticsAPI.uiModel.userKeys.UserKeyEvent;
import com.kuka.roboticsAPI.uiModel.userKeys.UserKeyLED;
import com.kuka.roboticsAPI.uiModel.userKeys.UserKeyLEDSize;
import com.kuka.sunrise.common.task.categories.BackgroundTaskCategory;

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
public class BackgroundTaskESM extends RoboticsAPICyclicBackgroundTask {
	private Controller controller;
	private LBR lbr;

	public void initialize() {
		controller = getController("KUKA_Sunrise_Cabinet_1");
		lbr = (LBR) getDevice(controller, "LBR_iiwa_14_R820_1");
		switchESMStates();
		initializeCyclic(0, 1000, TimeUnit.MILLISECONDS,
				CycleBehavior.BestEffort);
	}

	public void runCyclic() {
		
	}

	public void switchESMStates() {
		lbr.setESMState("4");
		IUserKeyBar keyBar = getApplicationUI().createUserKeyBar("ESMStates");

		IUserKeyListener listener1 = new IUserKeyListener() {

			@Override
			public void onKeyEvent(IUserKey key, UserKeyEvent event) {
				if (0 == key.getSlot()) {
					lbr.setESMState("1");
					}				
				if (1 == key.getSlot()) {
					lbr.setESMState("2");
				}
				if (2 == key.getSlot()) {
					lbr.setESMState("3");
				}
				if (3 == key.getSlot()) {
					lbr.setESMState("4");;
				}
			}
		};

		IUserKey button0 = keyBar.addUserKey(0, listener1, true);
		IUserKey button1 = keyBar.addUserKey(1, listener1, true);
		IUserKey button2 = keyBar.addUserKey(2, listener1, true);
		IUserKey button3 = keyBar.addUserKey(3, listener1, true);

		button0.setText(UserKeyAlignment.TopMiddle, "ESM1");
		button1.setText(UserKeyAlignment.TopMiddle, "ESM2");
		button2.setText(UserKeyAlignment.TopMiddle, "ESM3");
		button3.setText(UserKeyAlignment.TopMiddle, "ESM4");


		keyBar.publish();

	}
}
