package com.gates;


import static com.kuka.roboticsAPI.motionModel.BasicMotions.ptp;

import javax.inject.Inject;
import com.kuka.roboticsAPI.applicationModel.RoboticsAPIApplication;
import com.kuka.roboticsAPI.deviceModel.LBR;
import com.kuka.roboticsAPI.geometricModel.CartDOF;
import com.kuka.roboticsAPI.geometricModel.Frame;
import com.kuka.roboticsAPI.motionModel.controlModeModel.CartesianImpedanceControlMode;

/**
 * Implementation of a robot application.
 * <p>
 * The application provides a {@link RoboticsAPITask#initialize()} and a 
 * {@link RoboticsAPITask#run()} method, which will be called successively in 
 * the application lifecycle. The application will terminate automatically after 
 * the {@link RoboticsAPITask#run()} method has finished or after stopping the 
 * task. The {@link RoboticsAPITask#dispose()} method will be called, even if an 
 * exception is thrown during initialization or run. 
 * <p>
 * <b>It is imperative to call <code>super.dispose()</code> when overriding the 
 * {@link RoboticsAPITask#dispose()} method.</b> 
 * 
 * @see UseRoboticsAPIContext
 * @see #initialize()
 * @see #run()
 * @see #dispose()
 */
public class testCartesianControl extends RoboticsAPIApplication {
	
	private static final int stiffnessZ = 2500;
	private static final int stiffnessY = 700;
	private static final int stiffnessX = 1500;
	
	private static final double jointVelocityRel = 0.05;

	@Inject
	private LBR lbr;

	@Override
	public void initialize() {
		// initialize your application here
		lbr = getContext().getDeviceFromType(LBR.class);
	}
	

	@Override
	public void run() {

		getLogger().info("Set impedance");
		CartesianImpedanceControlMode impedanceControlMode = new CartesianImpedanceControlMode();
		impedanceControlMode.parametrize(CartDOF.X).setStiffness(stiffnessX);
		impedanceControlMode.parametrize(CartDOF.Y).setStiffness(stiffnessY);
		impedanceControlMode.parametrize(CartDOF.Z).setStiffness(stiffnessZ);
		
		Frame currentCommand = lbr.getCommandedCartesianPosition(lbr.getFlange());
	    Frame commandFrame = currentCommand.copyWithRedundancy();

	    commandFrame.setX(-322.74);
	    commandFrame.setY(0.00);
	    commandFrame.setZ(973.94);
	    commandFrame.setAlphaRad(-0.00);
	    commandFrame.setBetaRad(-1.13);
	    commandFrame.setGammaRad(0.00);
	    
    	getLogger().info("Move to new commanded frame.");
	    lbr.move(ptp(commandFrame).setMode(impedanceControlMode).setJointVelocityRel(jointVelocityRel));
	    getLogger().info("End");
	    
	    
	}
}