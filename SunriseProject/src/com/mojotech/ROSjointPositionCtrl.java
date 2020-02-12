package com.mojotech;


import java.util.concurrent.TimeUnit;
import java.util.concurrent.TimeoutException;

import javax.inject.Inject;

import com.kuka.common.ThreadUtil;
import com.kuka.connectivity.fastRobotInterface.ClientCommandMode;
import com.kuka.connectivity.fastRobotInterface.FRIConfiguration;
import com.kuka.connectivity.fastRobotInterface.FRIJointOverlay;
import com.kuka.connectivity.fastRobotInterface.FRISession;
import com.kuka.generated.ioAccess.MediaFlangeIOGroup;
import com.kuka.grippertoolbox.api.gripper.AbstractGripper;
import com.kuka.roboticsAPI.applicationModel.RoboticsAPIApplication;
import static com.kuka.roboticsAPI.motionModel.BasicMotions.*;

import com.kuka.roboticsAPI.deviceModel.LBR;
import com.kuka.roboticsAPI.motionModel.IMotionContainer;
import com.kuka.roboticsAPI.motionModel.PTP;
import com.kuka.roboticsAPI.motionModel.PositionHold;
//import com.kuka.roboticsAPI.motionModel.controlModeModel.JointImpedanceControlMode;
import com.kuka.roboticsAPI.motionModel.controlModeModel.PositionControlMode;
import com.kuka.roboticsAPI.uiModel.ApplicationDialogType;

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
public class ROSjointPositionCtrl extends RoboticsAPIApplication
{
	@Inject
	private AbstractGripper gripper;
	
    private String _clientName;
    
    final static double offsetAxis2And4=Math.toRadians(10);
	//private static double[] startPosition=new double[]{0,offsetAxis2And4,0,offsetAxis2And4-Math.toRadians(90),0,Math.toRadians(90),0};
	//private static double[] startPosition=new double[]{0,0,0,0,0,0,Math.toRadians(2)};
    
    @Inject
    private LBR lbr;
    
    @Inject
    private MediaFlangeIOGroup mediaFlange;
    
    @Override
    public void initialize()
    {
        _clientName = "192.170.10.86";
        mediaFlange = new MediaFlangeIOGroup(lbr.getController());
        gripper.attachTo(lbr.getFlange());
        mediaFlange.setLEDBlue(true);
        gripper.releaseAsync();
    }

    @Override
    public void run()
    {
        FRIConfiguration friConfiguration = FRIConfiguration.createRemoteConfiguration(lbr, _clientName);
        friConfiguration.setSendPeriodMilliSec(1);
        friConfiguration.setReceiveMultiplier(1);

        //friConfiguration.registerIO(friGroup.getInput("In_Bool_Clock_Enabled"));
        //friConfiguration.registerIO(friGroup.getOutput("Out_Bool_Enable_Clock"));
        //friConfiguration.registerIO(friGroup.getOutput("Out_Integer_Seconds"));
        //friConfiguration.registerIO(friGroup.getOutput("Out_Analog_Deci_Seconds"));
        
        //friConfiguration.registerIO(mediaFlange.getInput("InputX3Pin3"));
        //friConfiguration.registerIO(mediaFlange.getInput("InputX3Pin4"));
        friConfiguration.registerIO(mediaFlange.getInput("InputX3Pin10")); // Gripped without workpiece
        //friConfiguration.registerIO(mediaFlange.getInput("InputX3Pin13"));
        friConfiguration.registerIO(mediaFlange.getInput("InputX3Pin16")); // Released
        friConfiguration.registerIO(mediaFlange.getInput("UserButton"));
        friConfiguration.registerIO(mediaFlange.getOutput("LEDBlue")); // setLEDBlue
        //friConfiguration.registerIO(mediaFlange.getOutput("SwitchOffX3Voltage"));
        friConfiguration.registerIO(mediaFlange.getOutput("OutputX3Pin1")); // Grip
        friConfiguration.registerIO(mediaFlange.getOutput("OutputX3Pin2")); // Color
        friConfiguration.registerIO(mediaFlange.getOutput("OutputX3Pin11")); // Release
        friConfiguration.registerIO(mediaFlange.getOutput("OutputX3Pin12")); // Color
        
        getLogger().info("Creating FRI connection to " + friConfiguration.getHostName());
        getLogger().info("SendPeriod: " + friConfiguration.getSendPeriodMilliSec() + "ms |"
                + " ReceiveMultiplier: " + friConfiguration.getReceiveMultiplier());

        FRISession friSession = new FRISession(friConfiguration);
        FRIJointOverlay jointOverlay = new FRIJointOverlay(friSession, ClientCommandMode.POSITION);
        
        // wait until FRI session is ready to switch to command mode
        try
        {
            friSession.await(20, TimeUnit.SECONDS);
            getLogger().info("Connection to Client established");
        }
        catch (final TimeoutException e)
        {
            getLogger().error(e.getLocalizedMessage());
            friSession.close();
            return;
        }
        
        getLogger().info("Move to start position");
		PTP ptpToStartPosition = ptp(lbr.getCommandedJointPosition());
		ptpToStartPosition.setJointVelocityRel(0.25);
		lbr.move(ptpToStartPosition);
		ptpToStartPosition.setJointVelocityRel(1.0);
		
        PositionControlMode ctrMode = new PositionControlMode();
		
        ThreadUtil.milliSleep(5000);
                
        getLogger().info("do something ...");
        
        getLogger().info("Show modal dialog..");
        int isCancel = getApplicationUI().displayModalDialog(ApplicationDialogType.QUESTION, "Stawp?", "Yes", "Never!");
        if (isCancel == 0)
        {
        	getLogger().info("Close connection to client");
            friSession.close();
            return;
        }
        
        IMotionContainer positionHoldContainer = lbr.moveAsync((new PositionHold(ctrMode, -1, null)).addMotionOverlay(jointOverlay));

		getLogger().info("Show modal dialog while executing position hold");
		getApplicationUI().displayModalDialog(ApplicationDialogType.INFORMATION, "Press ok to finish the application.", "OK");

		// As soon as the modal dialog returns, the motion container will be cancelled. This finishes the position hold. 
		positionHoldContainer.cancel();
		
        
        getLogger().info("Close connection to client");
        friSession.close();

    }
    
}