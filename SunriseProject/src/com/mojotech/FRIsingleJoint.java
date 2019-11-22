package com.mojotech;


import java.util.concurrent.TimeUnit;
import java.util.concurrent.TimeoutException;

import javax.inject.Inject;

import com.kuka.common.ThreadUtil;
import com.kuka.connectivity.fastRobotInterface.FRIConfiguration;
import com.kuka.connectivity.fastRobotInterface.FRISession;
import com.kuka.connectivity.fri.example.FRIIOGroup;
import com.kuka.generated.ioAccess.MediaFlangeIOGroup;
import com.kuka.roboticsAPI.applicationModel.RoboticsAPIApplication;
import static com.kuka.roboticsAPI.motionModel.BasicMotions.*;

import com.kuka.roboticsAPI.deviceModel.LBR;
import com.kuka.roboticsAPI.geometricModel.CartDOF;
import com.kuka.roboticsAPI.motionModel.controlModeModel.CartesianImpedanceControlMode;
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
public class FRIsingleJoint extends RoboticsAPIApplication
{
    private String _clientName;
    
    private static final int stiffnessZ = 2500;
	private static final int stiffnessY = 700;
	private static final int stiffnessX = 1500;
    
    final static double radiusOfCircMove=120;
	final static int nullSpaceAngle = 80;
	final static double offsetAxis2And4=Math.toRadians(20);
	final static double offsetAxis4And6=Math.toRadians(-40);
	double[] loopCenterPosition= new double[]{
			0, offsetAxis2And4, 0, offsetAxis2And4 +offsetAxis4And6 -Math.toRadians(90), 0, offsetAxis4And6,Math.toRadians(90)};
	private final static String informationText=
			"This application is intended for floor mounted robots!"+ "\n" +
			"\n" +
			"The robot moves to the start position and jogs joint 1 ";
    
    @Inject
    private LBR lbr;
    
    @Inject
    private FRIIOGroup friGroup;
    
    @Inject
    private MediaFlangeIOGroup mediaFlange;
    
    @Override
    public void initialize()
    {
        _clientName = "192.170.10.86";
        mediaFlange = new MediaFlangeIOGroup(lbr.getController());
    }

    @Override
    public void run()
    {
        FRIConfiguration friConfiguration = FRIConfiguration.createRemoteConfiguration(lbr, _clientName);
        friConfiguration.setSendPeriodMilliSec(1);

        friConfiguration.registerIO(friGroup.getInput("In_Bool_Clock_Enabled"));
        friConfiguration.registerIO(friGroup.getOutput("Out_Bool_Enable_Clock"));
        friConfiguration.registerIO(friGroup.getOutput("Out_Integer_Seconds"));
        friConfiguration.registerIO(friGroup.getOutput("Out_Analog_Deci_Seconds"));
        friConfiguration.registerIO(mediaFlange.getInput("InputX3Pin10"));
        
        getLogger().info("Creating FRI connection to " + friConfiguration.getHostName());
        getLogger().info("SendPeriod: " + friConfiguration.getSendPeriodMilliSec() + "ms |"
                + " ReceiveMultiplier: " + friConfiguration.getReceiveMultiplier());

        FRISession friSession = new FRISession(friConfiguration);

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
        
        getLogger().info("enable clock");
        ThreadUtil.milliSleep(5000);
        friGroup.setOut_Bool_Enable_Clock(true);
        
        getLogger().info("do something ...");
        motions();
        
        getLogger().info("disable clock");
        friGroup.setOut_Bool_Enable_Clock(false);
        
        getLogger().info("Close connection to client");
        friSession.close();

        
        // lBR.move(ptpHome().setJointVelocityRel(0.25));
    }
    
    
    public void motions() {		
		getLogger().info("Show modal dialog and wait for user to confirm");
        int isCancel = getApplicationUI().displayModalDialog(ApplicationDialogType.QUESTION, informationText, "OK", "Cancel");
        if (isCancel == 1)
        {
            return;
        }
        
        getLogger().info("Set impedance");
		CartesianImpedanceControlMode impedanceControlMode = 	new CartesianImpedanceControlMode();
		impedanceControlMode.parametrize(CartDOF.X).setStiffness(stiffnessX);
		impedanceControlMode.parametrize(CartDOF.Y).setStiffness(stiffnessY);
		impedanceControlMode.parametrize(CartDOF.Z).setStiffness(stiffnessZ);
        	
        for (int i=0; i<100; i++){
        	loopCenterPosition[0] = Math.toRadians(30);
			lbr.move(ptp(loopCenterPosition).setJointVelocityRel(0.25));
			loopCenterPosition[0] = Math.toRadians(-30);
			lbr.move(ptp(loopCenterPosition).setJointVelocityRel(0.25));
        }
	}
    
}