package com.mojotech;


import java.util.concurrent.TimeUnit;
import java.util.concurrent.TimeoutException;

import javax.inject.Inject;

import com.kuka.common.ThreadUtil;
import com.kuka.connectivity.fastRobotInterface.FRIConfiguration;
import com.kuka.connectivity.fastRobotInterface.FRISession;
import com.kuka.generated.ioAccess.MediaFlangeIOGroup;
import com.kuka.roboticsAPI.applicationModel.RoboticsAPIApplication;
import static com.kuka.roboticsAPI.motionModel.BasicMotions.*;

import com.kuka.roboticsAPI.deviceModel.LBR;
import com.kuka.roboticsAPI.deviceModel.LBRE1Redundancy;
import com.kuka.roboticsAPI.geometricModel.CartDOF;
import com.kuka.roboticsAPI.geometricModel.Frame;
import com.kuka.roboticsAPI.motionModel.LIN;
import com.kuka.roboticsAPI.motionModel.PTP;
import com.kuka.roboticsAPI.motionModel.Spline;
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
public class FRIRvizMotions extends RoboticsAPIApplication
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
			"The robot moves to the start position and based on this position, a motion that " +
			"describes the symbol of lemniscate (a 'horizontal eight') will be executed." + "\n" +
			"In a next step the robot will move in nullspace by "+nullSpaceAngle+"? in both directions.";
    
    @Inject
    private LBR lbr;
    
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
        
        ThreadUtil.milliSleep(5000);
        
        getLogger().info("do something ...");
        motions();
        
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
			getLogger().info("Move to start position of the lemniscate motion");	
			PTP ptpToLoopCenter = ptp(loopCenterPosition);
			ptpToLoopCenter.setJointVelocityRel(0.25);
			lbr.move(ptpToLoopCenter);
	
			getLogger().info("Compute spline for lemniscate motion");	
			Frame startFrame = lbr.getCurrentCartesianPosition(lbr.getFlange());
			Spline lemniscateSpline = createLemniscateSpline(startFrame).setJointJerkRel(0.5).setCartVelocity(250);
	
			getLogger().info("Execute lemniscate motion");
			lemniscateSpline.setJointVelocityRel(0.25);
			lemniscateSpline.setMode(impedanceControlMode);
			lbr.move(lemniscateSpline);
	
			getLogger().info("Move in nullspace -"+nullSpaceAngle+"?");		
			Frame centerFrameWithChangedE1_1 = createChildFrameAndSetE1Offset(startFrame,Math.toRadians(-nullSpaceAngle));
			LIN linToCenterFrameWithE1_1 = lin(centerFrameWithChangedE1_1);
			linToCenterFrameWithE1_1.setJointVelocityRel(0.25);
			linToCenterFrameWithE1_1.setMode(impedanceControlMode);
			lbr.move(linToCenterFrameWithE1_1);
	
			getLogger().info("Move in nullspace "+nullSpaceAngle+"?");
			Frame centerFrameWithChangedE1_2 = createChildFrameAndSetE1Offset(startFrame,Math.toRadians(nullSpaceAngle));
			LIN linToCenterFrameWithE1_2 = lin(centerFrameWithChangedE1_2);
			linToCenterFrameWithE1_2.setJointVelocityRel(0.25);
			linToCenterFrameWithE1_2.setMode(impedanceControlMode);
			lbr.move(linToCenterFrameWithE1_2);
			
			getLogger().info("Move to start position");
			LIN linToStartFrame = lin(startFrame);
			linToStartFrame.setJointVelocityRel(0.25);
			linToStartFrame.setMode(impedanceControlMode);
			lbr.move(linToStartFrame);
        }
	}
    
    
    private Spline createLemniscateSpline(Frame centerFrame) {

		// Create a new frame with the center frame as parent. Set an offset for the x axis to this parent.
		Frame rightFrame=(new Frame(centerFrame)).setX(2*radiusOfCircMove);

		// Create a new frame with the center frame as parent. Set an offset for the x axis to this parent.
		Frame leftFrame= (new Frame(centerFrame)).setX(-2*radiusOfCircMove);	

		// Create a new frame with the center frame as parent. Set an offset for the x and y axes to this parent.
		Frame topLeftFrame= (new Frame(centerFrame)).setX(-radiusOfCircMove).setY(radiusOfCircMove);		

		// Create a new frame with the center frame as parent. Set an offset for the x and y axes to this parent.
		Frame topRightFrame= (new Frame(centerFrame)).setX(+radiusOfCircMove).setY(radiusOfCircMove);		

		// Create a new frame with the center frame as parent. Set an offset for the x and y axes to this parent.
		Frame bottomRightFrame= (new Frame(centerFrame)).setX(+radiusOfCircMove).setY(-radiusOfCircMove);

		// Create a new frame with the center frame as parent. Set an offset for the x and y axes to this parent.
		Frame bottomLeftFrame= (new Frame(centerFrame)).setX(-radiusOfCircMove).setY(-radiusOfCircMove);

		// Create a spline that describes a lemniscate
		Spline spline = new Spline(
				spl(bottomLeftFrame),
				spl(leftFrame),
				spl(topLeftFrame),
				spl(centerFrame),
				spl(bottomRightFrame),
				spl(rightFrame),
				spl(topRightFrame),
				spl(centerFrame));
		return spline;
	}

	
	private Frame createChildFrameAndSetE1Offset( Frame parent, double offset) {

		// Create a new frame
		Frame childFrame = new Frame(parent);

		// Create new redundancy information
		LBRE1Redundancy newRedundancyInformation = new LBRE1Redundancy().setE1(offset);

		// Add new redundancy information to new frame
		childFrame.setRedundancyInformation(lbr, newRedundancyInformation);
		return childFrame;
	}
}