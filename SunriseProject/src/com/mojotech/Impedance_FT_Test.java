package com.mojotech;

import com.kuka.generated.ioAccess.AtiAxiaFtSensorIOGroup;
import com.kuka.generated.ioAccess.MediaFlangeIOGroup;
import com.kuka.roboticsAPI.applicationModel.RoboticsAPIApplication;
import static com.kuka.roboticsAPI.motionModel.BasicMotions.*;

import com.kuka.roboticsAPI.deviceModel.LBR;
import com.kuka.roboticsAPI.deviceModel.LBRE1Redundancy;
import com.kuka.roboticsAPI.geometricModel.CartDOF;
import com.kuka.roboticsAPI.geometricModel.Frame;
import com.kuka.roboticsAPI.geometricModel.World;
import com.kuka.roboticsAPI.motionModel.LIN;
import com.kuka.roboticsAPI.motionModel.PTP;
import com.kuka.roboticsAPI.motionModel.controlModeModel.CartesianImpedanceControlMode;
import com.kuka.roboticsAPI.sensorModel.DataRecorder;
import com.kuka.roboticsAPI.uiModel.ApplicationDialogType;

public class Impedance_FT_Test extends RoboticsAPIApplication {
	
	private MediaFlangeIOGroup media_flange;
	private AtiAxiaFtSensorIOGroup ati_axia_ft_sensor;

	final static double offsetAxis2And4=Math.toRadians(20);
	final static double offsetAxis4And6=Math.toRadians(-40);
	final static int nullSpaceAngle = 80;
	final static int numLoops = 3;

	private static final int stiffnessZ = 2500;
	private static final int stiffnessY = 700;
	private static final int stiffnessX = 1500;
	private LBR lbr;
	private final static String informationText=
			"This application is intended for floor mounted robots!"+ "\n" +
			"\n" +
			"The robot moves to the start position and moves in null space." +
			"\n" +
			"The stiffness is set to " +
			"X="+stiffnessX+" Y="+stiffnessY +" Z="+stiffnessZ+" in N/m." +
			"\n" +
			"Null space angle is set to "+nullSpaceAngle;
	
	double[] loopCenterPosition= new double[]{
			0, offsetAxis2And4, 0, offsetAxis2And4 +offsetAxis4And6 -Math.toRadians(90), 0, offsetAxis4And6,Math.toRadians(90)};

	
	public void initialize() {
		lbr = getContext().getDeviceFromType(LBR.class);
		media_flange = new MediaFlangeIOGroup(lbr.getController());
		ati_axia_ft_sensor = new AtiAxiaFtSensorIOGroup(lbr.getController());
		
	}

	public void run() {
		getLogger().info("Show modal dialog and wait for user to confirm");
        int isCancel = getApplicationUI().displayModalDialog(ApplicationDialogType.QUESTION, informationText, "OK", "Cancel");
        if (isCancel == 1)
        {
            return;
        }
        
        media_flange.setLEDBlue(true);
        
        getLogger().info("Move to start position");	
		PTP ptpToLoopCenter = ptp(loopCenterPosition);
		ptpToLoopCenter.setJointVelocityRel(0.25);
		lbr.move(ptpToLoopCenter);

		getLogger().info("Look at this ATI measurement");
		getLogger().info(ati_axia_ft_sensor.getFx().toString());
		getLogger().info(ati_axia_ft_sensor.getFy().toString());
		getLogger().info(ati_axia_ft_sensor.getFz().toString());
		
		getLogger().info(ati_axia_ft_sensor.getTx().toString());
		getLogger().info(ati_axia_ft_sensor.getTy().toString());
		getLogger().info(ati_axia_ft_sensor.getTz().toString());
		
		getLogger().info(ati_axia_ft_sensor.getSampleCounter().toString());
		getLogger().info(ati_axia_ft_sensor.getStatusCode().toString());
		
		//getLogger().info(ati_axia_ft_sensor.getControl1().toString());
		//getLogger().info(ati_axia_ft_sensor.getControl2().toString());
		
		getLogger().info("Set start frame");
		Frame startFrame = lbr.getCurrentCartesianPosition(lbr.getFlange());
		
		getLogger().info("Set impedance");
		CartesianImpedanceControlMode impedanceControlMode = new CartesianImpedanceControlMode();
		impedanceControlMode.parametrize(CartDOF.X).setStiffness(stiffnessX);
		impedanceControlMode.parametrize(CartDOF.Y).setStiffness(stiffnessY);
		impedanceControlMode.parametrize(CartDOF.Z).setStiffness(stiffnessZ);
		
		getLogger().info(lbr.getFlange().toString());
		getLogger().info(World.Current.getRootFrame().toString());
		getLogger().info("Setup data recorder");
		DataRecorder rec = new DataRecorder();
		
		rec.setSampleInterval(1);
		
		rec.addInternalJointTorque(lbr);
		rec.addCurrentJointPosition(lbr, DataRecorder.AngleUnit.Radian);
		rec.addCommandedJointPosition(lbr, DataRecorder.AngleUnit.Radian);
		rec.addExternalJointTorque(lbr);
		rec.addCartesianForce(lbr.getFlange(), null);
		rec.addCartesianTorque(lbr.getFlange(), null);
		rec.addCurrentCartesianPositionXYZ(lbr.getFlange(), null);
		rec.addCommandedCartesianPositionXYZ(lbr.getFlange(), null);
		
		
		for (int i=1; i<=numLoops; i++){
			
			rec.enable();
			getLogger().info("Recording to file name "+rec.getFileName());
			rec.startRecording();
			
			getLogger().info("Loop "+i+" of "+numLoops);
			
			
			getLogger().info("Move in nullspace -"+nullSpaceAngle+"?");		
			Frame centerFrameWithChangedE1_1 = createChildFrameAndSetE1Offset(startFrame,Math.toRadians(-nullSpaceAngle));
			LIN linToCenterFrameWithE1_1 = lin(centerFrameWithChangedE1_1);
			linToCenterFrameWithE1_1.setJointVelocityRel(0.25);
			linToCenterFrameWithE1_1.setMode(impedanceControlMode);
			lbr.move(linToCenterFrameWithE1_1);
			
			/*
			getLogger().info("Move in nullspace "+nullSpaceAngle+"?");
			Frame centerFrameWithChangedE1_2 = createChildFrameAndSetE1Offset(startFrame,Math.toRadians(nullSpaceAngle));
			LIN linToCenterFrameWithE1_2 = lin(centerFrameWithChangedE1_2);
			linToCenterFrameWithE1_2.setJointVelocityRel(0.25);
			linToCenterFrameWithE1_2.setMode(impedanceControlMode);
			lbr.move(linToCenterFrameWithE1_2);
			*/
			
			getLogger().info("Move to start position");
			LIN linToStartFrame = lin(startFrame);
			linToStartFrame.setJointVelocityRel(0.25);
			linToStartFrame.setMode(impedanceControlMode);
			lbr.move(linToStartFrame);
			
			rec.stopRecording();
			
		}
		
		media_flange.setLEDBlue(false);
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
