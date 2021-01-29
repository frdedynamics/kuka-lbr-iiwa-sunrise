package atiBackgroundTask;


import javax.inject.Inject;
import java.util.concurrent.TimeUnit;

import com.kuka.generated.ioAccess.AtiAxiaFtSensorIOGroup;
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
public class AtiBackgroundTask extends RoboticsAPICyclicBackgroundTask {
	
	@Inject
    private AtiAxiaFtSensorIOGroup ati_axia_ft_sensor;
	
	private int Ctrl1 = 0;
	private int Ati_Lp_Filter = 0;
	private int Ati_Cal = 0;
	private int Ati_Rate = 0;
	private double CountsPerForce = 1000000.0;
	private double CountsPerTorque = CountsPerForce;
	

	@Override
	public void initialize() {
		// initialize your task here
		initializeCyclic(0, 100, TimeUnit.MILLISECONDS,
				CycleBehavior.BestEffort);
		
	}

	@Override
	public void runCyclic() {
		// your task execution starts here
		
		// See document #9610-05-EtherCAT Axia-09
		//  Manual, F/T Sensor, EtherCAT Axia
		//  page 41, section 5.2.1.11 Object 0x7010: Control Codes
		
		Ctrl1 = 0;
		
		if (getApplicationData().getProcessData("Ati_Bias").getValue()){
			Ctrl1 |= (1<<0);
		}
		
		if (getApplicationData().getProcessData("Ati_Clear_Bias").getValue()){
			Ctrl1 |= (1<<2);
		}
		
		Ati_Lp_Filter = getApplicationData().getProcessData("Ati_Lp_Filter").getValue();
		if (Ati_Lp_Filter == 1){
			Ctrl1 |= (1<<4);
		}else if (Ati_Lp_Filter == 2){
			Ctrl1 |= (1<<5);
		}else if (Ati_Lp_Filter == 3){
			Ctrl1 |= (1<<4) | (1<<5);
		}else if (Ati_Lp_Filter == 4){
			Ctrl1 |= (1<<6);
		}else if (Ati_Lp_Filter == 5){
			Ctrl1 |= (1<<4) | (1<<6);
		}else if (Ati_Lp_Filter == 6){
			Ctrl1 |= (1<<5) | (1<<6);
		}else if (Ati_Lp_Filter == 7){
			Ctrl1 |= (1<<4) | (1<<5) | (1<<6);
		}else if (Ati_Lp_Filter == 8){
			Ctrl1 |= (1<<7);
		}
		
		Ati_Cal = getApplicationData().getProcessData("Ati_Cal").getValue();
		if(Ati_Cal == 1){
			Ctrl1 |= (1<<8);
		}
		
		Ati_Rate = getApplicationData().getProcessData("Ati_Rate").getValue();
		if(Ati_Rate == 1){
			Ctrl1 |= (1<<12);
		}else if(Ati_Rate == 2){
			Ctrl1 |= (1<<13);
		}else if(Ati_Rate == 3){
			Ctrl1 |= (1<<12) | (1<<13);
		}
		
		getApplicationData().getProcessData("Ati_Control1").setValue(Ctrl1);
		
		getApplicationData().getProcessData("Fx_Ati").setValue(1.0*ati_axia_ft_sensor.getFx()/CountsPerForce);
		getApplicationData().getProcessData("Fy_Ati").setValue(1.0*ati_axia_ft_sensor.getFy()/CountsPerForce);
		getApplicationData().getProcessData("Fz_Ati").setValue(1.0*ati_axia_ft_sensor.getFz()/CountsPerForce);
		
		getApplicationData().getProcessData("Taux_Ati").setValue(1.0*ati_axia_ft_sensor.getTx()/CountsPerTorque);
		getApplicationData().getProcessData("Tauy_Ati").setValue(1.0*ati_axia_ft_sensor.getTy()/CountsPerTorque);
		getApplicationData().getProcessData("Tauz_Ati").setValue(1.0*ati_axia_ft_sensor.getTz()/CountsPerTorque);

	}
}