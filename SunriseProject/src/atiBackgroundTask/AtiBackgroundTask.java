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
	
	private double raw_max = 2147483647.0;
	private double raw_min = -2147483648.0;
	private double Fx_max_N = 500.0;
	private double Fx_min_N = -Fx_max_N;
	private double Fy_max_N = Fx_max_N;
	private double Fy_min_N = Fx_min_N;
	private double Fz_max_N = 900.0;
	private double Fz_min_N = -Fz_max_N;
	   
	private double Tx_max_Nm = 20.0;
	private double Tx_min_Nm = -Tx_max_Nm;
	private double Ty_max_Nm = Tx_max_Nm;
	private double Ty_min_Nm = Tx_min_Nm;
	private double Tz_max_Nm = Tx_max_Nm;
	private double Tz_min_Nm = Tx_min_Nm;	
	
	private double Fx_scale = 0.0;
	private double Fy_scale = 0.0;
	private double Fz_scale = 0.0;
	
	private double Tx_scale = 0.0;
	private double Ty_scale = 0.0;
	private double Tz_scale = 0.0;
	

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
		
		// See page 49, section 8.3 Calibration Ranges, Axia80-M20
		if (Ati_Cal == 0){
			
			Fx_max_N = 500.0;
			Fx_min_N = -Fx_max_N;
			Fy_max_N = Fx_max_N;
			Fy_min_N = Fx_min_N;
			Fz_max_N = 900.0;
			Fz_min_N = -Fz_max_N;
			   
			Tx_max_Nm = 20.0;
			Tx_min_Nm = -Tx_max_Nm;
			Ty_max_Nm = Tx_max_Nm;
			Ty_min_Nm = Tx_min_Nm;
			Tz_max_Nm = Tx_max_Nm;
			Tz_min_Nm = Tx_min_Nm;		
		
		}else if (Ati_Cal == 1){
			
			Fx_max_N = 200.0;
			Fx_min_N = -Fx_max_N;
			Fy_max_N = Fx_max_N;
			Fy_min_N = Fx_min_N;
			Fz_max_N = 360.0;
			Fz_min_N = -Fz_max_N;
			   
			Tx_max_Nm = 8.0;
			Tx_min_Nm = -Tx_max_Nm;
			Ty_max_Nm = Tx_max_Nm;
			Ty_min_Nm = Tx_min_Nm;
			Tz_max_Nm = Tx_max_Nm;
			Tz_min_Nm = Tx_min_Nm;
		}else {
			Fx_max_N = raw_max;
			Fx_min_N = -Fx_max_N;
			Fy_max_N = Fx_max_N;
			Fy_min_N = Fx_min_N;
			Fz_max_N = raw_max;
			Fz_min_N = -Fz_max_N;
			   
			Tx_max_Nm = raw_max;
			Tx_min_Nm = -Tx_max_Nm;
			Ty_max_Nm = Tx_max_Nm;
			Ty_min_Nm = Tx_min_Nm;
			Tz_max_Nm = Tx_max_Nm;
			Tz_min_Nm = Tx_min_Nm;
		}
		
		Fx_scale = (Fx_max_N - Fx_min_N)/(raw_max - raw_min);
		Fy_scale = (Fy_max_N - Fy_min_N)/(raw_max - raw_min);
		Fz_scale = (Fz_max_N - Fz_min_N)/(raw_max - raw_min);

		Tx_scale = (Tx_max_Nm - Tx_min_Nm)/(raw_max - raw_min);
		Ty_scale = (Ty_max_Nm - Ty_min_Nm)/(raw_max - raw_min);
		Tz_scale = (Tz_max_Nm - Tz_min_Nm)/(raw_max - raw_min);

		
		getApplicationData().getProcessData("Fx_Ati").setValue(1.0*ati_axia_ft_sensor.getFx()*Fx_scale);
		getApplicationData().getProcessData("Fy_Ati").setValue(1.0*ati_axia_ft_sensor.getFy()*Fy_scale);
		getApplicationData().getProcessData("Fz_Ati").setValue(1.0*ati_axia_ft_sensor.getFz()*Fz_scale);
		
		getApplicationData().getProcessData("Taux_Ati").setValue(1.0*ati_axia_ft_sensor.getTx()*Tx_scale);
		getApplicationData().getProcessData("Tauy_Ati").setValue(1.0*ati_axia_ft_sensor.getTy()*Ty_scale);
		getApplicationData().getProcessData("Tauz_Ati").setValue(1.0*ati_axia_ft_sensor.getTz()*Tz_scale);

	}
}