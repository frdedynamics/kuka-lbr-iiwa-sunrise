package com.kuka.generated.ioAccess;

import javax.inject.Inject;
import javax.inject.Singleton;

import com.kuka.roboticsAPI.controllerModel.Controller;
import com.kuka.roboticsAPI.ioModel.AbstractIOGroup;
import com.kuka.roboticsAPI.ioModel.IOTypes;

/**
 * Automatically generated class to abstract I/O access to I/O group <b>AtiAxiaFtSensor</b>.<br>
 * <i>Please, do not modify!</i>
 * <p>
 * <b>I/O group description:</b><br>
 * ATI AXIA F/T Sensor. Flange mounted with EtherCAT on X48.
 */
@Singleton
public class AtiAxiaFtSensorIOGroup extends AbstractIOGroup
{
	/**
	 * Constructor to create an instance of class 'AtiAxiaFtSensor'.<br>
	 * <i>This constructor is automatically generated. Please, do not modify!</i>
	 *
	 * @param controller
	 *            the controller, which has access to the I/O group 'AtiAxiaFtSensor'
	 */
	@Inject
	public AtiAxiaFtSensorIOGroup(Controller controller)
	{
		super(controller, "AtiAxiaFtSensor");

		addInput("Fx", IOTypes.INTEGER, 32);
		addInput("Fy", IOTypes.INTEGER, 32);
		addInput("Fz", IOTypes.INTEGER, 32);
		addInput("Tx", IOTypes.INTEGER, 32);
		addInput("Ty", IOTypes.INTEGER, 32);
		addInput("Tz", IOTypes.INTEGER, 32);
		addInput("SampleCounter", IOTypes.UNSIGNED_INTEGER, 32);
		addInput("StatusCode", IOTypes.UNSIGNED_INTEGER, 32);
		addDigitalOutput("Control1", IOTypes.UNSIGNED_INTEGER, 32);
		addDigitalOutput("Control2", IOTypes.UNSIGNED_INTEGER, 32);
	}

	/**
	 * Gets the value of the <b>digital input '<i>Fx</i>'</b>.<br>
	 * <i>This method is automatically generated. Please, do not modify!</i>
	 * <p>
	 * <b>I/O direction and type:</b><br>
	 * digital input
	 * <p>
	 * <b>User description of the I/O:</b><br>
	 * Force X-direction [N]
	 * <p>
	 * <b>Range of the I/O value:</b><br>
	 * [-2147483648; 2147483647]
	 *
	 * @return current value of the digital input 'Fx'
	 */
	public java.lang.Long getFx()
	{
		return getNumberIOValue("Fx", false).longValue();
	}

	/**
	 * Gets the value of the <b>digital input '<i>Fy</i>'</b>.<br>
	 * <i>This method is automatically generated. Please, do not modify!</i>
	 * <p>
	 * <b>I/O direction and type:</b><br>
	 * digital input
	 * <p>
	 * <b>User description of the I/O:</b><br>
	 * Force Y-direction [N]
	 * <p>
	 * <b>Range of the I/O value:</b><br>
	 * [-2147483648; 2147483647]
	 *
	 * @return current value of the digital input 'Fy'
	 */
	public java.lang.Long getFy()
	{
		return getNumberIOValue("Fy", false).longValue();
	}

	/**
	 * Gets the value of the <b>digital input '<i>Fz</i>'</b>.<br>
	 * <i>This method is automatically generated. Please, do not modify!</i>
	 * <p>
	 * <b>I/O direction and type:</b><br>
	 * digital input
	 * <p>
	 * <b>User description of the I/O:</b><br>
	 * Force Z-direction [N]
	 * <p>
	 * <b>Range of the I/O value:</b><br>
	 * [-2147483648; 2147483647]
	 *
	 * @return current value of the digital input 'Fz'
	 */
	public java.lang.Long getFz()
	{
		return getNumberIOValue("Fz", false).longValue();
	}

	/**
	 * Gets the value of the <b>digital input '<i>Tx</i>'</b>.<br>
	 * <i>This method is automatically generated. Please, do not modify!</i>
	 * <p>
	 * <b>I/O direction and type:</b><br>
	 * digital input
	 * <p>
	 * <b>User description of the I/O:</b><br>
	 * Torque around X-axis [Nm]
	 * <p>
	 * <b>Range of the I/O value:</b><br>
	 * [-2147483648; 2147483647]
	 *
	 * @return current value of the digital input 'Tx'
	 */
	public java.lang.Long getTx()
	{
		return getNumberIOValue("Tx", false).longValue();
	}

	/**
	 * Gets the value of the <b>digital input '<i>Ty</i>'</b>.<br>
	 * <i>This method is automatically generated. Please, do not modify!</i>
	 * <p>
	 * <b>I/O direction and type:</b><br>
	 * digital input
	 * <p>
	 * <b>User description of the I/O:</b><br>
	 * Torque around Y-axis [Nm]
	 * <p>
	 * <b>Range of the I/O value:</b><br>
	 * [-2147483648; 2147483647]
	 *
	 * @return current value of the digital input 'Ty'
	 */
	public java.lang.Long getTy()
	{
		return getNumberIOValue("Ty", false).longValue();
	}

	/**
	 * Gets the value of the <b>digital input '<i>Tz</i>'</b>.<br>
	 * <i>This method is automatically generated. Please, do not modify!</i>
	 * <p>
	 * <b>I/O direction and type:</b><br>
	 * digital input
	 * <p>
	 * <b>User description of the I/O:</b><br>
	 * Torque around Z-axis [Nm]
	 * <p>
	 * <b>Range of the I/O value:</b><br>
	 * [-2147483648; 2147483647]
	 *
	 * @return current value of the digital input 'Tz'
	 */
	public java.lang.Long getTz()
	{
		return getNumberIOValue("Tz", false).longValue();
	}

	/**
	 * Gets the value of the <b>digital input '<i>SampleCounter</i>'</b>.<br>
	 * <i>This method is automatically generated. Please, do not modify!</i>
	 * <p>
	 * <b>I/O direction and type:</b><br>
	 * digital input
	 * <p>
	 * <b>User description of the I/O:</b><br>
	 * ./.
	 * <p>
	 * <b>Range of the I/O value:</b><br>
	 * [0; 4294967295]
	 *
	 * @return current value of the digital input 'SampleCounter'
	 */
	public java.lang.Long getSampleCounter()
	{
		return getNumberIOValue("SampleCounter", false).longValue();
	}

	/**
	 * Gets the value of the <b>digital input '<i>StatusCode</i>'</b>.<br>
	 * <i>This method is automatically generated. Please, do not modify!</i>
	 * <p>
	 * <b>I/O direction and type:</b><br>
	 * digital input
	 * <p>
	 * <b>User description of the I/O:</b><br>
	 * ./.
	 * <p>
	 * <b>Range of the I/O value:</b><br>
	 * [0; 4294967295]
	 *
	 * @return current value of the digital input 'StatusCode'
	 */
	public java.lang.Long getStatusCode()
	{
		return getNumberIOValue("StatusCode", false).longValue();
	}

	/**
	 * Gets the value of the <b>digital output '<i>Control1</i>'</b>.<br>
	 * <i>This method is automatically generated. Please, do not modify!</i>
	 * <p>
	 * <b>I/O direction and type:</b><br>
	 * digital output
	 * <p>
	 * <b>User description of the I/O:</b><br>
	 * ./.
	 * <p>
	 * <b>Range of the I/O value:</b><br>
	 * [0; 4294967295]
	 *
	 * @return current value of the digital output 'Control1'
	 */
	public java.lang.Long getControl1()
	{
		return getNumberIOValue("Control1", true).longValue();
	}

	/**
	 * Sets the value of the <b>digital output '<i>Control1</i>'</b>.<br>
	 * <i>This method is automatically generated. Please, do not modify!</i>
	 * <p>
	 * <b>I/O direction and type:</b><br>
	 * digital output
	 * <p>
	 * <b>User description of the I/O:</b><br>
	 * ./.
	 * <p>
	 * <b>Range of the I/O value:</b><br>
	 * [0; 4294967295]
	 *
	 * @param value
	 *            the value, which has to be written to the digital output 'Control1'
	 */
	public void setControl1(java.lang.Long value)
	{
		setDigitalOutput("Control1", value);
	}

	/**
	 * Gets the value of the <b>digital output '<i>Control2</i>'</b>.<br>
	 * <i>This method is automatically generated. Please, do not modify!</i>
	 * <p>
	 * <b>I/O direction and type:</b><br>
	 * digital output
	 * <p>
	 * <b>User description of the I/O:</b><br>
	 * ./.
	 * <p>
	 * <b>Range of the I/O value:</b><br>
	 * [0; 4294967295]
	 *
	 * @return current value of the digital output 'Control2'
	 */
	public java.lang.Long getControl2()
	{
		return getNumberIOValue("Control2", true).longValue();
	}

	/**
	 * Sets the value of the <b>digital output '<i>Control2</i>'</b>.<br>
	 * <i>This method is automatically generated. Please, do not modify!</i>
	 * <p>
	 * <b>I/O direction and type:</b><br>
	 * digital output
	 * <p>
	 * <b>User description of the I/O:</b><br>
	 * ./.
	 * <p>
	 * <b>Range of the I/O value:</b><br>
	 * [0; 4294967295]
	 *
	 * @param value
	 *            the value, which has to be written to the digital output 'Control2'
	 */
	public void setControl2(java.lang.Long value)
	{
		setDigitalOutput("Control2", value);
	}

}
