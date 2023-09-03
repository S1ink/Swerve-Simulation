package frc.robot.swerve.simutil;

import edu.wpi.first.math.system.plant.DCMotor;


/** DCMotorModel provides extension functions to the {@link DCMotor} class (although these are currently only static) */
public class DCMotorModel {

	/** NOTE: The sum of getRawTorque(m, ...) and getEMFTorque(m, ...) should equal that of m.getTorque(m.getCurrent(...)) */

	/** Get the component of the total motor torque that is directly resultant of the voltage applied. */
	public static double getRawTorque(DCMotor m, double volts) {
		return m.KtNMPerAmp * (volts / m.rOhms);	// use the component of the total current from the source voltage, which is then proportional to the output torque
	}
	/** Get the component of the total motor torque that is directly resultant of the current rotational velocity. This component can be treated as frictional torque. */
	public static double getEMFTorque(DCMotor m, double r_vel) {
		return m.KtNMPerAmp * (-r_vel / (m.KvRadPerSecPerVolt * m.rOhms));
	}

	/** Get the sum frictional torque acting on the motor's rotation given a frictional model */
	public static double getFrictionalTorque(DCMotor m, FrictionModel f, double r_vel, double tq_app) {
		return f.calcNormalized(r_vel, tq_app) + getEMFTorque(m, r_vel);
	}


}