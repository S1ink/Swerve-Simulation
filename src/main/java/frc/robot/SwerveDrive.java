package frc.robot;

import frc.robot.team3407.drive.Types.*;
import edu.wpi.first.math.system.plant.DCMotor;


public class SwerveDrive {

	/** Base interface for all swerve modules.
	 * The interface allows for driving the module and optional simulation support. */
	public static interface SwerveModule {

		/** Set the desired state of the module. */
		public void setState(double angle_radians, double velocity);

		// default public double getWheelAngle() { return 0.0; }
		// default public double getWheelVelocity() { return 0.0; }

		/** Set the simulated steering angle in radians */
		default public void setSimulatedSteeringAngle(double angle) {}
		/** Set the simulated steering angular velocity in radians per second */
		default public void setSimulatedSteeringVelocity(double omega) {}
		/** Set the simulated wheel position in radians */
		default public void setSimulatedWheelPosition(double angle) {}
		/** Set the simulated wheel velocity in radians per second */
		default public void setSimulatedWheelVelocity(double velocity) {}

		/** Get the steering motors' torque given its voltage and angle. Units unspecified for now. */
		default public double getSteeringTorque(double a_volts, double b_volts, double a_omega, double b_omega) { return 0.0; }
		/** Get the driving motors' torque given its voltage and angle. Units unspecified for now. */
		default public double getDrivingTorque(double a_volts, double b_volts, double a_omega, double b_omega) { return 0.0; }
		/** Get the Rotational Inertia of the driving geartrain of the module */
		default public double getDrivingRI() { return 0.0; }
		/** Get the Rotational Inertia of the steering geartrain of the module */
		default public double getSteeringRI() { return 0.0; }

		/** Periodic code for updating the module's states. 'dt' is the time in seconds since the last call. */
		default public void periodic(double dt) {}

	}

	public static class SwerveConfig {



	}









}
