package frc.robot.swerve;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.util.sendable.*;
import edu.wpi.first.math.kinematics.SwerveModuleState;

import frc.robot.swerve.SwerveUtils.*;
import frc.robot.swerve.simutil.*;


/** Base interface for all swerve modules such that they can be 
 * driven, tracked using odometry, and optionally used in simulation. */
public abstract class SwerveModule implements Sendable {

	public final Translation2d module_location;
	protected SwerveModule(Translation2d loc) { this.module_location = loc; }

	/** Set the desired state of the module. */
	public void setState(SwerveModuleState state)
		{ this.setState(state.speedMetersPerSecond, state.angle.getRadians()); }
	/** Set the desired second order state of the module. */
	public void setState(SwerveModuleStates state)
		{ this.setState(state.linear_velocity, state.angle.getRadians(), state.linear_acceleration, state.angular_velocity); }
	/** Set the desired state of the module. Wheel velocity is linear. */
	public void setState(double linear_vel, double steer_angle_rad)
		{ this.setState(linear_vel, steer_angle_rad, 0.0, 0.0); }
	/** Set the desired second order state of the module. Wheel velocity and accelerations are linear. */
	abstract public void setState(
		double linear_vel,
		double steer_angle_rad,
		double linear_acc,
		double steer_angular_vel
	);

	/** Get the steering angle in radians. The coord system begins pointing forward (robot front) and is CCW+ (unit circle) */
	abstract public double getSteeringAngle();
	/** Get the wheel's linear displacement in meters (takes into account the radius of the wheel). Positive values represent forward displacement. */
	abstract public double getWheelDisplacement();	// << LINEAR NOT ANGULAR
	/** Get the steering angular velocity in radians per second */
	public double getSteeringRate() { return 0.0; }
	/** Get the wheel's linear velocity in meters per second */
	public double getWheelVelocity() { return 0.0; }	// << LINEAR NOT ANGULAR
	/** Get all combined states. */
	public SwerveModuleStates getStates() {
		return new SwerveModuleStates(
			this.getSteeringAngle(),
			this.getWheelDisplacement(),
			this.getWheelVelocity(),
			this.getSteeringRate(),
			0.0
		);
	}

	/** Periodic code for updating the module's internal states. 'dt' is the time in seconds since the last call. */
	public void periodic(double dt) {}


	/* SIMULATION */

	/** Get the voltage applied to motor A */
	public double getMotorAVolts() { return 0.0; }
	/** Get the voltage applied to motor B */
	public double getMotorBVolts() { return 0.0; }
	/** Get all simulation required functionality */
	public SwerveModuleModel getSimProperties() { return null; }

	/** Set the simulated steering angle in radians */
	public void setSimulatedSteeringAngle(double angle) {}
	/** Set the simulated steering angular velocity in radians per second */
	public void setSimulatedSteeringRate(double omega) {}
	/** Set the simulated wheel displacement in meters */
	public void setSimulatedWheelPosition(double angle) {}		// LINEAR NOT ANGULAR
	/** Set the simulated wheel velocity in meters per second */
	public void setSimulatedWheelVelocity(double omega) {}		// LINEAR NOT ANGULAR


	@Override
	public void initSendable(SendableBuilder b) {
		b.addDoubleProperty("Module Angle (Rad)", this::getSteeringAngle, null);
		b.addDoubleProperty("Module Displacement (M)", this::getWheelDisplacement, null);
	}


}
