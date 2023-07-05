package frc.robot;

import edu.wpi.first.util.sendable.*;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.*;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.constraint.SwerveDriveKinematicsConstraint;

import frc.robot.SwerveKinematics.SwerveVisualization;
import frc.robot.team3407.Util;
import frc.robot.team3407.drive.Types.*;


public class SwerveDrive implements Subsystem, Sendable {

	/** Base interface for all swerve modules such that they can be 
	 * driven, tracked using odometry, and optionally used in simulation. */
	public static abstract class SwerveModule implements Sendable {

		public final Translation2d module_location;
		protected SwerveModule(Translation2d loc) { this.module_location = loc; }

		/** Set the desired state of the module. */
		public void setState(SwerveModuleState state) { this.setState(state.speedMetersPerSecond, state.angle.getRadians()); }
		/** Set the desired state of the module. Velocity is linear. */
		abstract public void setState(double linear_vel, double steer_angle_rad);

		/** Get the steering angle in radians. The coord system begins pointing forward (robot front) and is CCW+ (unit circle) */
		abstract public double getSteeringAngle();
		/** Get the wheel's linear displacement in meters (takes into account the radius of the wheel). Positive values represent forward displacement. */
		abstract public double getWheelDisplacement();	// << LINEAR NOT ANGULAR
		/** Get the module's position as a SwerveModulePosition object. */
		public SwerveModulePosition getPosition() {
			return new SwerveModulePosition(this.getWheelDisplacement(), Rotation2d.fromRadians(this.getSteeringAngle()));
		}

		/** Get the steering angular velocity in radians per second */
		public double getSteeringRate() { return 0.0; }
		/** Get the wheel's linear velocity in meters per second */
		public double getWheelVelocity() { return 0.0; }	// << LINEAR NOT ANGULAR
		/** Get the module's state as a SwerveModuleState object. */
		public SwerveModuleState getState() {
			return new SwerveModuleState(this.getWheelVelocity(), Rotation2d.fromRadians(this.getSteeringAngle()));
		}

		/** Periodic code for updating the module's internal states. 'dt' is the time in seconds since the last call. */
		public void periodic(double dt) {}


		/* SIMULATION */
		/* NOTE: Above, the wheel's forward/backward movement is tracked linearly - meaning the
		 * distance/speed along the floor (meters/meters per second). Here, the movement is
		 * tracked angularly (radians, radians per second) because that is more convenient for the
		 * simulator. The conversion between linear and angular translations should be done internally
		 * using the wheel radius - this is done so that the wheel radius does not ever need to be known
		 * by the interface or larger swerve system. */

		/** Get the voltage applied to motor A */
		public double getMotorAVolts() { return 0.0; }
		/** Get the voltage applied to motor B */
		public double getMotorBVolts() { return 0.0; }
		/** Get the torque applied along the steering axis given both motors' input voltages and the steering/driving angular velocities. Units are in volts and radians per second. */
		public double getSteeringTorque(double a_volts, double b_volts, double s_omega, double d_omega) { return 0.0; }
		/** Get the torque applied along the driving axis given both motors' input voltages and the steering/driving angular velocities. Units are in volts and radians per second. */
		public double getDrivingTorque(double a_volts, double b_volts, double s_omega, double d_omega) { return 0.0; }
		/** Get the sum rotational inertia along the steering geartrain of the module */
		public double getSteeringRI() { return 0.0; }
		/** Get the sum rotational inertia along the driving geartrain of the module */
		public double getDrivingRI() { return 0.0; }

		/** Set the simulated steering angle in radians */
		public void setSimulatedSteeringAngle(double angle) {}
		/** Set the simulated steering angular velocity in radians per second */
		public void setSimulatedSteeringRate(double omega) {}
		/** Set the simulated wheel displacement in radians */
		public void setSimulatedWheelPosition(double angle) {}
		/** Set the simulated wheel velocity in radians per second */
		public void setSimulatedWheelVelocity(double omega) {}

		/** Whether simulation is supported or not based on if valid rotational inertias have been implemented. */
		public boolean isSimSupported() {
			return !(this.getSteeringRI() == 0.0 && this.getDrivingRI() == 0.0);
		}


		@Override
		public void initSendable(SendableBuilder b) {
			b.addDoubleProperty("Module Angle (Rad)", this::getSteeringAngle, null);
			b.addDoubleProperty("Module Displacement (M)", this::getWheelDisplacement, null);
		}

	}

	public static class SwerveConfig {	// put all base config options here, extend externally with additional implementation-specific options



	}










	private final SwerveModule[] modules;
	private final Gyro gyro;

	private final SwerveDriveKinematics kinematics;
	private final SwerveDriveOdometry odometry;
	private final SwerveVisualization visualization;

	public final int SIZE;

	private SwerveModulePosition[] positions;
	private SwerveModuleState[] states;

	public SwerveDrive(Gyro gyro, SwerveModule... modules) {
		this.modules = modules;
		this.gyro = gyro;
		this.SIZE = modules.length;
		this.positions = new SwerveModulePosition[this.SIZE];
		this.states = new SwerveModuleState[this.SIZE];

		final Translation2d[] locations = new Translation2d[this.SIZE];
		for(int i = 0; i < this.SIZE; i++) {
			locations[i] = this.modules[i].module_location;
			this.positions[i] = this.modules[i].getPosition();
			this.states[i] = this.modules[i].getState();
		}
		this.kinematics = new SwerveDriveKinematics(locations);
		this.odometry = new SwerveDriveOdometry(this.kinematics, this.gyro.getRotation2d(), this.positions);
		this.visualization = new SwerveVisualization(locations);
	}


	protected void updateCachedStates() {
		for(int i = 0; i < this.SIZE; i++) {
			this.positions[i] = this.modules[i].getPosition();
			this.states[i] = this.modules[i].getState();
		}
	}

	@Override
	public void periodic() {
		this.updateCachedStates();
		this.odometry.update(
			this.gyro.getRotation2d(),
			this.positions
		);
	}

	@Override
	public void initSendable(SendableBuilder b) {
		b.addDoubleArrayProperty("Odometry", ()->Util.toComponents2d(this.odometry.getPoseMeters()), null);
		b.addDoubleArrayProperty("Module Poses 3d", ()->Util.toComponents3d(this.visualization.getWheelPoses3d(this.positions)), null);
		b.addDoubleArrayProperty("Wheel Vectors 2d", ()->SwerveVisualization.getComponents2d(this.states), null);
	}

}
