package frc.robot.swerve;

import edu.wpi.first.util.sendable.*;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Subsystem;

import frc.robot.swerve.SwerveUtils.*;


/** A high-level swerve drive abstraction that allows driving and simulation of any possible {@link SwerveModule} implementation,
 * and allows for any number of modules and physical layouts. */
public class SwerveDrive<Module_T extends SwerveModule> implements Subsystem, Sendable {

	protected final Module_T[] modules;
	protected final Gyro gyro;

	protected final SwerveKinematics kinematics;
	protected final SwerveOdometry odometry;
	protected final SwerveVisualization visualization;

	protected final SwerveModuleStates[] lock_states;
	protected SwerveModuleStates[] states, targets;
	protected AugmentationParams aug_params = new AugmentationParams(true);
	private double prev_time, dt;

	public final int SIZE;


	/** Create a new SwerveDrive with the provided gyro interface and modules.
	 * MAKE SURE TO CALL register() ON THIS INSTANCE OR MANUALLY CALL periodic() SO THAT STATES GET UPDATED! */
	public SwerveDrive(Gyro gyro, Module_T... modules) {
		this.modules = modules;
		this.gyro = gyro;
		this.SIZE = modules.length;
		this.states = new SwerveModuleStates[this.SIZE];
		this.targets = new SwerveModuleStates[this.SIZE];

		final Translation2d[] locations = new Translation2d[this.SIZE];
		for(int i = 0; i < this.SIZE; i++) {
			locations[i] = this.modules[i].module_location;
			this.states[i] = this.modules[i].getStates();
			this.targets[i] = new SwerveModuleStates();
		}
		this.kinematics = new SwerveKinematics(locations);
		this.odometry = new SwerveOdometry(this);
		this.visualization = new SwerveVisualization(locations);
		this.lock_states = this.kinematics.lockFormation(null);
		this.prev_time = Timer.getFPGATimestamp();
	}


	protected void updateCachedStates() {
		for(int i = 0; i < this.SIZE; i++) {
			this.states[i] = this.modules[i].getStates();
		}
	}

	@Override
	public void periodic() {
		final double t = Timer.getFPGATimestamp();
		this.dt = t - this.prev_time;
		this.prev_time = t;
		this.updateCachedStates();
		this.odometry.update();
		for(final Module_T m : this.modules) {
			m.periodic(this.dt);
		}
	}

	@Override
	public void initSendable(SendableBuilder b) {
		b.addDoubleArrayProperty("Odometry Tracked Pose", ()->SwerveUtils.toComponents2d(this.odometry.getPose()), null);
		b.addDoubleArrayProperty("Odometry Pure Displacement", ()->SwerveUtils.toComponents2d(this.odometry.getDisplacement()), null);
		b.addDoubleArrayProperty("[S] Module Poses 3d", ()->SwerveUtils.toComponents3d(this.visualization.getWheelPoses3d(this.states)), null);
		b.addDoubleArrayProperty("[S] Wheel Vectors 2d", ()->SwerveVisualization.getVecComponents2d(this.states), null);
		b.addDoubleArrayProperty("[T] Module Poses 3d", ()->SwerveUtils.toComponents3d(this.visualization.getWheelPoses3d(this.targets)), null);
		b.addDoubleArrayProperty("[T] Wheel Vectors 2d", ()->SwerveVisualization.getVecComponents2d(this.targets), null);
		b.addDoubleProperty("Update Period", ()->this.dt, null);
	}


	/** Initialize this instance and each module under the given table in smart dashboard */
	public void smartDashboardInit(String table) {
		SmartDashboard.putData(table, this);
		for(int i = 0; i < this.SIZE; i++) {
			SmartDashboard.putData(
				String.format("%s/Module %d", table, i),
				this.modules[i]
			);
		}
	}


	/** Set the default parameters that specify which post-processing steps should be taken when applying new target states to the modules. */
	public void applyParams(AugmentationParams params) {
		this.aug_params = params;
	}


	/** Update the swerve's target state. Should be relative to the robot's coordinate frame. */
	public void applyTarget(ChassisSpeeds target_state) {
		this.applyTarget(new ChassisStates(target_state));
	}
	/** Update the swerve's target state. Should be relative to the robot's coordinate frame (field relative transform should be applied beforehand). */
	public void applyTarget(ChassisStates target_state) {
		this.targets = this.kinematics.toModuleStates(target_state, this.targets);	// targets is updated inline, although a new buffer is generated on size mismatches so reset anyway
		this.applyStates(this.targets, this.aug_params);
	}
	/** Set the target state for a locking formation. */
	public void applyLocked() {
		for(int i = 0; i < this.SIZE; i++) {
			this.targets[i].copy( this.lock_states[i] );	// post processing may augment the targets so copy to ensure that lock states do not change
		}
		this.applyStates(this.targets, this.aug_params);
	}
	/** Post process the provides states according to the parameters and apply to the modules. For internal use only. */
	protected void applyStates(SwerveModuleStates[] states, AugmentationParams params) {
		if(params.should_normalize) {
			SwerveKinematics.normalizeModuleVelocities(params.max_wheel_velocity, states);
		}
		for(int i = 0; i < this.SIZE; i++) {
			if(params.should_optimize) {
				SwerveModuleStates.optimize(
					states[i],
					this.modules[i].getSteeringAngle(),
					states[i]
				);
			}
			this.modules[i].setState(states[i]);
		}
	}
	/** Call each module's stop function to disable all operation. */
	public void stop() {
		for(int i = 0; i < this.SIZE; i++) {
			this.targets[i].zero();
			this.modules[i].stop();
		}
	}


	/** Access a module. Index must be in [0, SIZE) */
	public Module_T module(int i) {
		return (i >= 0 && i < this.SIZE) ? this.modules[i] : null;
	}

	/** Get the currently tracked pose. */
	public Pose2d getPose() {
		return this.odometry.getPose();
	}
	/** Get the full robot displacment from odometry since the class was constructed. */
	public Pose2d getOdometryDisplacement() {
		return this.odometry.getDisplacement();
	}
	/** Initialize a new pose from which odometry will track based off of */
	public void setPose(Pose2d new_pose) {
		this.odometry.reset(new_pose);
	}

	/** Access the rotation from the member gyro */
	public Rotation2d rawRotation() {
		return this.gyro.getRotation2d();
	}
	/** Access the rotation rate from the member gyro */
	public double rawRotationRate() {	// note that according to the gyro interface, this will be CW+ whereas Rotation2d is CCW+
		return this.gyro.getRate();
	}





	/** GenericSwerveDrive allows for multiple implementations of SwerveModule to be used in the same drivebase, but sacrifices specific module type functionality */
	public static class GenericSwerveDrive extends SwerveDrive<SwerveModule> {

		public GenericSwerveDrive(Gyro gyro, SwerveModule... modules) {
			super(gyro, modules);
		}

	}



	/** AugmentationParams encapsulates parameters for if optimization and/or normalization
	 * of module states should occur when applying a new target. */
	public static class AugmentationParams {

		final boolean
			should_normalize,
			should_optimize;
		final double
			max_wheel_velocity;

		/** Default false for optimization and normalization. */
		public AugmentationParams() { this(false); }
		/** Set whether optimization should occur. Normalization is assumed false. */
		public AugmentationParams(boolean s_opt) { this(s_opt, false, 0.0); }
		/** Set a maximum velocity for normalization. Optimization is assumed false. */
		public AugmentationParams(double max_wheel_velocity) { this(false, max_wheel_velocity != 0.0, max_wheel_velocity); }
		/** Set whether optimization should occur and a maximum velocity for normalization. */
		public AugmentationParams(boolean s_opt, double max_wheel_velocity) { this(s_opt, max_wheel_velocity != 0.0, max_wheel_velocity); }
		private AugmentationParams(boolean s_opt, boolean s_norm, double mvel) {
			this.should_normalize = s_norm;
			this.should_optimize = s_opt;
			this.max_wheel_velocity = mvel;
		}

	}


}

/**
 * Swerve control pipeline steps:
 * 1. raw ix iy ir
 * 2. apply deadband
 * 3. apply power scaling
 * 4. convert to measured velocity
 * 5. pre-normalize by estimated max velocity (?)
 * 5. slew rate limit -- (which target do we compare?)
 * 7. convert to constant curvature
 * 8. convert to module targets
 * 9. post-normalize by wheel velocity (? -- mutually exclusive with first pass)
 * 10. optimize rotations
 * 11. apply to modules
 */
