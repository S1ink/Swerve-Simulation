package frc.robot.swerve;

import org.ejml.simple.SimpleMatrix;

import edu.wpi.first.math.geometry.*;
// import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;

import frc.robot.swerve.SwerveUtils.*;


/** SwerveOdometry keep track of the robot's pose when supplied with module sensor displacements.
 * This class is designed to integrate easily with {@link SwerveDrive} to minimize the external implementation. */
public class SwerveOdometry {

	protected final SwerveDrive<SwerveModule> sdrive;
	protected final double[]
		prev_wheel_displacements;
		// prev_module_rotations;

	protected Pose2d pose, pure_displacement = new Pose2d();
	protected Rotation2d prev_rotation, rel_transform;


	/** Create a SwerveOdometry for the provided SwerveDrive. */
	public <Module_T extends SwerveModule> SwerveOdometry(SwerveDrive<Module_T> sdrive)
		{ this(sdrive, new Pose2d()); }
	/** Create a SwerveOdometry for the provided SwerveDrive and set an initial pose. */
	public <Module_T extends SwerveModule> SwerveOdometry(SwerveDrive<Module_T> sdrive, Pose2d init) {
		this.sdrive = (SwerveDrive<SwerveModule>)sdrive;
		this.prev_wheel_displacements = new double[this.sdrive.SIZE];
		// this.prev_module_rotations = new double[this.sdrive.SIZE];
		this.reset(init);
	}

	/** Reset the tracked pose. */
	public void reset(Pose2d new_pose)
		{ this.reset(new_pose, this.sdrive.gyro.getRotation2d(), this.sdrive.states); }
	/** Reset the tracked pose and set the current gyro rotation and swerve module states. */
	public void reset(Pose2d new_pose, Rotation2d rotation, SwerveModuleStates... new_states) {
		if(new_states.length < this.sdrive.SIZE) return;

		this.pose = new_pose;
		this.prev_rotation = rotation;
		this.rel_transform = new_pose.getRotation().minus(rotation);
		for(int i = 0; i < this.sdrive.SIZE; i++) {
			this.prev_wheel_displacements[i] = new_states[i].linear_displacement;
			// this.prev_module_rotations[i] = new_states[i].rotation;
		}
	}

	/** Get the currently tracked pose. */
	public Pose2d getPose() {
		return this.pose;
	}
	/** Get the full displacement that the odometry has tracked since being initialized (independant of resets). */
	public Pose2d getDisplacement() {
		return this.pure_displacement;
	}

	/** Update the tracked poses using sensor data from the SwerveDrive instance. */
	public Pose2d update()
		{ return this.update(this.sdrive.gyro.getRotation2d(), this.sdrive.states); }
	/** Update the tracked pose using specific sensor data. */
	protected Pose2d update(Rotation2d new_rotation, SwerveModuleStates... new_states) {

		if(new_states.length < this.sdrive.SIZE) return this.pose;

		final SimpleMatrix deltas = new SimpleMatrix(this.sdrive.SIZE * 2, 1);
		for(int i = 0; i < this.sdrive.SIZE; i++) {
			final double
				dd = new_states[i].linear_displacement - this.prev_wheel_displacements[i],
				dx = dd * Math.cos(this.sdrive.states[i].rotation),		// non constant curvature computation for dx and dy
				dy = dd * Math.sin(this.sdrive.states[i].rotation);
				// dr = new_states[i].rotation - this.prev_module_rotations[i],
				// cr = dd / dr,
				// _dx = dr == 0.0 ? dd : cr * Math.sin(dr),		// swapped since the curvature travels up from +x, thus mimicing a 90 degree rotation
				// _dy = dr == 0.0 ? 0.0 : cr * (Math.cos(dr) - 1.0),
				// _cos = Math.cos(this.prev_module_rotations[i]),
				// _sin = Math.sin(this.prev_module_rotations[i]),
				// dx = _dx * _cos - _dy * _sin,
				// dy = _dx * _sin + _dy * _cos;

			deltas.set(i * 2 + 0, 0, dx);
			deltas.set(i * 2 + 1, 0, dy);

			this.prev_wheel_displacements[i] = new_states[i].linear_displacement;
			// this.prev_module_rotations[i] = new_states[i].rotation;
		}
		final SimpleMatrix delta = this.sdrive.kinematics.fwd_kinematics.mult(deltas);
		final Twist2d twist = new Twist2d(
			delta.get(0, 0),
			delta.get(1, 0),
			new_rotation.minus(this.prev_rotation).getRadians()
		);

		this.pure_displacement = this.pure_displacement.exp(twist);
		this.pose = new Pose2d(
			this.pose.exp(twist).getTranslation(),
			new_rotation.plus(this.rel_transform)
		);
		this.prev_rotation = new_rotation;

		return this.pose;

	}


}
