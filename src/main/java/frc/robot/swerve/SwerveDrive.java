package frc.robot.swerve;

import edu.wpi.first.util.sendable.*;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.swerve.SwerveUtils.*;
import frc.robot.team3407.Util;


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

	public final int SIZE;


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
		}
		this.kinematics = new SwerveKinematics(locations);
		this.lock_states = this.kinematics.lockFormation(null);
		this.odometry = null;
		// this.odometry = new SwerveDriveOdometry(this.kinematics, this.gyro.getRotation2d(), this.positions);		// update with new odometry
		this.visualization = new SwerveVisualization(locations);
	}


	protected void updateCachedStates() {
		for(int i = 0; i < this.SIZE; i++) {
			this.states[i] = this.modules[i].getStates();
		}
	}

	@Override
	public void periodic() {
		this.updateCachedStates();
		// this.odometry.update(
		// 	this.gyro.getRotation2d(),
		// 	this.positions
		// );
	}

	@Override
	public void initSendable(SendableBuilder b) {
		// b.addDoubleArrayProperty("Odometry", ()->Util.toComponents2d(this.odometry.getPoseMeters()), null);
		b.addDoubleArrayProperty("Module Poses 3d", ()->Util.toComponents3d(this.visualization.getWheelPoses3d(this.states)), null);
		b.addDoubleArrayProperty("Wheel Vectors 2d", ()->SwerveVisualization.getVecComponents2d(this.states), null);
	}


	/** Update the swerve's target state. Should be relative to the robot's coordinate frame. */
	public void applyTarget(ChassisSpeeds target_state) {
		this.applyTarget(new ChassisStates(target_state));
	}
	/** Update the swerve's target state. Should be relative to the robot's coordinate frame. */
	public void applyTarget(ChassisStates target_state) {
		this.targets = this.kinematics.toModuleStates(target_state, this.targets);	// targets is updated inline, although a new buffer is generated on size mismatches so reset anyway
		// SwerveKinematics.normalizeModuleVelocities(MAX_VELOCITY, this.targets);	// need to figure out where MAX_VELOCITY belongs
		for(int i = 0; i < this.SIZE; i++) {
			SwerveModuleStates.optimize(
				this.targets[i],
				this.modules[i].getSteeringAngle(),
				this.targets[i]
			);
			this.modules[i].setState(this.targets[i]);
		}
	}
	public void applyLocked() {
		for(int i = 0; i < this.SIZE; i++) {
			this.modules[i].setState(this.lock_states[i]);
		}
	}





	/** GenericSwerveDrive allows for multiple implementations of SwerveModule to be used in the same drivebase, but sacrifices specific module type functionality */
	public static class GenericSwerveDrive extends SwerveDrive<SwerveModule> {

		public GenericSwerveDrive(Gyro gyro, SwerveModule... modules) {
			super(gyro, modules);
		}

	}


}
