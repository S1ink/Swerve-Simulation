package frc.robot.swerve;

import edu.wpi.first.util.sendable.*;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.math.geometry.*;

import frc.robot.swerve.SwerveUtils.*;
import frc.robot.team3407.Util;


/** A high-level swerve drive abstraction that allows driving and simulation of any possible {@link SwerveModule} implementation,
 * and allows for any number of modules and physical layouts. */
public class SwerveDrive<Module_T extends SwerveModule> implements Subsystem, Sendable {

	private final Module_T[] modules;
	private final Gyro gyro;

	private final SwerveKinematics kinematics;
	private final SwerveOdometry odometry;
	private final SwerveVisualization visualization;

	public final int SIZE;

	private SwerveModuleStates[] states;


	public SwerveDrive(Gyro gyro, Module_T... modules) {
		this.modules = modules;
		this.gyro = gyro;
		this.SIZE = modules.length;
		this.states = new SwerveModuleStates[this.SIZE];

		final Translation2d[] locations = new Translation2d[this.SIZE];
		for(int i = 0; i < this.SIZE; i++) {
			locations[i] = this.modules[i].module_location;
			this.states[i] = this.modules[i].getStates();
		}
		this.kinematics = new SwerveKinematics(locations);
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





	/** GenericSwerveDrive allows for multiple implementations of SwerveModule to be used in the same drivebase, but sacrifices specific module type functionality */
	public static class GenericSwerveDrive extends SwerveDrive<SwerveModule> {

		public GenericSwerveDrive(Gyro gyro, SwerveModule... modules) {
			super(gyro, modules);
		}

	}


}
