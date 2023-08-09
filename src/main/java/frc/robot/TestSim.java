package frc.robot;

import java.util.Arrays;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.Timer;

import frc.robot.swerve.SwerveKinematics;
import frc.robot.swerve.SwerveUtils.*;
import frc.robot.team3407.Util;


public class TestSim extends CommandBase {

	private final DoubleSupplier x_speed, y_speed, turn_speed;	// in meters per second and degrees per second
	private Pose2d robot_pose2d = new Pose2d();
	private Timer timer = new Timer();
	private ChassisStates robot_vec = new ChassisStates();

	private SwerveKinematics kinematics;
	private SwerveVisualization visualization;
	private SwerveModuleStates[] wheel_states = new SwerveModuleStates[4];

	public TestSim(
		DoubleSupplier xspeed, DoubleSupplier yspeed, DoubleSupplier trnspeed,
		Translation2d... modules
	) {
		this.kinematics = new SwerveKinematics(modules);
		this.visualization = new SwerveVisualization(modules);

		this.x_speed = xspeed;
		this.y_speed = yspeed;
		this.turn_speed = trnspeed;

		Arrays.fill(this.wheel_states, new SwerveModuleStates());
	}

	@Override
	public void initialize() {
		this.robot_vec = new ChassisStates();
		this.timer.reset();
		this.timer.start();
	}

	@Override
	public void execute() {

		final double dt = this.timer.get();
		this.timer.reset();
		this.timer.start();

		final double
			vx = x_speed.getAsDouble(),
			vy = y_speed.getAsDouble(),
			vtheta = turn_speed.getAsDouble();

		ChassisStates.accFromDelta(this.robot_vec, new ChassisStates(vx, vy, vtheta), dt, this.robot_vec);
		this.wheel_states = this.kinematics.toModuleStates(this.robot_vec);

		this.robot_pose2d = this.robot_pose2d.exp(new Twist2d(
			this.robot_vec.x_velocity * dt,
			this.robot_vec.y_velocity * dt,
			this.robot_vec.angular_velocity * dt
		));

	}

	@Override
	public boolean isFinished() {
		return false;
	}
	
	@Override
	public void end(boolean i) {
		
	}

	@Override
	public void initSendable(SendableBuilder builder) {
		builder.addDoubleArrayProperty("Robot Pose", ()->Util.toComponents2d(this.robot_pose2d), null);
		builder.addDoubleArrayProperty("Wheel Poses", ()->Util.toComponents3d(this.visualization.getWheelPoses3d(this.wheel_states)), null);
		builder.addDoubleArrayProperty("Wheel Vectors", ()->SwerveVisualization.getVecComponents2d(this.wheel_states), null);
	}

}
