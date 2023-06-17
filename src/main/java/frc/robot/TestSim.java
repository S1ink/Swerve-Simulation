package frc.robot;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.Timer;


public class TestSim extends CommandBase {

	private final DoubleSupplier x_speed, y_speed, turn_speed;	// in meters per second and degrees per second
	private Pose2d robot_pose2d = new Pose2d();
	private Timer timer = new Timer();
	private Rotation2d[] wheel_rotations = new Rotation2d[4];

	public TestSim(DoubleSupplier xspeed, DoubleSupplier yspeed, DoubleSupplier trnspeed)
	{
		this.x_speed = xspeed;
		this.y_speed = yspeed;
		this.turn_speed = trnspeed;
		for(int i = 0; i < wheel_rotations.length; i++) {
			wheel_rotations[i] = new Rotation2d();
		}
	}

	@Override
	public void initialize() {
		this.timer.reset();
	}

	@Override
	public void execute() {

		final double dt = this.timer.get();
		this.timer.reset();

		double dx = x_speed.getAsDouble() * dt;
		double dy = y_speed.getAsDouble() * dt;
		double dθ = turn_speed.getAsDouble() * dt;

		this.robot_pose2d = this.robot_pose2d.exp(new Twist2d(dx, dy, dθ));

	}

	@Override
	public boolean isFinished() {
		return false;
	}
	
	@Override
	public void end(boolean i) {
		
	}

	private double[] getWheelPoseData() {
		return SwerveKinematics.toComponentData(SwerveKinematics.getWheelPoses3d(
			this.robot_pose2d, wheel_rotations
		));
	}

	@Override
	public void initSendable(SendableBuilder builder) {
		builder.addDoubleArrayProperty("Wheel Poses", this::getWheelPoseData, null);
		builder.addDoubleArrayProperty("Robot Pose", ()->new double[]{
			this.robot_pose2d.getX(),
			this.robot_pose2d.getY(),
			this.robot_pose2d.getRotation().getRadians()
		}, null);
	}

}
 
