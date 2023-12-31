package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.swerve.*;
import frc.robot.swerve.SwerveUtils.*;
import frc.robot.team3407.Util;


public class TestDrive extends CommandBase implements Gyro {

	public static class PoseIntegrator {

		public Pose2d pose = new Pose2d();

		public Pose2d iterate(ChassisStates v, double dt) {
			return this.iterate(
				v.x_velocity,
				v.y_velocity,
				v.angular_velocity,
				dt
			);
		}
		public Pose2d iterate(double vx, double vy, double vr, double dt) {
			final Twist2d dx = new Twist2d(
				vx * dt,
				vy * dt,
				vr * dt
			);
			return (this.pose = this.pose.exp(dx));
		}

	}

	public static class DummyModule extends SwerveModule {

		private double
			target_vel, target_angle,
			s_wheel_pos,
			s_wheel_vel,
			s_rotation,
			s_rotation_rate;

		protected DummyModule(Translation2d loc) {
			super(loc);
		}


		@Override
		public void periodic(double dt) {
			this.s_wheel_pos += (this.target_vel + this.s_wheel_vel) / 2.0 * dt;	// trapazoid sum
			this.s_rotation_rate = (this.target_angle - this.s_rotation) / dt;		// rate of change
			this.s_wheel_vel = this.target_vel;
			this.s_rotation = this.target_angle;
		}

		@Override
		public void setState(double linear_vel, double steer_angle_rad, double linear_acc, double steer_angular_vel) {
			this.target_vel = linear_vel;
			this.target_angle = steer_angle_rad;
		}
		@Override
		public void stop() {
			this.target_vel = 0.0;
		}

		@Override public double getSteeringAngle() { return this.s_rotation; }
		@Override public double getWheelDisplacement() { return this.s_wheel_pos; }
		@Override public double getSteeringRate() { return this.s_rotation_rate; }
		@Override public double getWheelVelocity() { return this.s_wheel_vel; }


	}




	public static final double
		DEADBAND = 0.05,
		VELOCITY_SCALE = 4.0,
		ROTATION_VEL_SCALE = 2 * Math.PI,
		LINEAR_ACC_LIMIT = 9.8,
		ROTATIONAL_ACC_LIMIT = 2 * Math.PI;

	protected SwerveDrive<DummyModule> sdrive;
	protected XboxController xbox;
	protected ChassisStates prev_target = new ChassisStates();
	protected Rotation2d prev_rotation = new Rotation2d();
	protected PoseIntegrator
		target = new PoseIntegrator(),
		pure = new PoseIntegrator();

	public TestDrive(XboxController input) {
		this.xbox = input;
		final DummyModule[] modules = new DummyModule[4];
		final Translation2d[] locs = SwerveUtils.makeSquareLocationsCW(0.263525);
		for(int i = 0; i < 4; i++) {
			modules[i] = new DummyModule(locs[i]);
		}
		this.sdrive = new SwerveDrive<>(this, modules);
		this.sdrive.register();
		this.sdrive.smartDashboardInit("Test SwerveDrive");
	}


	@Override
	public void initSendable(SendableBuilder b) {
		b.addDoubleArrayProperty("Pure Target", ()->Util.toComponents2d(this.pure.pose), null);
		b.addDoubleArrayProperty("Target", ()->Util.toComponents2d(this.target.pose), null);
	}

	@Override
	public void initialize() {}

	@Override
	public void execute() {
		final double dt = 0.02;
		if(this.xbox.getLeftBumper()) {
			this.sdrive.stop();
			this.prev_target.zero();
			this.pure.pose = new Pose2d();
			this.prev_rotation = this.target.pose.getRotation();
			this.target.pose = new Pose2d();
			this.sdrive.setPose(new Pose2d());
		} else if(this.xbox.getRightBumper()) {
			this.sdrive.applyLocked();
			this.prev_target.zero();
			this.pure.iterate(0.0, 0.0, 0.0, dt);
		} else {
			final double
				vx = -MathUtil.applyDeadband(this.xbox.getRightY(), DEADBAND) * VELOCITY_SCALE,
				vy = -MathUtil.applyDeadband(this.xbox.getRightX(), DEADBAND) * VELOCITY_SCALE,
				vr = -MathUtil.applyDeadband(this.xbox.getLeftX(), DEADBAND) * ROTATION_VEL_SCALE;
			final ChassisStates target = new ChassisStates(vx, vy, vr);
			ChassisStates.normalizeByMaximumV(target, Math.hypot(0.263525, 0.263525), VELOCITY_SCALE);
			ChassisStates.rateLimitVelocities(target, this.prev_target, dt, LINEAR_ACC_LIMIT, ROTATIONAL_ACC_LIMIT);
			ChassisStates.descretizeCurvature(target, dt);
			this.sdrive.applyTarget(target);
			this.prev_target = target;
			this.pure.iterate(vx, vy, vr, dt);
		}
		this.target.iterate(this.prev_target, dt);
	}

	@Override
	public boolean isFinished() { return false; }
	@Override
	public void end(boolean i) {
		this.sdrive.stop();
	}
	@Override
	public boolean runsWhenDisabled() { return true; }



	@Override public void close() throws Exception {}
	@Override public void calibrate() {}
	@Override public void reset() {}

	@Override
	public Rotation2d getRotation2d() {
		return this.target.pose.getRotation().plus(this.prev_rotation);
	}
	@Override
	public double getAngle() {
		// TODO Auto-generated method stub
		return 0;
	}
	@Override
	public double getRate() {
		// TODO Auto-generated method stub
		return 0;
	}


}
