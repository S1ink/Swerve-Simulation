package frc.robot;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.Timer;

import frc.robot.swerve.*;
import frc.robot.swerve.SwerveUtils.*;
import frc.robot.swerve.simutil.*;
import frc.robot.swerve.simutil.FrictionModel.*;
import frc.robot.team3407.Util;


public class TestSim extends CommandBase {

	public static class TestModuleSim implements SwerveModuleModel {

		private static final double WHEEL_RADIUS = 0.04699;
		private static final DCMotor
			steer_motor = DCMotor.getFalcon500(1),
			drive_motor = DCMotor.getFalcon500(1);
		private static final GearTrainModel
			steer_gt = GearTrainModel.create()
				.addGear(2e-5, 0.0, 14)
				.addGearPulley(5.433e-5, 50, 10)
				.addGear(6.403e-4, 0.0, 60),
			drive_gt = GearTrainModel.create()
				.addGear(2e-5, 0.0, 14)
				.addGear(5.382e-5, 0.0, 50)
				.addDualGear(7.389e-5, 50, 27)
				.addDualGear(2e-5, 19, 15)
				.addGear(4.416e-4, WHEEL_RADIUS, 45);		// the radius is the wheel's radius
		private static final FrictionModel
			steer_gt_frict = new StribeckFriction(0, 0, 0, 0),
			drive_gt_frict = new StribeckFriction(0, 0, 0, 0),
			steer_floor_frict = new StribeckFriction(0, 0, 0, 0),
			wheel_side_frict = new StribeckFriction(0, 0, 0, 0);
		private static final double
			MODULE_STATIC_RI = 0.013,		// about the steer axis, or wherever the module's measured center is
			MODULE_STATIC_LI = 2.115,
			STEER_GT_RI = steer_gt.endInertia(),
			DRIVE_GT_RI = drive_gt.endInertia(),
			STEER_GT_RATIO = steer_gt.fwdRatio(),	// the FWD ratio -- from motor to output -> use the inverse in reverse operations
			DRIVE_GT_RATIO = drive_gt.fwdRatio();	// the FWD ratio -- from motor to wheel -> use the inverse in reverse operations

		@Override
		public double steerAAccel(
			double a_volts, double b_volts,
			double steer_rate, double wheel_vel_linear,
			double f_norm
		) {
			final double
				m_av = steer_gt.propegateVelRev(steer_rate),	// turn rate to motor speed via gearing
				gt_p = steer_gt.endInertia() * steer_rate,
				o_tq = steer_gt.propegateTq(steer_motor.getTorque(steer_motor.getCurrent(m_av, a_volts))),	// motor torque geared @ output
				f_tq = steer_gt.sumFrictionRev(steer_rate, o_tq, steer_gt_frict) + steer_floor_frict.calc(f_norm, steer_rate, o_tq),
				s_tq = FrictionModel.applyFriction(o_tq, gt_p, f_tq, 0.0);
			return s_tq / STEER_GT_RI;

		}
		@Override
		public double wheelForceM(
			double a_volts, double b_volts,
			double steer_rate, double wheel_vel_linear
		) {
			final double
				m_av = (wheel_vel_linear / WHEEL_RADIUS) / DRIVE_GT_RATIO,
				o_tq = drive_motor.getTorque(drive_motor.getCurrent(m_av, b_volts)) / DRIVE_GT_RATIO;
			return o_tq / WHEEL_RADIUS;
		}
		@Override
		public double wheelSideFriction(
			double f_norm, double f_app, double vel_linear
		) {
			return 0.0;
		}
		@Override
		public double wheelGearFriction(
			double f_norm, double f_app,
			double a_volts, double b_volts,
			double steer_rate, double wheel_vel_linear
		) {
			return 0.0;
		}
		@Override
		public double moduleMass() {
			return MODULE_STATIC_LI;
		}
		@Override
		public double effectiveLinearInertia(
			double vec_wheel_dtheta
		) {
			// checks for max/min shortcuts
			final double
				cos = Math.cos(vec_wheel_dtheta),
				cos2 = cos * cos,
				IR_l = drive_gt.endInertia() / (WHEEL_RADIUS * WHEEL_RADIUS),
				IL2 = MODULE_STATIC_LI * MODULE_STATIC_LI;
			return Math.sqrt(IL2 + (2.0 * MODULE_STATIC_LI + IR_l) * IR_l * cos2);
		}
		@Override
		public double effectiveRotationalInertia(
			double vec_wheel_dtheta, double module_radius
		) {
			final double LI_v = this.effectiveLinearInertia(vec_wheel_dtheta);
			return MODULE_STATIC_RI + LI_v * module_radius * module_radius;
		}


	}
	public static class TestModule extends SwerveModule {

		public TestModule() {
			super(null);
		}

		@Override
		public void setState(double linear_vel, double steer_angle_rad, double linear_acc, double steer_angular_vel) {}
		@Override
		public double getSteeringAngle() { return 0.0; }
		@Override
		public double getWheelDisplacement() { return 0.0; }
		@Override
		public double getMotorAVolts() { return 0.0; }
		@Override
		public double getMotorBVolts() { return 0.0; }
		@Override
		public void setSimulatedSteeringAngle(double angle) {}
		@Override
		public void setSimulatedSteeringRate(double omega) {}
		@Override
		public void setSimulatedWheelPosition(double angle) {}
		@Override
		public void setSimulatedWheelVelocity(double omega) {}


	}

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
