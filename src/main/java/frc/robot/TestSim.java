package frc.robot;

import java.util.Arrays;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.Timer;

import frc.robot.swerve.*;
import frc.robot.swerve.SwerveUtils.*;
import frc.robot.swerve.simutil.*;
import frc.robot.swerve.simutil.FrictionModel.*;
import frc.robot.team3407.Util;


public class TestSim extends CommandBase {

	public static class TestModuleModel implements SwerveModuleModel {

		public static TestModuleModel inst = new TestModuleModel();
		private TestModuleModel() {}

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
			// steer_motor_frict,	// this is a bit extra but physically accurate --> will need to compensate for the GT summation already applying the GT friction model to the motor's node...
			// drive_motor_frict = steer_motor_frict = new StribeckFriction(0, 0, 0, 0),
			steer_gt_frict = new StribeckFriction(0.1, 0.005, 0.002, 0.02),
			drive_gt_frict = new StribeckFriction(0.1, 0.005, 0.002, 0.02),
			steer_floor_frict = new StribeckFriction(0.1, 0.05, 0.02, 0),
			wheel_side_frict = new StribeckFriction(0.1, 0.3, 0.2, 0);
		private static final double
			MODULE_STATIC_RI = 0.013,		// about the steer axis, or wherever the module's measured center is
			MODULE_STATIC_LI = 2.115,
			STEER_GT_RI = steer_gt.endInertia(),
			DRIVE_GT_RI = drive_gt.endInertia();
			// STEER_GT_RATIO = steer_gt.fwdRatio(),	// the FWD ratio -- from motor to output -> use the inverse in reverse operations
			// DRIVE_GT_RATIO = drive_gt.fwdRatio();	// the FWD ratio -- from motor to wheel -> use the inverse in reverse operations


		@Override
		public double steerAAccel(
			double a_volts, double b_volts,
			double steer_rate, double wheel_vel_linear,
			double f_norm,
			double dt
		) {
			final double
			// the motor's rotational rate
				av_motor = steer_gt.propegateVelRev(steer_rate),
			// the entire geartrain's momentum @ the output
				p_gtrain = steer_gt.endInertia() * steer_rate,
			// the raw torque as result of applied voltage
				tq_raw = steer_gt.propegateTq(DCMotorModel.getRawTorque(steer_motor, a_volts)),	// motor torque geared @ output
			// sum of frictions in the geartrain and motor (and back EMF)
				tq_frict = ( DCMotorModel.getEMFTorque(steer_motor, av_motor)
					+ steer_gt.sumFrictionRev(steer_rate, tq_raw, steer_gt_frict)
					+ steer_floor_frict.calc(f_norm, steer_rate, tq_raw) ),
			// apply friction
				tq_sum = FrictionModel.applyFriction(tq_raw, p_gtrain, tq_frict, dt);
			// return net torque (@ output) / inertia of the entire chain (@ output)
			return tq_sum / STEER_GT_RI;

		}
		@Override
		public double wheelForceM(
			double a_volts, double b_volts,
			double steer_rate, double wheel_vel_linear,
			double dt
		) {
			return drive_gt.propegateTq(DCMotorModel.getRawTorque(drive_motor, b_volts)) / WHEEL_RADIUS;
		}
		@Override
		public double wheelSideFriction(
			double f_norm, double f_app, double vel_linear, double dt
		) {
			return wheel_side_frict.calc(f_norm, vel_linear, f_app);
		}
		@Override
		public double wheelGearFriction(
			double f_norm, double f_app,
			double a_volts, double b_volts,
			double steer_rate, double wheel_vel_linear,
			double dt
		) {
			final double
				av_wheel = wheel_vel_linear / WHEEL_RADIUS,
				av_motor = drive_gt.propegateVelRev(av_wheel),
				tq_frict = ( DCMotorModel.getEMFTorque(drive_motor, av_motor)
					+ drive_gt.sumFrictionRev(av_wheel, f_app * WHEEL_RADIUS, drive_gt_frict) );
			return tq_frict;
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
				lI_gt = DRIVE_GT_RI / (WHEEL_RADIUS * WHEEL_RADIUS),
				lI2 = MODULE_STATIC_LI * MODULE_STATIC_LI;
			return Math.sqrt(lI2 + (2.0 * MODULE_STATIC_LI + lI_gt) * lI_gt * cos2);
		}
		@Override
		public double effectiveRotationalInertia(
			double vec_wheel_dtheta, double module_radius
		) {
			final double lI_v = this.effectiveLinearInertia(vec_wheel_dtheta);
			return MODULE_STATIC_RI + lI_v * module_radius * module_radius;
		}


	}
	public static class TestModule extends SwerveModule {

		private double va, vb;

		public TestModule(Translation2d location) {
			super(location);
		}

		public synchronized void setVoltage(double steer_volts, double drive_volts) {
			this.va = steer_volts;
			this.vb = drive_volts;
		}

		@Override
		public void setState(double linear_vel, double steer_angle_rad, double linear_acc, double steer_angular_vel) {
			this.va = steer_angular_vel;
			this.vb = linear_vel;
		}
		@Override
		public double getSteeringAngle() { return 0.0; }
		@Override
		public double getWheelDisplacement() { return 0.0; }
		@Override
		public double getMotorAVolts() { return this.va; }
		@Override
		public double getMotorBVolts() { return this.vb; }
		@Override
		public SwerveModuleModel getSimProperties() { return TestModuleModel.inst; }
		@Override
		public void setSimulatedSteeringAngle(double angle) {}
		@Override
		public void setSimulatedSteeringRate(double omega) {}
		@Override
		public void setSimulatedWheelPosition(double angle) {}
		@Override
		public void setSimulatedWheelVelocity(double omega) {}


	}



	/** INSTANCE MEMBERS */

	private static final SwerveSimulator.SimConfig
		SIM_CONFIG = new SwerveSimulator.SimConfig(10.0, 5.0);

	private final TestModule[] modules;
	private final SwerveKinematics kinematics;
	private final SwerveVisualization visualization;
	private final SwerveSimulator simulator;

	private final DoubleSupplier x_speed, y_speed, turn_speed;	// in meters per second and degrees per second
	private Pose2d robot_pose2d = new Pose2d();
	private Timer timer = new Timer();
	private ChassisStates robot_vec = new ChassisStates();

	private SwerveModuleStates[] wheel_states;

	public TestSim(
		DoubleSupplier xspeed, DoubleSupplier yspeed, DoubleSupplier trnspeed,
		Translation2d... modules
	) {
		this.modules = new TestModule[modules.length];
		for(int i = 0; i < this.modules.length; i++) {
			this.modules[i] = new TestModule(modules[i]);
		}
		this.kinematics = new SwerveKinematics(modules);
		this.visualization = new SwerveVisualization(modules);
		this.simulator = new SwerveSimulator(this.visualization, SIM_CONFIG, TestModuleModel.inst, this.modules);

		this.x_speed = xspeed;
		this.y_speed = yspeed;
		this.turn_speed = trnspeed;

		this.wheel_states = new SwerveModuleStates[this.modules.length];
		Arrays.fill(this.wheel_states, new SwerveModuleStates());
	}


	public SwerveSimulator getSim() { return this.simulator; }

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

		// for(int i = 0; i < this.modules.length; i++) {
		// 	this.modules[i].setState(this.wheel_states[i]);
		// }
		this.modules[0].setVoltage(vy, vx);
		this.modules[1].setVoltage(vtheta, vx);

	}

	@Override
	public boolean isFinished() {
		return false;
	}
	
	@Override
	public void end(boolean i) {
		for(TestModule m : this.modules) {
			m.setVoltage(0.0, 0.0);
		}
	}

	@Override
	public void initSendable(SendableBuilder builder) {
		// builder.addDoubleProperty("A Volts", this.turn_speed, null);
		// builder.addDoubleProperty("B Volts", this.x_speed, null);
		builder.addDoubleArrayProperty("Robot Pose", ()->Util.toComponents2d(this.robot_pose2d), null);
		builder.addDoubleArrayProperty("Wheel Poses", ()->Util.toComponents3d(this.visualization.getWheelPoses3d(this.wheel_states)), null);
		builder.addDoubleArrayProperty("Wheel Vectors", ()->SwerveVisualization.getVecComponents2d(this.wheel_states), null);
	}

}
