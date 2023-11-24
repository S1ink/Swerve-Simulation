package frc.robot;

import java.util.Arrays;
import java.util.LinkedList;
import java.util.List;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.controller.PIDController;

import frc.robot.swerve.*;
import frc.robot.swerve.SwerveUtils.*;
import frc.robot.swerve.simutil.*;
import frc.robot.swerve.simutil.FrictionModel.*;
import frc.robot.team3407.Util;
import frc.robot.team3407.SenderNT;
import frc.robot.team3407.SenderNT.RecursiveSendable;


public class TestSim extends CommandBase implements RecursiveSendable {

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
				.addDualGear(2e-5, 17, 15)
				.addGear(4.416e-4, WHEEL_RADIUS, 45);		// the radius is the wheel's radius
		private static final FrictionModel
			// steer_motor_frict,	// this is a bit extra but physically accurate --> will need to compensate for the GT summation already applying the GT friction model to the motor's node...
			// drive_motor_frict = steer_motor_frict = new StribeckFriction(0, 0, 0, 0),
			steer_gt_frict = new StribeckFriction(0.1, 0.005, 0.002, 0.02),
			drive_gt_frict = new StribeckFriction(0.1, 0.005, 0.002, 0.02),
			steer_floor_frict = new StribeckFriction(0.1, 0.05, 0.02, 0),
			wheel_side_frict = new StribeckFriction(0.1, 0.6, 0.5, 0);
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

		private static SimpleMotorFeedforward
			steer_ff = new SimpleMotorFeedforward(0, 0, 0),
			drive_ff = new SimpleMotorFeedforward(1.15, 0.53, 0);
		private PIDController
			steer_pid = new PIDController(10, 0, 0),
			drive_pid = new PIDController(2, 0, 0);
		private double
			va, vb,
			theta, target_theta,
			omega, target_omega,
			lx_wheel, lv_wheel,
			target_lv, target_la;
		private boolean
			stopped = false;
		private int error_state;

		public TestModule(Translation2d location) {
			super(location);
			this.steer_pid.enableContinuousInput(0, Math.PI * 2);
		}

		public synchronized void setVoltage(double steer_volts, double drive_volts) {
			this.va = steer_volts;
			this.vb = drive_volts;
		}

		@Override
		public void setState(double linear_vel, double steer_angle_rad, double linear_acc, double steer_angular_vel) {
			// optimize setpoint
			if(Math.abs(steer_angle_rad - this.theta) % (Math.PI * 2) > Math.PI / 2) {
				steer_angle_rad += Math.PI;
				steer_angle_rad %= (Math.PI * 2);
				linear_vel *= -1;
			}
			this.target_theta = steer_angle_rad;
			this.target_omega = steer_angular_vel;
			this.target_lv = linear_vel;
			this.target_la = linear_acc;
			this.stopped = false;
		}
		@Override
		public void stop() {
			this.va = this.vb = 0.0;
			this.stopped = true;
		}
		@Override
		public double getSteeringAngle() { return this.theta; }
		@Override
		public double getWheelDisplacement() { return this.lx_wheel; }
		@Override
		public double getSteeringRate() { return this.omega; }
		@Override
		public double getWheelVelocity() { return this.lv_wheel; }
		@Override
		public double getMotorAVolts() { return this.va; }
		@Override
		public double getMotorBVolts() { return this.vb; }
		@Override
		public SwerveModuleModel getSimProperties() { return TestModuleModel.inst; }
		@Override
		public void setSimulatedSteeringAngle(double theta) { this.theta = theta; }
		@Override
		public void setSimulatedSteeringRate(double omega) { this.omega = omega; }
		@Override
		public void setSimulatedWheelPosition(double x) { this.lx_wheel = x; }
		@Override
		public void setSimulatedWheelVelocity(double v) { this.lv_wheel = v; }

		@Override
		public void periodic(double dt) {

			if(!this.stopped) {

				// set PID dt somehow :(
				final double
					v_ff_steer = steer_ff.calculate(this.target_omega, (this.target_omega - this.omega) / dt),		// feedforward based on what speed we should be going
					v_fb_steer = steer_pid.calculate(this.theta, this.target_theta),		// feedback based on how far off the set position we are -- velocity PID as well?
					v_ff_drive = drive_ff.calculate(this.target_lv, this.target_la),		// feedforward based on what speed and acceleration we are targeting
					v_fb_drive = drive_pid.calculate(this.lv_wheel, this.target_lv);		// feedback based on how far off the set velocity we are

				if(Double.isNaN(v_ff_steer)) { this.error_state |= 0b0001; }
				if(Double.isNaN(v_fb_steer)) { this.error_state |= 0b0010; }
				if(Double.isNaN(v_ff_drive)) { this.error_state |= 0b0100; }
				if(Double.isNaN(v_fb_drive)) { this.error_state |= 0b1000; }

				this.setVoltage(
					v_ff_steer + v_fb_steer,
					v_ff_drive + v_fb_drive
				);

			}

		}

		@Override
		public void initSendable(SendableBuilder b) {
			super.initSendable(b);
			b.addDoubleProperty("Motor A Volts", ()->this.va, null);
			b.addDoubleProperty("Motor B Volts", ()->this.vb, null);
			b.addStringProperty("Error Code", ()->Integer.toBinaryString(this.error_state), null);
			b.addDoubleArrayProperty("States", ()->new double[]{
				this.theta, this.omega, this.lx_wheel, this.lv_wheel
			}, null);
			b.addDoubleArrayProperty("Target States", ()->new double[]{
				this.target_theta, this.target_omega, this.target_lv, this.target_la
			}, null);
		}


	}



	/** INSTANCE MEMBERS */

	private static final SwerveSimulator.SimConfig
		SIM_CONFIG = new SwerveSimulator.SimConfig(10.0, 5.0);

	private final TestModule[] modules;
	private final SwerveKinematics kinematics;
	private final SwerveVisualization visualization;
	private final SwerveSimulator simulator;
	private final SwerveFFCharacterization<TestModule> characterization;
	private final CommandBase steer_runner, drive_runner;

	private final DoubleSupplier x_speed, y_speed, turn_speed;	// in meters per second and degrees per second
	private Pose2d direct_pose2d = new Pose2d(), odo_pose2d = new Pose2d();
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
		this.characterization = new SwerveFFCharacterization<>(this.modules);
		this.steer_runner = this.characterization.steerCommand((TestModule m, double v)->m.setVoltage(v, 0.0));
		this.drive_runner = this.characterization.driveCommand((TestModule m, double v)->m.setVoltage(0.0, v));

		this.x_speed = xspeed;
		this.y_speed = yspeed;
		this.turn_speed = trnspeed;

		this.wheel_states = new SwerveModuleStates[this.modules.length];
		Arrays.fill(this.wheel_states, new SwerveModuleStates());
	}


	public SwerveSimulator getSim() { return this.simulator; }
	public CommandBase getSteerCharacterization() { return this.steer_runner; }
	public CommandBase getDriveCharacterization() { return this.drive_runner; }
	public void periodic(double dt) {
		this.simulator.integrate(dt);
		this.simulator.applyStates();
		for(SwerveModule m : this.modules) {
			m.periodic(dt);
		}
	}


	@Override
	public void initialize() {
		this.robot_vec.zero();
		this.timer.restart();
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
		this.kinematics.toModuleStates(this.robot_vec, this.wheel_states);
		ChassisStates back = this.kinematics.toChassisStates(this.wheel_states).toFieldRelative(this.odo_pose2d.getRotation().getRadians());

		this.direct_pose2d = this.direct_pose2d.exp(this.robot_vec.integrate(dt));
		this.odo_pose2d = this.odo_pose2d.exp(back.integrate(dt));

		for(int i = 0; i < this.modules.length; i++) {
			this.modules[i].setState(this.wheel_states[i]);
		}

	}

	@Override
	public boolean isFinished() {
		return false;
	}
	
	@Override
	public void end(boolean i) {
		for(TestModule m : this.modules) {
			m.stop();
		}
	}

	@Override
	public void initSendable(SendableBuilder builder) {
		// builder.addDoubleProperty("A Volts", this.turn_speed, null);
		// builder.addDoubleProperty("B Volts", this.x_speed, null);
		builder.addDoubleArrayProperty("Robot Pose", ()->Util.toComponents2d(this.direct_pose2d), null);
		builder.addDoubleArrayProperty("Integrated Pose", ()->Util.toComponents2d(this.odo_pose2d), null);
		builder.addDoubleArrayProperty("Wheel Poses", ()->Util.toComponents3d(this.visualization.getWheelPoses3d(this.wheel_states)), null);
		builder.addDoubleArrayProperty("Wheel Vectors", ()->SwerveVisualization.getVecComponents2d(this.wheel_states), null);
	}

	@Override
	public void initRecursive(SenderNT inst, String base) {

		inst.putData(base, (Sendable)this);
		inst.putData(base + "/Chassis State", this.robot_vec);
		for(int i = 0; i < this.modules.length; i++) {
			inst.putData(String.format("%s/Module[%d]", base, i), this.modules[i]);
		}
		inst.putData(base + "/Steering Characterization", this.steer_runner);
		inst.putData(base + "/Driving Characterization", this.drive_runner);

	}



	public static class TestSim2 extends CommandBase {

		private final TestModule[] modules;
		public final SwerveSimulator simulator;
		private final DoubleSupplier
			steer_volts_1,
			steer_volts_2,
			steer_volts_3,
			steer_volts_4,
			drive_volts;

		public TestSim2(
			DoubleSupplier sv1,
			DoubleSupplier sv2,
			DoubleSupplier sv3,
			DoubleSupplier sv4,
			DoubleSupplier dv,
			Translation2d... modules
		) {
			this.modules = new TestModule[modules.length];
			for(int i = 0; i < this.modules.length; i++) {
				this.modules[i] = new TestModule(modules[i]);
			}
			this.simulator = new SwerveSimulator(SIM_CONFIG, TestModuleModel.inst, this.modules);

			this.steer_volts_1 = sv1;
			this.steer_volts_2 = sv2;
			this.steer_volts_3 = sv3;
			this.steer_volts_4 = sv4;
			this.drive_volts = dv;
		}

		@Override
		public void initialize() {

		}
		@Override
		public void execute() {
			final double
				sv1 = this.steer_volts_1.getAsDouble(),
				sv2 = this.steer_volts_2.getAsDouble(),
				sv3 = this.steer_volts_3.getAsDouble(),
				sv4 = this.steer_volts_4.getAsDouble(),
				dv = this.drive_volts.getAsDouble();
			this.modules[0].setVoltage(sv1, dv);
			this.modules[1].setVoltage(sv2, dv);
			this.modules[2].setVoltage(sv3, dv);
			this.modules[3].setVoltage(sv4, dv);
		}
		@Override
		public void end(boolean i) {
			for(TestModule m : this.modules) {
				m.setVoltage(0, 0);
			}
		}


	}



	public static class SwerveFFCharacterization<Module_T extends SwerveModule> {

		public static final double
			DEFAULT_VOLTAGE_RAMP = 8,
			DEFAULT_MAX_VOLTS = 12.0,
			DEFAULT_VELOCITY_CUTOFF = 1e-3;

		private final Module_T[] modules;
		private double
			voltage_ramp = DEFAULT_VOLTAGE_RAMP,
			max_voltage = DEFAULT_MAX_VOLTS,
			velocity_cutoff = DEFAULT_VELOCITY_CUTOFF;

		public SwerveFFCharacterization(Module_T... modules) {
			this.modules = modules;
		}


		public void setVoltageRamp(double volts_per_second) {
			this.voltage_ramp = volts_per_second;
		}
		public void setMaxVoltage(double max_volts) {
			this.max_voltage = max_volts;
		}
		public void setVelocityCutoff(double min_velocity) {
			this.velocity_cutoff = Math.abs(min_velocity);
		}

		public CommandBase steerCommand(ModuleDoubleConsumer<? super Module_T> steer_voltage_setter) {
			return new SteerCharacterization(steer_voltage_setter);
		}
		public CommandBase driveCommand(ModuleDoubleConsumer<? super Module_T> drive_voltage_setter) {
			return new DriveCharacterization(drive_voltage_setter);
		}


		private class BaseCharacterization extends CommandBase {

			protected final ModuleDoubleConsumer<? super Module_T> voltage_setter;
			protected final ModuleDoubleSupplier<? super Module_T> voltage_getter, velocity_getter;
			protected final List<Double>[]
				voltages = new LinkedList[modules.length + 1],
				velocities = new LinkedList[modules.length + 1];
			protected final Timer timer = new Timer();
			protected double
				running_avg_volts = 0.0,
				running_avg_vel = 0.0,
				running_kS = 0.0,
				running_kV = 0.0,
				running_R2 = 0.0;

			public BaseCharacterization(
				ModuleDoubleConsumer<? super Module_T> v_set,
				ModuleDoubleSupplier<? super Module_T> v_get,
				ModuleDoubleSupplier<? super Module_T> vel_get
			) {
				this.voltage_setter = v_set;
				this.voltage_getter = v_get;
				this.velocity_getter = vel_get;
				for(int i = 0; i < modules.length + 1; i++) {
					this.voltages[i] = new LinkedList<>();
					this.velocities[i] = new LinkedList<>();
				}
			}


			@Override
			public void initialize() {
				System.out.println("Characterization Init!?");
				this.timer.restart();
			}
			@Override
			public void execute() {
				final double volts = Util.clamp(timer.get() * voltage_ramp, -max_voltage, max_voltage);
				double avg_voltage = 0.0, avg_velocity = 0.0;
				for(int i = 0; i < modules.length; i++) {
					final Module_T m = modules[i];
					this.voltage_setter.set(m, volts);	// set voltage per module
					final double
						vlt = this.voltage_getter == null ? volts : this.voltage_getter.get(m),	// actual voltage per module
						vel = this.velocity_getter.get(m);		// get the velocity
					if(Math.abs(vel) > velocity_cutoff) {
						this.voltages[i].add(vlt);
						this.velocities[i].add(vel);
					}
					avg_voltage += vlt;
					avg_velocity += vel;
				}
				this.running_avg_volts = (avg_voltage /= modules.length);
				this.running_avg_vel = (avg_velocity /= modules.length);
				if(Math.abs(avg_velocity) > velocity_cutoff) {
					this.voltages[modules.length].add(avg_voltage);
					this.velocities[modules.length].add(avg_velocity);
				}
				if(this.voltages[modules.length].size() > 1 && this.velocities[modules.length].size() > 1) {
					PolynomialRegression p = new PolynomialRegression(
						velocities[modules.length].stream().mapToDouble(Double::doubleValue).toArray(),
						voltages[modules.length].stream().mapToDouble(Double::doubleValue).toArray(),
						1
					);
					this.running_kS = p.beta(0);
					this.running_kV = p.beta(1);
					this.running_R2 = p.R2();
				}
			}
			@Override
			public void end(boolean i) {
				System.out.println("Characterization End!?");
				this.timer.stop();
				for(Module_T m : modules) {
					m.stop();
				}
			}

			@Override
			public void initSendable(SendableBuilder b) {
				b.addDoubleProperty("kS", ()->this.running_kS, null);
				b.addDoubleProperty("kV", ()->this.running_kV, null);
				b.addDoubleProperty("R-Squared", ()->this.running_R2, null);
				b.addDoubleProperty("Average Output Voltage", ()->this.running_avg_volts, null);
			}


		}

		protected class SteerCharacterization extends BaseCharacterization {

			public SteerCharacterization(ModuleDoubleConsumer<? super Module_T> voltage_setter) {
				super(
					voltage_setter,
					(Module_T m)->m.getMotorAVolts(),
					(Module_T m)->m.getSteeringRate()
				);
			}

			@Override
			public void initSendable(SendableBuilder b) {
				super.initSendable(b);
				b.addDoubleProperty("Average Angular Velocity (radps)", ()->super.running_avg_vel, null);
			}

		}
		protected class DriveCharacterization extends BaseCharacterization {

			public DriveCharacterization(ModuleDoubleConsumer<? super Module_T> voltage_setter) {
				super(
					voltage_setter,
					(Module_T m)->m.getMotorBVolts(),
					(Module_T m)->m.getWheelVelocity()
				);
			}

			@Override
			public void initSendable(SendableBuilder b) {
				super.initSendable(b);
				b.addDoubleProperty("Average Linear Velocity (mps)", ()->super.running_avg_vel, null);
			}

		}

	}


}
