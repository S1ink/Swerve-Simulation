package frc.robot.swerve;

import org.ejml.simple.SimpleMatrix;

import edu.wpi.first.math.*;
import edu.wpi.first.math.numbers.*;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.system.NumericalIntegration;
import edu.wpi.first.util.sendable.*;

import frc.robot.swerve.SwerveDrive.*;
import frc.robot.swerve.SwerveUtils.*;
import frc.robot.swerve.simutil.*;


/** SwerveSimulator applies the "high-level" physics computation and integration required to simulate a
 * a swerve base's movement as a result of motor voltages. The architecture allows for most of the module-specific
 * physical properties to be reimplemented and swapped using various interfaces. In this way, close to none of
 * the simulator's properties are locked in - the number of modules used, and module physical properties can all be
 * reimplemented. */
public class SwerveSimulator implements Sendable {

	public static class SimConfig {

		public final double
			ROBOT_MASS,
			ROBOT_RI;

		public SimConfig(
			double robot_mass, double robot_RI
		) {
			this.ROBOT_MASS = robot_mass;
			this.ROBOT_RI = robot_RI;
		}

	}


	private static final class NX extends Num implements Nat<NX> {

		private final int value;
		public NX(int v) { this.value = v; }

		@Override public int getNum() { return this.value; }

		static NX of(int v) { return new NX(v); }
		static NX of_x(int v, int scale) { return new NX(v * scale); }

	}

	protected enum State {
		FramePositionX		(-6),
		FramePositionY		(-5),
		FrameVelocityX		(-4),
		FrameVelocityY		(-3),
		FrameRotation		(-2),
		FrameAngularVel		(-1),

		SteerAngle			(0),
		SteerRate			(1),
		DrivePosition		(2),
		DriveVelocity		(3);

		public int idx;
		private State(int i) { this.idx = i; }

		public double from(Matrix<NX, N1> x) { return this.fromN(x, 0); }
		public double fromN(Matrix<NX, N1> x, int module) {
			if(this.idx < 0) {
				return x.get(x.getNumRows() + this.idx, 0);
			}
			return x.get(module * 4 + this.idx, 0);
		}
		public void set(Matrix<NX, N1> x_, double val) { this.setN(x_, 0, val); }
		public void setN(Matrix<NX, N1> x_, int module, double val) {
			if(this.idx < 0) {
				x_.set(x_.getNumRows() + this.idx, 0, val);
			}
			x_.set(module * 4 + this.idx, 0, val);
		}
	}



	/** INSTANCE MEMBERS */

	private final SwerveModule[] modules;
	private final SwerveModuleModel[] module_models;
	private final SimConfig config;
	private final SwerveVisualization visualization;
	private final Vector2[] module_locs, module_dirs;
	private final int SIZE;
	private final NX n_inputs, n_states;
	private double STATIC_MASS;
	// locations2d?

	private final Matrix<NX, N1> u_inputs;
	private Matrix<NX, N1> x_states, y_outputs;


	public SwerveSimulator(SimConfig config, SwerveModule... modules) {
		this(config, null, modules);
	}
	public SwerveSimulator(SimConfig config, SwerveModuleModel sim_properties, SwerveModule... modules) {
		this(new SwerveVisualization(getTranslations(modules)), config, sim_properties, modules);
	}
	public SwerveSimulator(SwerveVisualization viz, SimConfig config, SwerveModule... modules) {
		this(viz, config, null, modules);
	}
	public SwerveSimulator(SwerveVisualization viz, SimConfig config, SwerveModuleModel sim_properties, SwerveModule... modules) {
		this.modules = modules;
		this.visualization = viz;
		this.config = config;
		this.SIZE = modules.length;
		this.module_models = new SwerveModuleModel[this.SIZE];
		this.module_locs = new Vector2[this.SIZE];
		this.module_dirs = new Vector2[this.SIZE];
		this.n_inputs = NX.of(this.SIZE * 2);
		this.n_states = NX.of(this.SIZE * 4 + 6);
		this.u_inputs = new Matrix<>(this.n_inputs, Nat.N1());
		this.x_states = new Matrix<>(this.n_states, Nat.N1());
		this.y_outputs = this.x_states.copy();
		if(sim_properties == null) {
			this.applyModuleSpecificProperties();
		} else {
			this.applySimProperties(sim_properties);
		}
		for(int i = 0; i < this.SIZE; i++) {
			this.module_locs[i] = new Vector2(this.modules[i].module_location);
			this.module_dirs[i] = Vector2.unitVec(this.module_locs[i]);
		}
	}


	public void applySimProperties(SwerveModuleModel properties) {
		this.STATIC_MASS = this.config.ROBOT_MASS + properties.moduleMass() * this.SIZE;
		for(int i = 0; i < this.SIZE; i++) {
			this.module_models[i] = properties;
		}
	}
	public void applyModuleSpecificProperties() {
		this.STATIC_MASS = this.config.ROBOT_MASS;
		for(int i = 0; i < this.SIZE; i++) {
			final SwerveModuleModel props = this.modules[i].getSimProperties();
			if(props != null) {
				this.module_models[i] = props;
				this.STATIC_MASS += props.moduleMass();
			} else if(this.module_models[i] != null) {
				this.STATIC_MASS += this.module_models[i].moduleMass();
			}
		}
	}

	private static Translation3d[] getTranslations(SwerveModule... modules) {
		final Translation3d[] t = new Translation3d[modules.length];
		for(int i = 0; i < t.length; i++) {
			t[i] = new Translation3d(modules[i].module_location.getX(), modules[i].module_location.getY(), 0);
		}
		return t;
	}


	private synchronized void updateModuleSim(SwerveModule module, int index) {
		if(index < this.SIZE) {
			module.setSimulatedSteeringAngle(State.SteerAngle.fromN(this.y_outputs, index));
			module.setSimulatedSteeringRate(State.SteerRate.fromN(this.y_outputs, index));
			module.setSimulatedWheelPosition(State.DrivePosition.fromN(this.y_outputs, index));
			module.setSimulatedWheelVelocity(State.DriveVelocity.fromN(this.y_outputs, index));
		}
	}
	public synchronized void integrate(double dt_seconds) {
		for(int i = 0; i < this.SIZE; i++) {
			this.u_inputs.set(i * 2 + 0, 0, this.modules[i].getMotorAVolts());
			this.u_inputs.set(i * 2 + 1, 0, this.modules[i].getMotorBVolts());
		}
		// check that no ModuleSim's are null -- integration will be invalid if so
		this.x_states = NumericalIntegration.rk4(this::dynamics, this.x_states, this.u_inputs, dt_seconds);
		this.y_outputs = x_states.copy();
	}
	public synchronized void updateSimHW() {
		for(int i = 0; i < this.SIZE; i++) {
			this.updateModuleSim(this.modules[i], i);
		}
	}
	/** Integrate and update the states of the stored modules. If the states should not be updated (ex. not in sim mode), then call integrate() */
	public synchronized void update(double dt_seconds) {
		this.integrate(dt_seconds);
		this.updateSimHW();
	}


	protected Matrix<NX, N1> dynamics(Matrix<NX, N1> x, Matrix<NX, N1> u) {

		/** A note on 'physically quantitative' variable names:
		 * PREFIXES {r, l}:
		 * - r: "rotational"
		 * - l: "linear"
		 * SUFFIXES {x, v, a, P, I, F, Tq, volts, etc...}: (generally speaking, the unit dimensionality)
		 * - x: position		-- m & rad
		 * - v: velocity		-- m/s & rad/s
		 * - a: acceleration	-- m/s^2 & rad/s^2
		 * - P: momentum		-- Kgm/s & Kgm^2/s
		 * - I: inertia			-- Kg & Kgm^2
		 * - F: force			-- N
		 * - Tq: torque			-- Nm
		 * - volts: volts		-- V
		 * 
		 * ...anything after is a descriptor. */

		// STEP -0: Static final shortcut calculations
		final double F_norm_z = this.STATIC_MASS * 9.8;

		// STEP 0A: Allocate buffers for delta, sum applictant force/torque, headings, momentum
		final Matrix<NX, N1>
			x_prime = new Matrix<>(x.getStorage().createLike());
		final Vector2
			F_app = new Vector2();
		final double[]
			wheel_headings = new double[this.SIZE];
		double
			tq_app = 0.0,
			lI_momentum = this.config.ROBOT_MASS,
			rI_momentum = this.config.ROBOT_RI;

		// STEP 0B: Extract system states
		final Vector2
			lv_frame = new Vector2(
				State.FrameVelocityX.from(x),
				State.FrameVelocityY.from(x)	);		// transform by heading???
		final double
			av_frame = State.FrameAngularVel.from(x),
			ax_frame_vel = lv_frame.theta();			// transform by heading???

		// STEP 1: Module iteration #1
		for(int i = 0; i < this.SIZE; i++) {
			final double
			// STEP 1A(xN): Extract module states
				volts_a = u.get(i * 2, 0),
				volts_b = u.get(i * 2 + 1, 0),
				rx_steer = State.SteerAngle.fromN(x, i) % (Math.PI * 2),
				rv_steer = State.SteerRate.fromN(x, i),
				lv_wheel = State.DriveVelocity.fromN(x, i),
			// STEP 1B(xN): Initial module property calculations
				ra_steer = this.module_models[i].steerAAccel(volts_a, volts_b, rv_steer, lv_wheel, F_norm_z),
				F_wheel = this.module_models[i].wheelForceM(volts_a, volts_b, rv_steer, lv_wheel);
			// STEP 1C(xN): Set applicant output deltas
			State.SteerAngle.setN(x_prime, i, rv_steer);
			State.SteerRate.setN(x_prime, i, ra_steer);
			State.DrivePosition.setN(x_prime, i, lv_wheel);

			// STEP 1D(xN): Wheel force vector
			final Vector2
				F_wheel_2d = Vector2.fromPolar(F_wheel, rx_steer);
			// STEP 1E(xN): Sum the system's applicant force and torque, store headings
			F_app.append(F_wheel_2d);
			tq_app += this.module_locs[i].cross(F_wheel_2d);
			wheel_headings[i] = rx_steer;
			// STEP 1F(xN): Sum the system's linear and rotational momentum based on the direction of these velocities (includes wheel geartrain inertia)
			lI_momentum += this.module_models[i].effectiveLinearInertia(ax_frame_vel - rx_steer);
			rI_momentum += this.module_models[i].effectiveRotationalInertia(
				Vector2.cross(this.module_locs[i], av_frame).theta() - rx_steer, this.module_locs[i].norm());
		}
		// STEP 1G: Total system linear and rotational momentum
		final Vector2
			lP_sys = new Vector2(lv_frame).times(lI_momentum);
		final double
			rP_sys = av_frame * rI_momentum;

		// debug initial summations here

		final Vector2 f_frict = new Vector2();
		double tq_frict = 0.0;
		for(int i = 0; i < this.SIZE; i++) {
			final Vector2 f = Vector2.add(
				F_app,
				Vector2.invCross(this.module_locs[i], tq_app)
			).div((double)this.SIZE);
			final Vector2 p = Vector2.add(
				lP_sys,
				Vector2.invCross(this.module_locs[i], rP_sys)
			).div((double)this.SIZE);
			final Vector2 wdir = Vector2.fromPolar(1.0, wheel_headings[i]);
			final double
				a_volts = u.get(i * 2, 0),
				b_volts = u.get(i * 2 + 1, 0),
				s_omega = State.SteerRate.fromN(x, i),
				d_velocity = State.DriveVelocity.fromN(x, i),
				f_para = Vector2.dot(f, wdir),
				f_poip = Vector2.cross(f, wdir),
				p_para = Vector2.dot(p, wdir),
				p_poip = Vector2.cross(p, wdir),
				fr_inline = this.module_models[i].wheelGearFriction(f_para, p_para, a_volts, b_volts, s_omega, d_velocity, F_norm_z),
				fr_side = this.module_models[i].wheelSideFriction(f_poip, p_poip, F_norm_z);	// STEP 2A: collect side and geartrain friction for each module
			final Vector2 frict = new Vector2(fr_inline, fr_side).rotate(wheel_headings[i]);

			f_frict.append(frict);
			tq_frict += this.module_locs[i].cross(frict);	// STEP 2B: sum friction forces
		}
		// add sources and friction
		final Vector2 f_sys = Vector2.applyFriction(F_app, lP_sys, f_frict, 0.0);	// STEP 3A: sum source and friction forces/torque for the whole system
		final double
			tq_sys = FrictionModel.applyFriction(tq_app, rP_sys, tq_frict, 0.0),
			f_sys_theta = f_sys.theta();
		double sys_LI = this.config.ROBOT_MASS, sys_RI = this.config.ROBOT_RI;
		for(int i = 0; i < this.SIZE; i++) {
			sys_LI += this.module_models[i].effectiveLinearInertia(f_sys_theta - wheel_headings[i]);	// STEP 3B: collect module inertias based on summed force/torque vectors
			sys_RI += this.module_models[i].effectiveRotationalInertia(Vector2.cross(this.module_locs[i], tq_sys).theta() - wheel_headings[i], this.module_locs[i].norm());
		}
		final Vector2 frame_acc = new Vector2(f_sys).div(sys_LI);	// STEP 4: calc system accelerations
		final double frame_aacc = tq_sys / sys_RI;

		// transform based on heading???
		State.FrameVelocityX.set(x_prime, frame_acc.x());
		State.FrameVelocityY.set(x_prime, frame_acc.y());
		State.FrameAngularVel.set(x_prime, frame_aacc);
		State.FramePositionX.set(x_prime, lv_frame.x());
		State.FramePositionY.set(x_prime, lv_frame.y());
		State.FrameRotation.set(x_prime, av_frame);

		// use kinematics to set wheel accelerations

		return x_prime;

	}



	@Override
	public void initSendable(SendableBuilder b) {
		
	}


}
