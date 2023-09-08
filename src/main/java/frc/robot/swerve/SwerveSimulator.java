package frc.robot.swerve;

import java.util.ArrayList;
import java.util.function.BiFunction;

import edu.wpi.first.math.system.NumericalIntegration;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.numbers.*;
import edu.wpi.first.math.*;
import edu.wpi.first.util.sendable.*;

import frc.robot.swerve.SwerveUtils.*;
import frc.robot.swerve.simutil.*;
import frc.robot.team3407.Util;


/** SwerveSimulator applies the "high-level" physics computation and integration required to simulate a
 * a swerve base's movement as a result of motor voltages. The architecture allows for most of the module-specific
 * physical properties to be reimplemented and swapped using various interfaces. In this way, close to none of
 * the simulator's properties are locked in - the number of modules used, and module physical properties can all be
 * reimplemented. */
public class SwerveSimulator implements Sendable {

	/** A container for all the extra simulation parameters. */
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

	/** NX can be used for any {@link Num} generic type but it's actual value can be changed dynamically. Also provides easy creation of numbers >20 */
	private static final class NX extends Num implements Nat<NX> {

		private final int value;
		public NX(int v) { this.value = v; }

		@Override public int getNum() { return this.value; }

		static NX of(int v) { return new NX(v); }
		static NX of_x(int v, int scale) { return new NX(v * scale); }

	}

	/** An interface to match the dynamics sampler (with dt param). */
	public static interface DynamicsDT_F<States extends Num, Inputs extends Num> {
		public Matrix<States, N1> sample(Matrix<States, N1> x, Matrix<Inputs, N1> u, double dt);
	}
	/** An interface to match the DEBUG dynamics sampler (with dt param). */
	public static interface DynamicsDTd_F<States extends Num, Inputs extends Num> {
		public Matrix<States, N1> sample(Matrix<States, N1> x, Matrix<Inputs, N1> u, double dt, ArrayList<double[]> debug);
	}
	/** Generate a lambda function that wraps the dynamics with a specified dt param. */
	public static <States extends Num, Inputs extends Num>
		BiFunction< Matrix<States, N1>, Matrix<Inputs, N1>, Matrix<States, N1> >
			genWrapper(DynamicsDT_F<States, Inputs> f, double dt)
	{
		return (Matrix<States, N1> _x, Matrix<Inputs, N1> _u)->{ return f.sample(_x, _u, dt); };
	}
	/** Generate a lambda function that wraps the DEBUG dynamics with a specified dt param and attaches the debug array. */
	public static <States extends Num, Inputs extends Num>
		BiFunction< Matrix<States, N1>, Matrix<Inputs, N1>, Matrix<States, N1> >
			genDebugWrapper(DynamicsDTd_F<States, Inputs> f_d, double dt, ArrayList<double[]> debug)
	{
		return (Matrix<States, N1> _x, Matrix<Inputs, N1> _u)->{ return f_d.sample(_x, _u, dt, debug); };
	}

	/** All the states present in the states matrix along with helper methods for manipulating the matrix data. */
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
			} else {
				x_.set(module * 4 + this.idx, 0, val);
			}
		}
	}





	/** INSTANCE MEMBERS */

	/** Helper to convert an array of SwerveModules' internal translation(2d) to a 3d representation. */
	private static Translation3d[] getTranslations(SwerveModule... modules) {
		final Translation3d[] t = new Translation3d[modules.length];
		for(int i = 0; i < t.length; i++) {
			t[i] = new Translation3d(modules[i].module_location.getX(), modules[i].module_location.getY(), 0);
		}
		return t;
	}
	/** Helper to update a module based on the stored Matrix buffer. */
	private synchronized void updateModuleSim(SwerveModule module, int index) {
		if(index < this.SIZE) {
			module.setSimulatedSteeringAngle(State.SteerAngle.fromN(this.y_outputs, index));
			module.setSimulatedSteeringRate(State.SteerRate.fromN(this.y_outputs, index));
			module.setSimulatedWheelPosition(State.DrivePosition.fromN(this.y_outputs, index));
			module.setSimulatedWheelVelocity(State.DriveVelocity.fromN(this.y_outputs, index));
		}
	}
	/** Helper to emplace all the wheel rotations into a single array. */
	private synchronized double[] getWheelRotations() {
		final double[] rotations = new double[this.SIZE];
		for(int i = 0; i < this.SIZE; i++) {
			rotations[i] = State.SteerAngle.fromN(this.y_outputs, i);
		}
		return rotations;
	}


	/** MAIN INTERFACE */

	private final SwerveModule[] modules;
	private final SwerveModuleModel[] module_models;
	private final SimConfig config;
	private final SwerveVisualization visualization;
	private final Vector2[] module_locs;
	private final int SIZE;
	private final NX N_INPUTS, N_STATES;
	private double STATIC_MASS;

	private final Matrix<NX, N1> u_inputs;
	private Matrix<NX, N1> x_states, y_outputs;
	private ArrayList<double[]> debug_buff = new ArrayList<>();
	private double[] last_debug = new double[29];


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
		this.N_INPUTS = NX.of(this.SIZE * 2);
		this.N_STATES = NX.of(this.SIZE * 4 + 6);
		this.u_inputs = new Matrix<>(this.N_INPUTS, Nat.N1());
		this.x_states = new Matrix<>(this.N_STATES, Nat.N1());
		this.y_outputs = this.x_states.copy();
		if(sim_properties == null) {
			this.applyModuleSpecificProperties();
		} else {
			this.applySimProperties(sim_properties);
		}
		for(int i = 0; i < this.SIZE; i++) {
			this.module_locs[i] = new Vector2(this.modules[i].module_location);
		}
	}


	/** Set all modules' properties to that specified. */
	public void applySimProperties(SwerveModuleModel properties) {
		this.STATIC_MASS = this.config.ROBOT_MASS + properties.moduleMass() * this.SIZE;
		for(int i = 0; i < this.SIZE; i++) {
			this.module_models[i] = properties;
		}
	}
	/** Set each module's properties based on the getter that is apart of each SwerveModule implementation. */
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


	/** Run a single iteration of numerical integration on the simulation dynamics. */
	public synchronized void integrate(double dt_seconds) {
		for(int i = 0; i < this.SIZE; i++) {
			this.u_inputs.set(i * 2 + 0, 0, this.modules[i].getMotorAVolts());
			this.u_inputs.set(i * 2 + 1, 0, this.modules[i].getMotorBVolts());
		}
		// check that no ModuleSim's are null -- integration will be invalid if so
		this.x_states = NumericalIntegration.rk4(
			genWrapper(this::dynamics, dt_seconds), this.x_states, this.u_inputs, dt_seconds);
		this.y_outputs = x_states.copy();
	}
	/** Run a single iteration of numerical integration on the simulation dynamics -- with debugging output. */
	public synchronized void integrate_D(double dt_seconds) {
		for(int i = 0; i < this.SIZE; i++) {
			this.u_inputs.set(i * 2 + 0, 0, this.modules[i].getMotorAVolts());
			this.u_inputs.set(i * 2 + 1, 0, this.modules[i].getMotorBVolts());
		}
		this.x_states = NumericalIntegration.rk4(
			genDebugWrapper(this::dynamics, dt_seconds, this.debug_buff), this.x_states, this.u_inputs, dt_seconds);
		this.y_outputs = x_states.copy();
		// this.last_debug = this.debug_buff.get(this.debug_buff.size() - 1);
		this.last_debug = this.debug_buff.get(0);
		this.debug_buff.clear();
	}
	/** Update each module's feedback data. */
	public synchronized void updateSimHW() {
		for(int i = 0; i < this.SIZE; i++) {
			this.updateModuleSim(this.modules[i], i);
		}
	}
	/** Integrate and update the states of the stored modules. If the states should not be updated (ex. not in sim mode), then only call integrate() */
	public synchronized void update(double dt_seconds) {
		this.integrate(dt_seconds);
		this.updateSimHW();
	}

	protected Matrix<NX, N1> dynamics(Matrix<NX, N1> x, Matrix<NX, N1> u, double dt_seconds)
		{ return this.dynamics(x, u, dt_seconds, null); }
	/** The simulation dynamics. Given a state, input, and timestep, compute the change in state. */
	protected Matrix<NX, N1> dynamics(Matrix<NX, N1> x, Matrix<NX, N1> u, double dt_seconds, ArrayList<double[]> debug) {

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

		// STEP -0: Static shortcut calculations
		final double
			F_norm_z = this.STATIC_MASS * 9.8,
			PI2 = (Math.PI * 2.0);

		// STEP 0A: Allocate buffers for delta, system applictant force/torque, friction, headings, system momentum
		final Matrix<NX, N1>
			x_prime = new Matrix<>(x.getStorage().createLike());
		final Vector2
			F_app = new Vector2(),
			F_frict = new Vector2();
		final double[]
			wheel_headings = new double[this.SIZE],
			wheel_velocities = new double[this.SIZE];
		double
			Tq_app = 0.0,
			Tq_frict = 0.0,
			lI_momentum = this.config.ROBOT_MASS,
			rI_momentum = this.config.ROBOT_RI;

		// STEP 0B: Extract system states
		final double
			rx_frame = State.FrameRotation.from(x) % PI2,	// theta offset from the frame coordinate system
			rv_frame = State.FrameAngularVel.from(x);		// the frame's rotation rate -- abstract coordinate space?
		final Vector2
			lv_field = new Vector2(					// the frame's velocity in the field coordinate system
				State.FrameVelocityX.from(x),
				State.FrameVelocityY.from(x) ),
			lv_frame = lv_field.rotate(-rx_frame);	// the frame's velocity in it's own coordinate system
		final double
			rx_frame_lv = lv_frame.theta();			// the angle theta of the frame's velocity vector in it's own coordinate system

		// STEP 1: Module iteration #1
		for(int i = 0; i < this.SIZE; i++) {
			final double
			// STEP 1A(xN): Extract module states
				volts_a = u.get(i * 2, 0),
				volts_b = u.get(i * 2 + 1, 0),
				rx_steer = State.SteerAngle.fromN(x, i) % PI2,		// the steer angle in the frames's coordinate system
				rv_steer = State.SteerRate.fromN(x, i),				// the steer rate from the frame's reference
				// lv_wheel = State.DriveVelocity.fromN(x, i),			// the linear velocity of the module from the module's reference
				lv_wheel = Vector2.add( lv_frame, Vector2.cross( rv_frame, this.module_locs[i] ) ).rotate(-rx_steer).x(),	// ^ compute based on frame velocity for better results
			// STEP 1B(xN): Initial module property calculations
				ra_steer = this.module_models[i].steerAAccel( volts_a, volts_b, rv_steer, lv_wheel, F_norm_z, dt_seconds ),
				F_wheel = this.module_models[i].wheelForceM( volts_a, volts_b, rv_steer, lv_wheel, dt_seconds );
			// STEP 1C(xN): Set applicant output deltas
			State.SteerAngle.setN(x_prime, i, rv_steer);
			State.SteerRate.setN(x_prime, i, ra_steer);
			State.DrivePosition.setN(x_prime, i, lv_wheel);

			// STEP 1D(xN): Wheel force vector
			final Vector2
				F_wheel_2d = Vector2.fromPolar(F_wheel, rx_steer);			// the wheel force vector in the frame's coordinate system
			// STEP 1E(xN): Sum the system's applicant force and torque, store headings
			F_app.append(F_wheel_2d);
			Tq_app += this.module_locs[i].cross(F_wheel_2d);				// the cross product takes place in the frame's coordinate system
			wheel_headings[i] = rx_steer;
			wheel_velocities[i] = lv_wheel;
			// STEP 1F(xN): Sum the system's linear and rotational momentum based on the direction of these velocities (includes wheel geartrain inertia)
			lI_momentum += this.module_models[i].effectiveLinearInertia( (rx_frame_lv - rx_steer) );	// <-- the difference occurs in the frame coordinate system
			rI_momentum += this.module_models[i].effectiveRotationalInertia(
				(Vector2.cross( rv_frame, this.module_locs[i] ).theta() - rx_steer), this.module_locs[i].norm() );	// <-- also in frame coordinate system because module_locs[] is frame local while omega is abstract
		}
		// STEP 1G: Total system linear and rotational momentum
		final Vector2
			lP_sys = new Vector2(lv_frame).times(lI_momentum);	// the momentum in frame coordinate space
		final double
			rP_sys = rv_frame * rI_momentum;					// the rotational momenum is abstract?

		// debug initial summations here

		// STEP 2: Calculate friction (module iteration #2)
		for(int i = 0; i < this.SIZE; i++) {
			final Vector2
			// STEP 2A(xN): Calc the force component acting at the module's CG
				Fn = Vector2
					.add( F_app, Vector2.invCross( Tq_app / this.SIZE, this.module_locs[i] ) ),		// addition and cross product take place in frame coord space
				lvN = Vector2
					.add( lv_frame, Vector2.cross( rv_frame, this.module_locs[i] ) ),	// part of this is already calculated in the first iteration :\
				// Pn = Vector2
				// 	.add( lP_sys, Vector2.invCross(this.module_locs[i], rP_sys / this.SIZE) ),
				wheel_heading = Vector2.fromPolar( 1.0, wheel_headings[i] );		// unit vector in the wheel fwd direction -- in field coord space
			final double
			// STEP 2B(xN): Split Fn vector into components that are aligned with the wheel's heading
				F_para = Vector2.dot( wheel_heading, Fn ),
				F_poip = Vector2.cross( wheel_heading, Fn ),
				lv_poip = Vector2.cross( wheel_heading, lvN ),
				// P_para = Vector2.dot(Pn, wheel_heading),
				// P_poip = Vector2.cross(Pn, wheel_heading),
				volts_a = u.get(i * 2, 0),
				volts_b = u.get(i * 2 + 1, 0),
				rv_steer = State.SteerRate.fromN(x, i),
				lv_wheel = wheel_velocities[i],
			// STEP 2C(xN): Calc maximum friction force in each direction via module properties
				F_inline_frict = this.module_models[i]
					.wheelGearFriction( F_norm_z, F_para, volts_a, volts_b, rv_steer, lv_wheel, dt_seconds ),
				F_side_frict = this.module_models[i]
					.wheelSideFriction( F_norm_z, F_poip, lv_poip, dt_seconds );
			final Vector2
			// STEP 2D(xN): Combine friction components into a single vector
				F_frict_n = new Vector2( F_inline_frict, F_side_frict ).rotate( wheel_headings[i] );
			// STEP 2E(xN): Append to total friction and torque vectors
			F_frict.append(F_frict_n);
			Tq_frict += this.module_locs[i].cross(F_frict_n);
		}

		// STEP 3: Sum the applicant and friction force/torque such that the momentum does not change direction because of friction
		final Vector2
			F_sys = Vector2.applyFriction( F_app, lP_sys, F_frict, dt_seconds );	// the net linear force vector in frame coord space
		final double	// ^ need to change this. friction should only be applied in 1D at the direction of net force, all side components don't mean
			Tq_sys = FrictionModel.applyFriction( Tq_app, rP_sys, Tq_frict, dt_seconds ),
			ax_f_sys = F_sys.theta();		// the angle of net linear force in frame coord space
		// STEP 4: Sum the system inertias based on the direction of net force/torque
		double
			lI_sys = this.config.ROBOT_MASS,
			rI_sys = this.config.ROBOT_RI;
		for(int i = 0; i < this.SIZE; i++) {
			lI_sys += this.module_models[i].effectiveLinearInertia( (ax_f_sys - wheel_headings[i]) );	// OK -- both in frame coord system
			rI_sys += this.module_models[i].effectiveRotationalInertia(
				(Vector2.cross( Tq_sys, this.module_locs[i] ).theta() - wheel_headings[i]), this.module_locs[i].norm() );
		}
		// STEP 5: System linear and rotational acceleration
		final Vector2
			la_sys = new Vector2(F_sys).div(lI_sys);	// the net linear acceleration vector in frame coordinate space
		final double
			ra_sys = Tq_sys / rI_sys;					// the net angular acceleration (abstract reference)

		// STEP 6: Fill x_prime
		for(int i = 0; i < this.SIZE; i++) {	// alternatively, update wheel position based on integrated frame position -- take a delta and work backwards from that...
			// STEP 6A(xN): Find the acceleration of each module in the direction of the wheel, update velocity delta
			final Vector2
				laN = Vector2	// the linear acceleration in frame coord space
					.add( la_sys, Vector2.cross( ra_sys, this.module_locs[i] ) )	// the component of acceleration from adding linear and angular static -- frame coord sys operations
					.sub( Vector2.mult( this.module_locs[i], (rv_frame * rv_frame) ) );		// the component from centripetal due to angular velocity -- frame coord sys because normalized to module_locs[]
			State.DriveVelocity.setN( x_prime, i, laN.rotate(-wheel_headings[i]).x() );		// <-- rotate the vector to be in wheel/module reference and take the x(fwd) component as the wheel's acceleration
		}
		// STEP 6B: Convert linear acceleration from frame reference back to field reference because we want to keep track of the robot relative to the field, not relative to itself
		final Vector2
			la_field = la_sys.rotate(rx_frame);		// convert back to field coordinate space
		// STEP 6C: Set fields
		State.FrameVelocityX.set(x_prime, la_field.x());	// delta velocity in field space
		State.FrameVelocityY.set(x_prime, la_field.y());
		State.FrameAngularVel.set(x_prime, ra_sys);			// delta rotation rate (abstract ref)
		State.FramePositionX.set(x_prime, lv_field.x());	// delta position in field space -- not modified from previous state
		State.FramePositionY.set(x_prime, lv_field.y());
		State.FrameRotation.set(x_prime, rv_frame);			// delta rotation (absolute ref)

		// STEP 7: Load debug array
		if(debug != null) {
			final double[]
				vals = new double[] {
					F_app.x(), F_app.y(),
					F_frict.x(), F_frict.y(),
					Tq_app,
					Tq_frict,
					lI_momentum,
					rI_momentum,
					rx_frame,
					rv_frame,
					lv_field.x(), lv_field.y(),
					lv_frame.x(), lv_frame.y(),
					rx_frame_lv,
					lP_sys.x(), lP_sys.y(),
					rP_sys,
					F_sys.x(), F_sys.y(),
					Tq_sys,
					ax_f_sys,
					lI_sys,
					rI_sys,
					la_sys.x(), la_sys.y(),
					ra_sys,
					la_field.x(), la_field.y()	// 29 fields
				};
			debug.add(vals);
		}

		return x_prime;

	}



	@Override
	public void initSendable(SendableBuilder b) {
		b.addDoubleArrayProperty("State Data", ()->this.y_outputs.getData(), null);
		b.addDoubleArrayProperty("Robot Pose",
			()->new double[]{
				State.FramePositionX.from(this.y_outputs),
				State.FramePositionY.from(this.y_outputs),
				State.FrameRotation.from(this.y_outputs)
			}, null);
		// b.addDoubleArrayProperty("Robot Pose",
		// 	()->Util.toComponents2d( new Pose2d() ), null);
		b.addDoubleArrayProperty("Wheel Poses",
			()->Util.toComponents3d( this.visualization.getWheelPoses3d( this.getWheelRotations() ) ), null);

		b.addDoubleProperty("Applied Torque",				()->this.last_debug[4], null);
		b.addDoubleProperty("Frictional Torque",			()->this.last_debug[5], null);
		b.addDoubleProperty("Linear Inertia (P-pass)",		()->this.last_debug[6], null);
		b.addDoubleProperty("Rotational Inertia (P-pass)",	()->this.last_debug[7], null);
		b.addDoubleProperty("Frame Ref Angle",				()->this.last_debug[8], null);
		b.addDoubleProperty("Frame Angular Vel",			()->this.last_debug[9], null);
		b.addDoubleProperty("Frame Velocity Direction",	()->this.last_debug[14], null);
		b.addDoubleProperty("Angular Momentum",			()->this.last_debug[17], null);
		b.addDoubleProperty("Net Torque",					()->this.last_debug[20], null);
		b.addDoubleProperty("Net Force Direction",			()->this.last_debug[21], null);
		b.addDoubleProperty("Linear Inertia (F-pass)",		()->this.last_debug[22], null);
		b.addDoubleProperty("Rotational Inertia (F-pass)",	()->this.last_debug[23], null);
		b.addDoubleProperty("Rotational Acceleration",		()->this.last_debug[26], null);
		b.addDoubleArrayProperty("Net Applied Force",		()->new double[]{ this.last_debug[0], this.last_debug[1] }, null);
		b.addDoubleArrayProperty("Net Friction",			()->new double[]{ this.last_debug[2], this.last_debug[3] }, null);
		b.addDoubleArrayProperty("Field Velocity",			()->new double[]{ this.last_debug[10], this.last_debug[11] }, null);
		b.addDoubleArrayProperty("Frame Velocity",			()->new double[]{ this.last_debug[12], this.last_debug[13] }, null);
		b.addDoubleArrayProperty("Linear Momentum",		()->new double[]{ this.last_debug[15], this.last_debug[16] }, null);
		b.addDoubleArrayProperty("Net Summed Force",		()->new double[]{ this.last_debug[18], this.last_debug[19] }, null);
		b.addDoubleArrayProperty("Net Frame Acceleration",	()->new double[]{ this.last_debug[24], this.last_debug[25] }, null);
		b.addDoubleArrayProperty("Net Field Acceleration",	()->new double[]{ this.last_debug[27], this.last_debug[28] }, null);

		b.addIntegerProperty("Debug Buffer Size", ()->this.debug_buff.size(), null);
	}

	@Override
	public String toString() {
		final Matrix<NX, N1> vec = this.y_outputs;
		String s = String.format(
			"Swerve Simulation State -->\n" +
			"Frame Position: <%.2f, %.2f>m\n" +
			"Frame Velocity: <%.2f, %.2f>m\n" +
			"Frame Rotations: %.2f rad\n" +
			"Frame Turn Rate: %.2f rad/s\n",
			State.FramePositionX.from(vec),
			State.FramePositionY.from(vec),
			State.FrameVelocityX.from(vec),
			State.FrameVelocityY.from(vec),
			State.FrameRotation.from(vec),
			State.FrameAngularVel.from(vec)
		);
		for(int i = 0; i < this.SIZE; i++) {
			s += String.format(
				"Module #[%d] -->\n" +
				"\tSteer Angle: %.2f rad\n" +
				"\tSteer Rate: %.2f rad/s\n" +
				"\tWheel Displacement: %.2f m\n" +
				"\tWheel Velocity: %.2f m/s\n",
				i,
				State.SteerAngle.fromN(vec, i),
				State.SteerRate.fromN(vec, i),
				State.DrivePosition.fromN(vec, i),
				State.DriveVelocity.fromN(vec, i)
			);
		}
		return s;
	}


}
