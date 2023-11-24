package frc.robot.swerve;

import java.util.function.BiFunction;
import java.util.Iterator;

import edu.wpi.first.math.system.NumericalIntegration;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.numbers.*;
import edu.wpi.first.math.*;
import edu.wpi.first.util.sendable.*;

import frc.robot.swerve.SwerveUtils.*;
import frc.robot.swerve.simutil.*;
import frc.robot.team3407.*;
import frc.robot.team3407.SenderNT.RecursiveSendable;


/** SwerveSimulator applies the "high-level" physics computation and integration required to simulate a
 * a swerve base's movement as a result of motor voltages. The architecture allows for most of the module-specific
 * physical properties to be reimplemented and swapped using various interfaces. In this way, close to none of
 * the simulator's properties are locked in - the number of modules used, and module physical properties can all be
 * reimplemented. */
public class SwerveSimulator implements RecursiveSendable {

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
		public Matrix<States, N1> sample(Matrix<States, N1> x, Matrix<Inputs, N1> u, double dt, DynamicsBuffer buff);
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
			genDebugWrapper(DynamicsDTd_F<States, Inputs> f_d, double dt, ArrayIterator<DynamicsBuffer> dbarr)
	{
		return (Matrix<States, N1> _x, Matrix<Inputs, N1> _u)->{ return f_d.sample(_x, _u, dt, dbarr.next()); };
	}

	/** All the states present in the states matrix along with helper methods for manipulating the matrix data. */
	protected enum State {
		FramePositionX		(-6),
		FramePositionY		(-5),
		FrameVelocityX		(-4),
		FrameVelocityY		(-3),
		FramePositionA		(-2),	// 'angular'
		FrameVelocityA		(-1),	// 'angular'

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
	private final DynamicsBuffer[] buffers = new DynamicsBuffer[4];		// 4 --> # of samples to dynamics within RK4 integration
	private final int SIZE;
	private final NX N_INPUTS, N_STATES;
	private final Matrix<NX, N1> u_inputs;
	private Matrix<NX, N1> x_states, y_outputs;
	private DynamicsBuffer non_debug_buff;
	private double STATIC_MASS;


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
		for(int i = 0; i < this.buffers.length; i++) {
			this.buffers[i] = new DynamicsBuffer(this.SIZE);
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
			this.u_inputs.set(i * 2 + 0, 0, Util.clamp(this.modules[i].getMotorAVolts(), -12, 12));
			this.u_inputs.set(i * 2 + 1, 0, Util.clamp(this.modules[i].getMotorBVolts(), -12, 12));
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
		final ArrayIterator<DynamicsBuffer> dbarr = new ArrayIterator<>(this.buffers);
		this.x_states = NumericalIntegration.rk4(
			genDebugWrapper(this::dynamics, dt_seconds, dbarr), this.x_states, this.u_inputs, dt_seconds);
		this.y_outputs = x_states.copy();
	}
	/** Synchronize each module's hardware simulation properties with the internal stored state. */
	public synchronized void applyStates() {
		for(int i = 0; i < this.SIZE; i++) {
			this.modules[i].setSimulatedSteeringAngle(	State.SteerAngle	.fromN(this.y_outputs, i) );
			this.modules[i].setSimulatedSteeringRate(	State.SteerRate		.fromN(this.y_outputs, i) );
			this.modules[i].setSimulatedWheelPosition(	State.DrivePosition	.fromN(this.y_outputs, i) );
			this.modules[i].setSimulatedWheelVelocity(	State.DriveVelocity	.fromN(this.y_outputs, i) );
		}
	}
	/** Integrate and update the states of the stored modules. If the states should not be updated (ex. not in sim mode), then only call integrate() */
	public synchronized void update(double dt_seconds) {
		this.integrate(dt_seconds);
		this.applyStates();
	}

	/** Simulation dynamics w/o an attached debug reference. */
	protected Matrix<NX, N1> dynamics(Matrix<NX, N1> x, Matrix<NX, N1> u, double dt_seconds) {
		return this.dynamics(x, u, dt_seconds, null);
	}
	/** The simulation dynamics. Given a state, input, and timestep, compute the change in state. */
	protected Matrix<NX, N1> dynamics(Matrix<NX, N1> x, Matrix<NX, N1> u, double dt_seconds, DynamicsBuffer buffer) {

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

		// STEP 0A: Allocate buffers for delta, reset summations
		if(buffer == null) {
			if(this.non_debug_buff == null) {
				this.non_debug_buff = new DynamicsBuffer(this.SIZE);	// use the last buffer since the last call will be the last overwrite
			}
			buffer = this.non_debug_buff;
		}
		final DynamicsBuffer db = buffer;
		final Matrix<NX, N1>
			x_prime = new Matrix<>(x.getStorage().createLike());

		db.F_app.clear();
		db.F_frict.clear();
		db.Tq_app = 0.0;
		db.Tq_frict = 0.0;
		db.lI_momentum = this.config.ROBOT_MASS;
		db.rI_momentum = this.config.ROBOT_RI;

		// STEP 0B: Extract system states
		db.rx_frame = State.FramePositionA.from(x) % PI2;	// theta offset from the frame coordinate system
		db.rv_frame = State.FrameVelocityA.from(x);			// the frame's rotation rate -- abstract coordinate space?
		db.lv_field.set(									// the frame's velocity in the field coordinate system
			State.FrameVelocityX.from(x),
			State.FrameVelocityY.from(x) );
		db.lv_frame.set(db.lv_field)			// the frame's velocity in it's own coordinate system
			.rotate(-db.rx_frame);
		db.rx_frame_lv = db.lv_frame.theta();	// the angle theta of the frame's velocity vector in it's own coordinate system

		// STEP 1: Module iteration #1
		for(int i = 0; i < this.SIZE; i++) {
			final DynamicsBuffer.ModuleBuffer mb = db.modn(i);
			final Vector2 lv_tangent = Vector2.cross( db.rv_frame, this.module_locs[i] );	// the tangential velocity from angular velocity for the module
		// STEP 1A(xN): Extract module states
			mb.volts_a = u.get(i * 2, 0);
			mb.volts_b = u.get(i * 2 + 1, 0);
			mb.rx_steer = State.SteerAngle.fromN(x, i) % PI2;		// the steer angle in the frames's coordinate system
			mb.rv_steer = State.SteerRate.fromN(x, i);				// the steer rate from the frame's reference
			mb.lv_ext
				.set(db.lv_frame)
				.append(lv_tangent);								// the linear velocity of the module in the frame's reference
			// mb.lv_wheel = State.DriveVelocity.fromN(x, i),		// the (wheel) linear velocity of the module from the module's reference
			mb.lv_wheel = Vector2
				.angleComponent( mb.lv_ext, mb.rx_steer );	// compute based on frame velocity for better results compared to ^
		// STEP 1B(xN): Initial module property calculations
			mb.ra_steer = this.module_models[i]
				.steerAAccel( mb.volts_a, mb.volts_b, mb.rv_steer, mb.lv_wheel, F_norm_z, dt_seconds );
			mb.F_src_mag = this.module_models[i]
				.wheelForceM( mb.volts_a, mb.volts_b, mb.rv_steer, mb.lv_wheel, dt_seconds );
		// STEP 1C(xN): Set applicant output deltas
			State.SteerAngle.setN(x_prime, i, mb.rv_steer);
			State.SteerRate.setN(x_prime, i, mb.ra_steer);
			State.DrivePosition.setN(x_prime, i, mb.lv_wheel);

		// STEP 1D(xN): Wheel force vector
			mb.F_src_vec = Vector2
				.fromPolar(mb.F_src_mag, mb.rx_steer);		// the wheel force vector in the frame's coordinate system
		// STEP 1E(xN): Sum the system's applicant force and torque, store headings
			db.F_app.append(mb.F_src_vec);
			db.Tq_app += this.module_locs[i]
				.cross(mb.F_src_vec);			// the cross product takes place in the frame's coordinate system
		// STEP 1F(xN): Sum the system's linear and rotational momentum based on the direction of these velocities (includes wheel geartrain inertia)
			db.lI_momentum += this.module_models[i]
				.effectiveLinearInertia( (db.rx_frame_lv - mb.rx_steer) );	// <-- the difference occurs in the frame coordinate system
			db.rI_momentum += this.module_models[i]
				.effectiveRotationalInertia(
					lv_tangent.theta() - mb.rx_steer,
					this.module_locs[i].norm() );	// <-- also in frame coordinate system because module_locs[] is frame local while omega is abstract
		}
		// STEP 1G: Total system linear and rotational momentum
		db.lP_sys
			.set(db.lv_frame)
			.times(db.lI_momentum);		// the momentum in frame coordinate space
		db.rP_sys = db.rv_frame * db.rI_momentum;				// the rotational momenum is abstract?

		// STEP 2: Calculate friction (module iteration #2)
		for(int i = 0; i < this.SIZE; i++) {
			final DynamicsBuffer.ModuleBuffer mb = db.modn(i);
			final Vector2 wheel_vec = Vector2.fromPolar( 1.0, mb.rx_steer );	// unit vector in the wheel fwd direction -- in field coord space
		// STEP 2A(xN): Calc the force component acting at the module's CG
			mb.F_ext
				.set(db.F_app)
				.append( Vector2
					.invCross( db.Tq_app / this.SIZE, this.module_locs[i] ) );	// addition and cross product take place in frame coord space
			// >> previously set lv_ext here --> cached in first loop
		// STEP 2B(xN): Split Fn vector into components that are aligned with the wheel's heading
			mb.F_ext_para = Vector2.dot( wheel_vec, mb.F_ext );
			mb.F_ext_poip = Vector2.cross( wheel_vec, mb.F_ext );
			mb.lv_side = Vector2.cross( wheel_vec, mb.lv_ext );
		// STEP 2C(xN): Calc maximum friction force in each direction via module properties
			mb.F_frict_para = this.module_models[i]
				.wheelGearFriction( F_norm_z, mb.F_ext_para, mb.volts_a, mb.volts_b, mb.rv_steer, mb.lv_wheel, dt_seconds );
			mb.F_frict_poip = this.module_models[i]
				.wheelSideFriction( F_norm_z, mb.F_ext_poip, mb.lv_side, dt_seconds );
		// STEP 2D(xN): Combine friction components into a single vector
			mb.F_frict_sum
				.set( mb.F_frict_para, mb.F_frict_poip )
				.rotate( mb.rx_steer );
		// STEP 2E(xN): Append to total friction and torque vectors
			db.F_frict.append(mb.F_frict_sum);
			db.Tq_frict += this.module_locs[i]
				.cross(mb.F_frict_sum);
		}

		// STEP 3: Sum the applicant and friction force/torque such that the momentum does not change direction because of friction
		db.F_sys.set( Vector2
			.applyFriction( db.F_app, db.lP_sys, db.F_frict, dt_seconds ) );	// the net linear force vector in frame coord space
		db.Tq_sys = FrictionModel
			.applyFriction( db.Tq_app, db.rP_sys, db.Tq_frict, dt_seconds );
		db.rx_f_sys = db.F_sys.theta();		// the angle of net linear force in frame coord space

		// STEP 4: Sum the system inertias based on the direction of net force/torque
		db.lI_sys = this.config.ROBOT_MASS;
		db.rI_sys = this.config.ROBOT_RI;
		for(int i = 0; i < this.SIZE; i++) {
			final DynamicsBuffer.ModuleBuffer mb = db.modn(i);
			db.lI_sys += this.module_models[i]
				.effectiveLinearInertia( (db.rx_f_sys - mb.rx_steer) );	// OK -- both in frame coord system
			db.rI_sys += this.module_models[i]
				.effectiveRotationalInertia(
					(Vector2
						.cross( db.Tq_sys, this.module_locs[i] )
						.theta() - mb.rx_steer),
					this.module_locs[i].norm() );	// add these as items in the buffer so we can see individual module inertias
		}

		// STEP 5: System linear and rotational acceleration
		db.la_sys
			.set(db.F_sys)
			.div(db.lI_sys);		// the net linear acceleration vector in frame coordinate space
		db.ra_sys = db.Tq_sys / db.rI_sys;			// the net angular acceleration (abstract reference)

		// STEP 6: Fill x_prime
		for(int i = 0; i < this.SIZE; i++) {	// alternatively, update wheel position based on integrated frame position -- take a delta and work backwards from that...
			final DynamicsBuffer.ModuleBuffer mb = db.modn(i);
		// STEP 6A(xN): Find the acceleration of each module in the direction of the wheel, update velocity delta
			mb.la_ext.set(db.la_sys)	// the linear acceleration in frame coord space
				.append( Vector2.cross( db.ra_sys, this.module_locs[i] ) )		// the component of acceleration from adding linear and angular static -- frame coord sys operations
				.sub( Vector2.mult( this.module_locs[i], (db.rv_frame * db.rv_frame) ) );			// the component from centripetal due to angular velocity -- frame coord sys because normalized to module_locs[]
			State.DriveVelocity.setN( x_prime, i, Vector2.angleComponent( mb.la_ext, mb.rx_steer ) );	// <-- rotate the vector to be in wheel/module reference and take the x(fwd) component as the wheel's acceleration
		}
		// STEP 6B: Convert linear acceleration from frame reference back to field reference because we want to keep track of the robot relative to the field, not relative to itself
		db.la_field = db.la_sys.rotate(db.rx_frame);		// convert back to field coordinate space
		// STEP 6C: Set fields
		State.FrameVelocityX.set(x_prime, db.la_field.x());		// delta velocity in field space
		State.FrameVelocityY.set(x_prime, db.la_field.y());
		State.FrameVelocityA.set(x_prime, db.ra_sys);			// delta rotation rate (abstract ref)
		State.FramePositionX.set(x_prime, db.lv_field.x());		// delta position in field space -- not modified from previous state
		State.FramePositionY.set(x_prime, db.lv_field.y());
		State.FramePositionA.set(x_prime, db.rv_frame);			// delta rotation (absolute ref)

		return x_prime;

	}



	@Override
	public void initSendable(SendableBuilder b) {
		b.addDoubleArrayProperty("State Data", ()->this.y_outputs.getData(), null);
		b.addDoubleArrayProperty("Robot Pose",
			()->new double[]{
				State.FramePositionX.from(this.y_outputs),
				State.FramePositionY.from(this.y_outputs),
				State.FramePositionA.from(this.y_outputs)
			}, null);
		b.addDoubleArrayProperty("Wheel Poses",
			()->Util.toComponents3d( this.visualization.getWheelPoses3d( this.getWheelRotations() ) ), null);
	}
	@Override
	public void initRecursive(SenderNT s, String key) {
		s.putData(key, (Sendable)this);
		for(int i = 0; i < this.buffers.length; i++) {
			s.putData(key + "/Buffer [" + i + "]", this.buffers[i]);
		}
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
			State.FramePositionA.from(vec),
			State.FrameVelocityA.from(vec)
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





	/** DynamicsBuffer contains all the temporary states used within a dynamics iteration. */
	private static class DynamicsBuffer implements RecursiveSendable {

		/** ModuleBuffer contains all the temporary for a single module within a dynamics iteration. */
		public static class ModuleBuffer implements Sendable {

			public Vector2
				F_src_vec	= new Vector2(),	// the force from the wheel in vector form
				F_ext		= new Vector2(),	// the redistributed average force on the module
				F_frict_sum	= new Vector2(),
				lv_ext		= new Vector2(),	// the externally calculated module velocity
				la_ext		= new Vector2();	// the externally calculated module acceleration
			public double
				volts_a,		// input volts a
				volts_b,		// input volts b
				rx_steer,		// steer angle
				rv_steer,		// steer angular velocity
				ra_steer,		// steer angular acceleration
				lv_wheel,		// linear velocity of the wheel along the floor
				lv_side,		// externally calculated side sliding velocity (perpendicular to the wheel's heading)
				F_src_mag,		// the magnitude of the force from the wheel
				F_ext_para,		// the component of redistributed average force on the module in the direction of the wheel
				F_ext_poip,		// the component of redistributed average force on the module in the direction perpendiculat to the wheel
				F_frict_para,	// the friction from the wheel and geartrain
				F_frict_poip;	// the friction from sliding sideways

			@Override
			public void initSendable(SendableBuilder b) {
				b.addDoubleArrayProperty("Wheel Force 2D",				this.F_src_vec::getData, null);
				b.addDoubleArrayProperty("Redistributed Force 2D",		this.F_ext::getData, null);
				b.addDoubleArrayProperty("Net Friction Force 2D",		this.F_frict_sum::getData, null);
				b.addDoubleArrayProperty("Module Velocity 2D",			this.lv_ext::getData, null);
				b.addDoubleArrayProperty("Module Acceleration 2D",		this.la_ext::getData, null);
				b.addDoubleProperty("Input Volts A",				()->this.volts_a, null);
				b.addDoubleProperty("Input Volts B",				()->this.volts_b, null);
				b.addDoubleProperty("Steer Angle (frame)",			()->this.rx_steer, null);
				b.addDoubleProperty("Steer Angular Velocity",		()->this.rv_steer, null);
				b.addDoubleProperty("Steer Angular Acceleration",	()->this.ra_steer, null);
				b.addDoubleProperty("Wheel Inline Velocity",		()->this.lv_wheel, null);
				b.addDoubleProperty("Module Slide Velocity",		()->this.lv_side, null);
				b.addDoubleProperty("Wheel Force Magnitude",		()->this.F_src_mag, null);
				b.addDoubleProperty("Inline Redistributed Force",	()->this.F_ext_para, null);
				b.addDoubleProperty("Side Redistributed Force",	()->this.F_ext_poip, null);
				b.addDoubleProperty("Inline Friction Force",		()->this.F_frict_para, null);
				b.addDoubleProperty("Side Friction Force",			()->this.F_frict_poip, null);
			}


		}

		public ModuleBuffer[]
			module_buffers;
		public Vector2
			lP_sys		= new Vector2(),	// linear momentum of system
			F_app		= new Vector2(),	// sum applied linear force on frame
			F_frict		= new Vector2(),	// sum linear frict on frame
			F_sys		= new Vector2(),	// net linear force on frame
			lv_field	= new Vector2(),	// linear velocity of frame in field coords
			lv_frame	= new Vector2(),	// linear velocity of frame in frame coords
			la_sys		= new Vector2(),	// linear acceleration of frame in frame coords
			la_field	= new Vector2();	// linear acceleration of frame in field coords
		public double
			lI_momentum,	// linear inertia in the direction of net linear momentum
			rI_momentum,	// rotational inertia in the direction of net rotational momentum
			lI_sys,			// linear inertia in the direction of net linear force
			rI_sys,			// rotational inertia in the direction if net rotational force
			rP_sys,			// rotational momentum of system
			Tq_app,			// applied torque on frame
			Tq_frict,		// frictional torque on frame
			Tq_sys,			// net torque on frame
			rx_frame,		// rotational position of the frame in field coords (angle)
			rx_frame_lv,	// rotational position of the frame's velocity vector (offset angle)
			rx_f_sys,		// rotational position of direction of net force (offset angle)
			rv_frame,		// rotational velocity of the frame
			ra_sys;			// rotational acceleration of the frame

		public DynamicsBuffer(int num_modules) {
			this.module_buffers = new ModuleBuffer[num_modules];
			for(int i = 0; i < num_modules; i++) {
				this.module_buffers[i] = new ModuleBuffer();
			}
		}

		public ModuleBuffer modn(int i) {
			return (i >= 0 && i < this.module_buffers.length) ? this.module_buffers[i] : null;
		}

		@Override
		public void initSendable(SendableBuilder b) {
			b.addDoubleArrayProperty("Frame Momentum 2D",		this.lP_sys::getData, null);
			b.addDoubleArrayProperty("Applied Force 2D",		this.F_app::getData, null);
			b.addDoubleArrayProperty("Friction Force 2D",		this.F_frict::getData, null);
			b.addDoubleArrayProperty("Net Force 2D",			this.F_sys::getData, null);
			b.addDoubleArrayProperty("Field Velocity 2D",		this.lv_field::getData, null);
			b.addDoubleArrayProperty("Frame Velocity 2D",		this.lv_frame::getData, null);
			b.addDoubleArrayProperty("Frame Acceleration 2D",	this.la_sys::getData, null);
			b.addDoubleArrayProperty("Field Acceleration 2D",	this.la_field::getData, null);
			b.addDoubleProperty("Linear Inertia (P)",		()->this.lI_momentum, null);
			b.addDoubleProperty("Rotational Inertia (P)",	()->this.rI_momentum, null);
			b.addDoubleProperty("Linear Inertia (F)",		()->this.lI_sys, null);
			b.addDoubleProperty("Rotational Inertia (F)",	()->this.rI_sys, null);
			b.addDoubleProperty("Rotational Momentum",		()->this.rP_sys, null);
			b.addDoubleProperty("Applied Torque",			()->this.Tq_app, null);
			b.addDoubleProperty("Frictional Torque",		()->this.Tq_frict, null);
			b.addDoubleProperty("Net Torque",				()->this.Tq_sys, null);
			b.addDoubleProperty("Frame Rotation (field)",	()->this.rx_frame, null);
			b.addDoubleProperty("Velocity Direction",		()->this.rx_frame_lv, null);
			b.addDoubleProperty("Net Force Direction",		()->this.rx_f_sys, null);
			b.addDoubleProperty("Rotational Velocity",		()->this.rv_frame, null);
			b.addDoubleProperty("Rotational Acceleration",	()->this.ra_sys, null);
		}
		@Override
		public void initRecursive(SenderNT s, String key) {
			s.putData(key, (Sendable)this);
			for(int i = 0; i < this.module_buffers.length; i++) {
				s.putData(key + "/Module [" + i + "]", this.module_buffers[i]);
			}
		}


	}

	private static class ArrayIterator<T> implements Iterator<T> {

		private final T[] arr;
		private int ptr = 0;

		public ArrayIterator(T[] arr) { this.arr = arr; }


		public void reset() {
			this.ptr = 0;
		}

		@Override
		public boolean hasNext() {
			return this.ptr < arr.length;
		}
		@Override
		public T next() {
			return this.hasNext() ? this.arr[this.ptr++] : null;
		}

	}


}
