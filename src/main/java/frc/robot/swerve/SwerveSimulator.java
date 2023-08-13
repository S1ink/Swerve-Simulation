package frc.robot.swerve;

import org.ejml.simple.SimpleMatrix;

import edu.wpi.first.math.*;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.*;
import edu.wpi.first.math.system.NumericalIntegration;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.math.Vector;

import frc.robot.swerve.SwerveDrive.*;
import frc.robot.swerve.SwerveUtils.*;


public class SwerveSimulator implements Sendable {

	public static interface FrictionSim {

		public static final double DEFAULT_MOMENTUM_EPSILON = 1e-5;		// no standardized units so this may not be applicable for every situation
		public static boolean movement(double momentum) { return movement(momentum, DEFAULT_MOMENTUM_EPSILON); }
		public static boolean movement(double momentum, double epsilon) { return Math.abs(momentum) > epsilon; }

		public double rawFriction(double force, double momentum);

		// default public double applyFriction(double force, double momentum) {	// "dumb" (non temporal) addition of applied friciton vector and external force
		// 	final double raw = this.rawFriction(force, momentum);
		// 	return force + Math.copySign(Math.min(Math.abs(raw), Math.abs(force)), raw);
		// }
		// default public double applyFriction(double force, double momentum, double dt) {		// "smart" integration of applied friction as it opposes momentum and external forces temporally -- returns the time-weighted average applied force over the change in time
		// 	final double initial = force + this.rawFriction(force, momentum);
		// 	double dt_a = 0.0;
		// 	if((initial * momentum < 0.0) && ((dt_a = Math.abs(momentum / initial)) < dt)) {		// inital sum opposes the momentum AND cancels out the momentum in less time than 'dt'
		// 		final double secondary = force + this.rawFriction(force, 0.0);

		// 	} else {
		// 		return initial;
		// 	}
		// }
	}
	public static class DualStateFrictionSim implements FrictionSim {

		public final double
			u_static, u_kinetic;
		protected double
			norm_scalar = 0.0;

		public DualStateFrictionSim(double u_static, double u_kinetic) {
			this.u_static = u_static;
			this.u_kinetic = u_kinetic;
		}
		public DualStateFrictionSim(double u_static, double u_kinetic, double norm_force) {
			this(u_static, u_kinetic);
			this.norm_scalar = norm_force;
		}

		public void setNormalForce(double f) { this.norm_scalar = f; }
		private double appliedStatic() { return this.norm_scalar * this.u_static; }
		private double appliedKinetic() { return this.norm_scalar * this.u_kinetic; }

		@Override
		public double rawFriction(double force, double momentum) {
			if(FrictionSim.movement(momentum)) {
				return -Math.copySign(appliedKinetic(), momentum);	// (kinetic) friction will oppose the prexisting momentum
			} else {
				return -Math.copySign(appliedStatic(), force);	// (static) friciton will oppose the applied force
			}
		}

	}

	/** {@link ModuleSim} represents/contains all the necessary physical behavior (or approximations) needed
	 * for the simulator to be able to simulate the robot's movement as a result of voltage inputs and possible
	 * additional external forces. */
	public static interface ModuleSim {

		/** Get the torque applied to the wheel assembly about the steering axis in Nm ("felt" by the output of any possible gearing)
		 * as a result of the motor dynamics. 
		 * @param a_volts - the voltage applied to motor A in volts
		 * @param b_volts - the voltage applied to motor B in volts
		 * @param steer_rate - the angular velocity of the wheel assembly about the steering axis in rad/s
		 * @param wheel_vel_linear - the linear velocity of the module along the wheel path in m/s
		 * @return the resulting torque about the steer axis in Nm */
		public double steerTorqueM(double a_volts, double b_volts, double steer_rate, double wheel_vel_linear);
		/** Get the sum angular acceleration of the wheel assembly about the steer axis
		 * @param steer_torque - the torque applied about the steer axis as a result of the motor dynamics in Nm
		 * @param steer_rate - the current angular velocity of the wheel assimbly about the steer axis in rad/s
		 * @param wheel_vel_linear - the linear velocity of the module along the wheel path in m/s
		 * @param f_norm - the normal force in N applied at the wheel-floor contact as a result of gravity and possible external forces
		 * @return the angular acceleration of the wheel assembly about the steering axis in rad/s^2 */
		public double steerAAccel(double steer_torque, double steer_rate, double wheel_vel_linear, double f_norm);

		/** Get the output force at the wheel-floor contact in N as a result of the motor dynamics.
		 * @param a_volts - the voltage applied to motor A in volts
		 * @param b_volts - the voltage applied to motor B in volts
		 * @param steer_rate - the angular velocity of the wheel assembly about the steering axis in rad/s
		 * @param wheel_vel_linear - the linear velocity of the module along the wheel path in m/s
		 * @return the resulting force applied by the wheel (at the floor) in N */
		public double wheelForceM(double a_volts, double b_volts, double steer_rate, double wheel_vel_linear);
		/** Get the friction force in N applied at the wheel-floor contact 
		 * as a result of the wheel being pushed/moved sideways.
		 * @param f_src - the sum external force acting on the wheel in N
		 * @param momentum - the robot's tangential momentum felt at the wheel contact location in Kgm/s
		 * @param f_norm - the normal force in N applied at the wheel-floor contact as a result of gravity and possible external forces
		 * @return the friciton force in N */
		public double wheelSideFriction(double f_src, double momentum, double f_norm);
		/** Get the friction force in Nm applied at the wheel radius as a result of the wheel being
		 * pushed/moved inline with its heading.
		 * @param f_src - the sum external force acting on the wheel in N
		 * @param momenum - the robot's tangential momentum felt at the wheel contact location in Kgm/s
		 * @param f_norm - the normal force in N applied at the wheel-floor contact as a result of gravity and possible external forces
		 * @return the friction force in N at the wheel radius */
		public double wheelGearFriction(double f_src, double momentum, double f_norm);

		/** Get the linear inertia of the module in Kg, not including any inertia from the wheel gear train.
		 * @return the mass of the module in Kg	*/
		public double moduleMass();
		/** Get the effective linear inertia of the module in Kg - meaning the acutal mass plus any
		 * rotational inertia from the wheel being inline with the force applied, translated by the radius squared
		 * to represent a linear effect
		 * @param vec_wheel_dtheta - The delta angle between the wheel heading and the applicant vector. A delta of 0 means that the vector and wheel are inline.
		 * @return the combined linear inertia of the module's mass in Kg and any additional inertia introduced from the wheel's geartrain being inline with the applied force. */
		public double effectiveLinearInertia(double vec_wheel_dtheta);	// << 1/sqrt((cos(theta)/(full inertia))^2 + (sin(theta)/(min inertia))^2)
		/** Get the effective rotational inertia in Kgm^2 of the module about the robot's center of gravity. 
		 * @param vec_wheel_dtheta - the delta angle between the wheel heading and the tangential direction that the vector is acting in
		 * @param module_radius - the distance from the center of the module to the robot's center of mass, ie the radius at which the vector is being applied, and the radius at which the module's inertia is orbiting
		 * @return the combined rotatinal inertia in Kgm^2 of the module (about the robot CG) that is a result of the module's static inertia and any additional
		 * inertia introduced by the wheel geartrain. */
		public double effectiveRotationalInertia(double vec_wheel_dtheta, double module_radius);

	}

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

	private final SwerveModule[] modules;
	private final ModuleSim[] module_sims;
	private final SimConfig config;
	private final SwerveVisualization visualization;
	private final Vector2[] module_locs, module_dirs;
	private final int SIZE;
	private double STATIC_MASS;
	private final NX n_inputs, n_states;
	// locations2d?

	private final Matrix<NX, N1> u_inputs;
	private Matrix<NX, N1> x_states, y_outputs;



	public SwerveSimulator(SimConfig config, SwerveModule... modules) {
		this(config, null, modules);
	}
	public SwerveSimulator(SimConfig config, ModuleSim sim_properties, SwerveModule... modules) {
		this(new SwerveVisualization(getTranslations(modules)), config, sim_properties, modules);
	}
	public SwerveSimulator(SwerveVisualization viz, SimConfig config, SwerveModule... modules) {
		this(viz, config, null, modules);
	}
	public SwerveSimulator(SwerveVisualization viz, SimConfig config, ModuleSim sim_properties, SwerveModule... modules) {
		this.modules = modules;
		this.visualization = viz;
		this.config = config;
		this.SIZE = modules.length;
		this.module_sims = new ModuleSim[this.SIZE];
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

	public void applySimProperties(ModuleSim properties) {
		this.STATIC_MASS = this.config.ROBOT_MASS + properties.moduleMass() * this.SIZE;
		for(int i = 0; i < this.SIZE; i++) {
			this.module_sims[i] = properties;
		}
	}
	public void applyModuleSpecificProperties() {
		this.STATIC_MASS = this.config.ROBOT_MASS;
		for(int i = 0; i < this.SIZE; i++) {
			final ModuleSim props = null;	// get from this.modules[i]
			if(props != null) {
				this.module_sims[i] = props;
				this.STATIC_MASS += props.moduleMass();
			} else if(this.module_sims[i] != null) {
				this.STATIC_MASS += this.module_sims[i].moduleMass();
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


	protected final double
		STEER_GEARTRAIN_FRICTION_TQ = 1.0,
		STEER_FLOOR_FRICTION_TQ = 10.0;		// coeff of friction * f_norm (robot mass * fg) -- integrated about the wheel contact patch (radius at each point) --> multiply by 2/3*r

	protected double applyFriction(double src_d2x, double src_dx, double f_d2x, double dt) {
		final double vdir = Math.signum(src_dx);	// apply epsilon --> 0.0
		if(vdir != 0.0) {	// friction opposes the movement
			return src_d2x - Math.min(Math.abs(f_d2x), Math.abs(src_dx / dt)) * vdir;	// make sure that the applied friction cannot reverse the direction based on dt
		} else {	// sum the source forces and friction (applied in opposition)
			return src_d2x - Math.min(Math.abs(f_d2x), Math.abs(src_d2x)) * Math.signum(src_d2x);
		}
	}
	protected Matrix<NX, N1> dynamics(Matrix<NX, N1> x, Matrix<NX, N1> u) {

		Matrix<NX, N1> x_prime = new Matrix<>(x.getStorage().createLike());

		final Vector2 f_src = new Vector2();
		double tq_src = 0.0, momentum_LI = this.config.ROBOT_MASS, momentum_RI = this.config.ROBOT_RI;
		final double[] wheel_headings = new double[this.SIZE];

		final Vector2 f_vel = new Vector2(
			State.FrameVelocityX.from(x),
			State.FrameVelocityY.from(x)
		); // transform by heading???
		final double
			f_avel = State.FrameAngularVel.from(x),
			f_vel_theta = f_vel.theta();

		for(int i = 0; i < this.SIZE; i++) {
			final double
				a_volts = u.get(i * 2, 0),
				b_volts = u.get(i * 2 + 1, 0),
				s_angle = State.SteerAngle.fromN(x, i) % (Math.PI * 2),
				s_omega = State.SteerRate.fromN(x, i),
				d_velocity = State.DriveVelocity.fromN(x, i),
				s_torque = this.module_sims[i].steerTorqueM(a_volts, b_volts, s_omega, d_velocity),
				steer_aa = this.module_sims[i].steerAAccel(s_torque, s_omega, d_velocity, this.STATIC_MASS * 9.8),	// STEP 0: module turn AA
				f_wheel = this.module_sims[i].wheelForceM(a_volts, b_volts, s_omega, d_velocity);
			State.SteerAngle.setN(x_prime, i, s_omega);
			State.SteerRate.setN(x_prime, i, steer_aa);
			State.DrivePosition.setN(x_prime, i, d_velocity);

			final Vector2 wf = Vector2.fromPolar(f_wheel, s_angle);
			f_src.append(wf);
			tq_src += this.module_locs[i].cross(wf);			// STEP 1: sum the wheel force vectors to get the net linear force and net torque
			wheel_headings[i] = s_angle;

			momentum_LI += this.module_sims[i].effectiveLinearInertia(f_vel_theta - s_angle);
			momentum_RI += this.module_sims[i].effectiveRotationalInertia(Vector2.cross(this.module_locs[i], f_avel).theta() - s_angle, this.module_locs[i].norm());
		}
		// debug output for summed force/torque
		// find system momentum
		final Vector2 l_momentum = new Vector2(f_vel).times(momentum_LI);
		final double r_momentum = f_avel * momentum_RI;

		final Vector2 f_frict = new Vector2();
		double tq_frict = 0.0;
		for(int i = 0; i < this.SIZE; i++) {
			final Vector2 f = Vector2.add(
				f_src,
				Vector2.invCross(this.module_locs[i], tq_src)
			).div((double)this.SIZE);
			final Vector2 p = Vector2.add(
				l_momentum,
				Vector2.invCross(this.module_locs[i], r_momentum)
			).div((double)this.SIZE);
			final Vector2 wdir = Vector2.fromPolar(1.0, wheel_headings[i]);
			final double
				f_para = Vector2.dot(f, wdir),
				f_poip = Vector2.cross(f, wdir),
				p_para = Vector2.dot(p, wdir),
				p_poip = Vector2.cross(p, wdir),
				fr_inline = this.module_sims[i].wheelGearFriction(f_para, p_para, this.STATIC_MASS * 9.8),
				fr_side = this.module_sims[i].wheelSideFriction(f_poip, p_poip, this.STATIC_MASS * 9.8);
			final Vector2 frict = new Vector2(fr_inline, fr_side).rotate(wheel_headings[i]);

			f_frict.append(frict);
			tq_frict += this.module_locs[i].cross(frict);
		}
		// add sources and friction
		final Vector2 f_sys = new Vector2();
		final double tq_sys = 0.0, f_sys_theta = f_sys.theta();
		double sys_LI = this.config.ROBOT_MASS, sys_RI = this.config.ROBOT_RI;
		for(int i = 0; i < this.SIZE; i++) {
			sys_LI += this.module_sims[i].effectiveLinearInertia(f_sys_theta - wheel_headings[i]);
			sys_RI += this.module_sims[i].effectiveRotationalInertia(Vector2.cross(this.module_locs[i], tq_sys).theta() - wheel_headings[i], this.module_locs[i].norm());
		}
		final Vector2 frame_acc = new Vector2(f_sys).div(sys_LI);
		final double frame_aacc = tq_sys / sys_RI;

		// transform based on heading???
		State.FrameVelocityX.set(x_prime, frame_acc.x());
		State.FrameVelocityY.set(x_prime, frame_acc.y());
		State.FrameAngularVel.set(x_prime, frame_aacc);
		State.FramePositionX.set(x_prime, f_vel.x());
		State.FramePositionY.set(x_prime, f_vel.y());
		State.FrameRotation.set(x_prime, f_avel);

		// use kinematics to set wheel accelerations

		return x_prime;

	}



	@Override
	public void initSendable(SendableBuilder b) {
		
	}










	public static class Vector2 extends Vector<N2> {

		public Vector2()
			{ this(0, 0); }
		public Vector2(double x, double y) {
			super(Nat.N2());
			this.set(x, y);
		}
		public Vector2(Translation2d v) {
			this(v.getX(), v.getY());
		}
		public Vector2(Matrix<N2, N1> mat) {
			super(mat);
		}

		public double x() {
			return this.get(0, 0);
		}
		public double y() {
			return this.get(1, 0);
		}
		public Vector2 set(double x, double y) {
			this.set(0, 0, x);
			this.set(1, 0, y);
			return this;
		}
		public Vector2 set(Matrix<N2, N1> m) {
			super.m_storage.setTo(m.getStorage());
			return this;
		}

		public Vector2 negate() {
			this.set(-x(), -y());
			return this;
		}
		public Vector2 append(Vector2 v) {
			this.set(x() + v.x(), y() + v.y());
			return this;
		}
		public Vector2 normalize() {
			final double n = this.norm();
			this.set(x() / n, y() / n);
			return this;
		}
		public Vector2 div(double v) {
			this.set(x() / v, y() / v);
			return this;
		}
		public Vector2 times(double v) {
			this.set(x() * v, y() * v);
			return this;
		}

		public Vector2 rotate(double radians) {
			final double
				sin = Math.sin(radians),
				cos = Math.cos(radians);
			this.set(
				this.x() * cos - this.y() * sin,
				this.x() * sin + this.y() * cos
			);
			return this;
		}
		public Vector2 transform(Matrix<N2, N2> rmat) {
			this.set(rmat.times(this));
			return this;
		}
		public double dot(Vector2 v) {
			return this.x() * v.x() + this.y() * v.y();
		}
		public double cross(Vector2 v) {
			return (this.x() * v.y()) - (this.y() * v.x());
		}
		public Vector2 cross(double z_projection) {		// takes the cross product between the current vector and a z-projected vector -- thus rotating the current vector about z by 90 degrees (CCW+)
			this.set(-z_projection * this.y(), z_projection * this.x());
			return this;
		}
		public Vector2 cross_CW(double z_projection) {	// the opposite of the normal CCW+ cross (CW+) -- this is the same as applying the above cross product backwards
			this.set(z_projection * this.y(), -z_projection * this.x());
			return this;
		}
		public double norm() {
			return Math.sqrt(this.dot(this));
		}
		public double sin(Vector2 v) {
			return this.cross(v) / (this.norm() * v.norm());
		}
		public double cos(Vector2 v) {
			return this.dot(v) / (this.norm() * v.norm());
		}
		public double theta() {
			return Math.atan2(y(), x());
		}


		public static Vector2 add(Vector2 a, Vector2 b) {
			return new Vector2(a.x() + b.x(), a.y() + b.y());
		}
		public static Vector2 sub(Vector2 a, Vector2 b) {
			return new Vector2(a.x() - b.x(), a.y() - b.y());
		}
		public static Vector2 fromPolar(double mag, double radians) {
			return new Vector2(mag * Math.cos(radians), mag * Math.sin(radians));
		}
		public static Vector2 rotate(Vector2 v, double radians) {
			return new Vector2(v).rotate(radians);
		}
		public static Vector2 transform(Vector2 v, Matrix<N2, N2> t) {
			return new Vector2(v).transform(t);
		}
		public static double dot(Vector2 a, Vector2 b) {
			return a.dot(b);
		}
		public static double cross(Vector2 a, Vector2 b) {
			return a.cross(b);
		}
		public static Vector2 cross(double z_projection, Vector2 v) {
			return new Vector2(v).cross(z_projection);
		}
		public static Vector2 cross(Vector2 v, double z_projection) {
			return new Vector2(v).cross_CW(z_projection);
		}
		public static Vector2 invCross(Vector2 r, double z) {			// perform a cross product but the result is equal to |tq|sin(theta)/|r| not |tq||r|sin(theta) -- used to find a resultant force vector from a given torque and applicant radius vector
			return new Vector2(r).cross(z).div(Vector2.dot(r, r));
		}
		public static Vector2 unitVec(Vector2 v) {
			return new Vector2(v).normalize();
		}
		public static double sin(Vector2 a, Vector2 b) {
			return a.sin(b);
		}
		public static double cos(Vector2 a, Vector2 b) {
			return a.cos(b);
		}

		public static Matrix<N2, N2> rmat(double radians) {
			final double
				sin = Math.sin(radians),
				cos = Math.cos(radians);
			return Matrix.mat(Nat.N2(), Nat.N2()).fill(
				+cos, -sin,
				+sin, +cos
			);
		}


	}

}
