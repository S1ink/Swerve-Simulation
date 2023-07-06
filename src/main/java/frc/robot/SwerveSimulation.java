package frc.robot;

import org.ejml.simple.SimpleMatrix;

import edu.wpi.first.math.*;
import edu.wpi.first.math.numbers.*;
import edu.wpi.first.math.system.NumericalIntegration;

import frc.robot.SwerveDrive.*;


public class SwerveSimulation {

	private static final class NX extends Num implements Nat<NX> {

		private final int value;
		public NX(int v) { this.value = v; }

		@Override public int getNum() { return this.value; }

		static NX of(int v) { return new NX(v); }
		static NX of_x(int v, int scale) { return new NX(v * scale); }

	}

	protected enum State {
		SteerAngle		(0),
		SteerRate		(1),
		DrivePosition	(2),
		DriveVelocity	(3);

		public int idx;
		private State(int i) { this.idx = i; }

		public double from(Matrix<NX, N1> x, int module) {
			return x.get(module * 4 + this.idx, 0);
		}
		public void set(Matrix<NX, N1> x_, int module, double val) {
			x_.set(module * 4 + this.idx, 0, val);
		}
	}

	private final SwerveModule[] modules;
	private final int SIZE;
	private final NX n_inputs, n_states;
	// locations2d?

	private final Matrix<NX, N1> u_inputs;
	private Matrix<NX, N1> x_states, y_outputs;



	public SwerveSimulation(SwerveModule... modules) {
		this.modules = modules;
		this.SIZE = modules.length;
		this.n_inputs = NX.of_x(this.SIZE, 2);
		this.n_states = NX.of_x(this.SIZE, 4);
		this.u_inputs = new Matrix<>(this.n_inputs, Nat.N1());
		this.x_states = new Matrix<>(this.n_states, Nat.N1());
		this.y_outputs = this.x_states.copy();
	}


	private synchronized void extractState(SwerveModule module, int index) {
		if(index < this.SIZE) {
			module.setSimulatedSteeringAngle(State.SteerAngle.from(this.y_outputs, index));
			module.setSimulatedSteeringRate(State.SteerRate.from(this.y_outputs, index));
			module.setSimulatedWheelPosition(State.DrivePosition.from(this.y_outputs, index));
			module.setSimulatedWheelVelocity(State.DriveVelocity.from(this.y_outputs, index));
		}
	}

	public synchronized void integrate(double dt_seconds) {
		for(int i = 0; i < this.SIZE; i++) {
			this.u_inputs.set(i * 2 + 0, 0, this.modules[i].getMotorAVolts());
			this.u_inputs.set(i * 2 + 1, 0, this.modules[i].getMotorBVolts());
		}
		this.x_states = NumericalIntegration.rk4(this::dynamics, this.x_states, this.u_inputs, dt_seconds);
		this.y_outputs = x_states.copy();
	}
	public synchronized void updateInternal() {
		for(int i = 0; i < this.SIZE; i++) {
			this.extractState(this.modules[i], i);
		}
	}
	/** Integrate and update the states of the stored modules. If the states should not be updated (ex. not in sim mode), then call integrate() */
	public synchronized void update(double dt_seconds) {
		this.integrate(dt_seconds);
		this.updateInternal();
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

		for(int i = 0; i < this.SIZE; i++) {
			final double
				a_volts = u.get(i * 2, 0),
				b_volts = u.get(i * 2 + 1, 0),
				s_omega = State.SteerRate.from(x, i),
				d_omega = State.DriveVelocity.from(x, i),
				s_torque = this.modules[i].getSteeringTorque(a_volts, b_volts, s_omega, d_omega),
				s_RI = this.modules[i].getSteeringRI(),
				max_frict = STEER_GEARTRAIN_FRICTION_TQ + STEER_FLOOR_FRICTION_TQ,
				steer_aa = applyFriction(s_torque / s_RI, s_omega, max_frict / s_RI, 0.0);
			State.SteerAngle.set(x_prime, i, s_omega);
			State.SteerRate.set(x_prime, i, steer_aa);
		}

		return x_prime;

	}


}
