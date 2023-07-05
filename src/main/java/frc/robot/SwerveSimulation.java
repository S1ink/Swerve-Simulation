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

	protected Matrix<NX, N1> dynamics(Matrix<NX, N1> x, Matrix<NX, N1> u) {

		Matrix<NX, N1> x_prime = new Matrix<>(x.getStorage().createLike());

		// compute derivatives here

		return x_prime;

	}


}
