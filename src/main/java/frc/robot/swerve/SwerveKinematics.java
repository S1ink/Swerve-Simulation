package frc.robot.swerve;

import java.util.Arrays;
import java.util.Collections;

import org.ejml.simple.SimpleMatrix;

import edu.wpi.first.math.geometry.*;
import frc.robot.swerve.SwerveUtils.*;


/** SwerveKinematics provides conversions between the robot state and each module's state. */
public final class SwerveKinematics {

	protected final Translation2d[]
		module_locations;
	protected final SimpleMatrix
		inv_kinematics, inv_kinematics2,
		fwd_kinematics, fwd_kinematics2;
	protected final int SIZE;
	protected static Translation2d
		no_recenter = new Translation2d();
	protected Translation2d
		stored_recenter = no_recenter;


	/** Create a SwerveKinematics instance for a set of modules in the provided robot-relative positions.
	 * All method calls assume the same module ordering as are provided here. */
	public SwerveKinematics(Translation2d... locations) {

		this.SIZE = locations.length;
		this.module_locations = locations;

		this.inv_kinematics = new SimpleMatrix(this.SIZE * 2, 3);
		this.inv_kinematics2 = new SimpleMatrix(this.SIZE * 2, 4);

		for(int i = 0; i < this.SIZE; i++) {
			final double
				x = this.module_locations[i].getX(),
				y = this.module_locations[i].getY();
			this.inv_kinematics.setRow(i * 2 + 0, 0, 1, 0, -y);
			this.inv_kinematics.setRow(i * 2 + 1, 0, 0, 1, +x);
			this.inv_kinematics2.setRow(i * 2 + 0, 0, 1, 0, -x, -y);
			this.inv_kinematics2.setRow(i * 2 + 1, 0, 0, 1, -y, +x);
		}

		this.fwd_kinematics = this.inv_kinematics.pseudoInverse();
		this.fwd_kinematics2 = this.inv_kinematics2.pseudoInverse();
		
	}


	/** Get the module states required for a chassis state. */
	public SwerveModuleStates[] toModuleStates(ChassisStates robot_state, SwerveModuleStates[] output) {
		return this.toModuleStates(robot_state, output, no_recenter);
	}

	/** Get the module states required for a chassis state with a modifiable axis of rotation. */
	public SwerveModuleStates[] toModuleStates(ChassisStates robot_state, SwerveModuleStates[] output, Translation2d recenter) {

		if(!this.stored_recenter.equals(recenter)) {

			for(int i = 0; i < this.SIZE; i++) {
				final double
					x = this.module_locations[i].getX() - recenter.getX(),
					y = this.module_locations[i].getY() - recenter.getY();
				this.inv_kinematics.setRow(i * 2 + 0, 0, 1, 0, -y);
				this.inv_kinematics.setRow(i * 2 + 1, 0, 0, 1, +x);
				this.inv_kinematics2.setRow(i * 2 + 0, 0, 1, 0, -x, -y);
				this.inv_kinematics2.setRow(i * 2 + 1, 0, 0, 1, -y, +x);
			}
			this.stored_recenter = recenter;

		}

		final SimpleMatrix
			first_order_inputs = new SimpleMatrix(3, 1),
			second_order_inputs = new SimpleMatrix(4, 1);

		first_order_inputs.setColumn(0, 0,
			robot_state.x_velocity,
			robot_state.y_velocity,
			robot_state.angular_velocity
		);
		second_order_inputs.setColumn(0, 0,
			robot_state.x_acceleration,
			robot_state.y_acceleration,
			robot_state.angular_velocity * robot_state.angular_velocity,
			robot_state.angular_acceleration
		);

		final SimpleMatrix
			first_order_states = this.inv_kinematics.mult(first_order_inputs),
			second_order_states = this.inv_kinematics2.mult(second_order_inputs);

		if(output == null || output.length < this.SIZE) {
			output = new SwerveModuleStates[this.SIZE];
		}
		for(int i = 0; i < this.SIZE; i++) {
			final double
				x_v = first_order_states.get(i * 2 + 0, 0),
				y_v = first_order_states.get(i * 2 + 1, 0),
				v = Math.hypot(x_v, y_v),
				x_a = second_order_states.get(i * 2 + 0, 0),
				y_a = second_order_states.get(i * 2 + 1, 0);
			final Rotation2d
				angle = new Rotation2d(x_v, y_v);
			final double
				sin = angle.getSin(),
				cos = angle.getCos(),
				a = cos * x_a + sin * y_a,
				omega = (v == 0) ? 0 : ((-sin * x_a + cos * y_a) / v);

			output[i] = SwerveModuleStates.makeSecondOrder(angle, v, omega, a);
		}

		return output;

	}

	/** Get the module states for a locking (aka 'X') formation - used when trying to stay in the same position. */
	public SwerveModuleStates[] lockFormation(Translation2d relative_center, SwerveModuleStates[] output) {
		if(output == null || output.length < this.SIZE) {
			output = new SwerveModuleStates[this.SIZE];
		}
		for(int i = 0; i < this.SIZE; i++) {
			if(output[i] == null) output[i] = new SwerveModuleStates();
			else output[i].zero();
			output[i].rotation = (this.module_locations[i].minus(relative_center).getAngle().getRadians());
		}
		return output;
	}
	/** Get the module states for a locking (aka 'X') formation - used when trying to stay in the same position. */
	public SwerveModuleStates[] lockFormation(SwerveModuleStates[] output) {
		return this.lockFormation(no_recenter, output);
	}


	/** Get the resulting chassis state from a set of module states. */
	public ChassisStates toChassisStates(SwerveModuleStates... states) {

		if(states.length < this.SIZE) {
			return null;
		}

		final SimpleMatrix
			module_states_order1 = new SimpleMatrix(this.SIZE * 2, 1),
			module_states_order2 = new SimpleMatrix(this.SIZE * 2, 1);

		for(int i = 0; i < this.SIZE; i++) {
			final SwerveModuleStates state = states[i];
			final double
				lv = state.linear_velocity,
				av = state.angular_velocity,
				la = state.linear_acceleration,
				sin = Math.sin(state.rotation),
				cos = Math.cos(state.rotation),
				la_x = (cos * la - sin * lv * av),
				la_y = (sin * la + cos * lv * av);
			module_states_order1.set(i * 2 + 0, 0, lv * cos);
			module_states_order1.set(i * 2 + 1, 0, lv * sin);
			module_states_order2.set(i * 2 + 0, 0, la_x);
			module_states_order2.set(i * 2 + 1, 0, la_y);
		}

		final SimpleMatrix
			chassis_states_order1 = this.fwd_kinematics.mult(module_states_order1),		// first order states are (3x1)[vx, vy, omega]
			chassis_states_order2 = this.fwd_kinematics2.mult(module_states_order2);	// second order states are (4x1)[ax, ay, omega^2, alpha]
			
		return new ChassisStates(	// output is from robot reference
			chassis_states_order1.get(0, 0),	// vx
			chassis_states_order1.get(1, 0),	// vy
			chassis_states_order1.get(2, 0),	// omega
			chassis_states_order2.get(0, 0),	// ax
			chassis_states_order2.get(1, 0),	// ay
			chassis_states_order2.get(3, 0)	// alpha
		);

	}



	/** Normalize a set of module states based on a maximum wheel velocity. */
	public static void normalizeModuleVelocities(double max_velocity, SwerveModuleStates... states) {
		final double max = Collections.max(
			Arrays.asList(states),
			(SwerveModuleStates a, SwerveModuleStates b)->Double.compare(a.linear_velocity, b.linear_velocity)
		).linear_velocity;
		if(max > max_velocity) {
			for(final SwerveModuleStates s : states) {
				s.angular_velocity *= (max_velocity / max);
			}
		}
	}





	public static SimpleMatrix invKinematicsMat_D1(Translation2d... modules) {
		return invKinematicsMat_D1(new SimpleMatrix(modules.length * 2, 3), modules);
	}
	public static SimpleMatrix invKinematicsMat_D1(SimpleMatrix mat, Translation2d... modules) {
		final int LEN = modules.length;
		if(mat.numCols() != 3 || mat.numRows() != LEN * 2) {
			mat.reshape(LEN * 2, 3);
		}
		for(int i = 0; i < LEN; i++) {
			mat.setRow(i * 2 + 0, 0, 1, 0, -modules[i].getY());
			mat.setRow(i * 2 + 1, 0, 0, 1, +modules[i].getX());
		}
		return mat;
	}
	public static SimpleMatrix invKinematicsMat_D2(Translation2d... modules) {
		return invKinematicsMat_D2(new SimpleMatrix(modules.length * 2, 4), modules);
	}
	public static SimpleMatrix invKinematicsMat_D2(SimpleMatrix mat, Translation2d... modules) {
		final int LEN = modules.length;
		if(mat.numCols() != 4 || mat.numRows() != LEN * 2) {
			mat.reshape(LEN * 2, 4);
		}
		for(int i = 0; i < LEN; i++) {
			final double x = modules[i].getX(), y = modules[i].getY();
			mat.setRow(i * 2 + 0, 0, 1, 0, -x, -y);
			mat.setRow(i * 2 + 1, 0, 0, 1, -y, +x);
		}
		return mat;
	}


}
