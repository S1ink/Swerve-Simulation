package frc.robot.swerve;

import java.util.Arrays;

import org.ejml.simple.SimpleMatrix;

import edu.wpi.first.math.geometry.*;

import frc.robot.swerve.SwerveUtils.*;


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


	public SwerveModuleStates[] toModuleStates(ChassisStates robot_state, SwerveModuleStates[] output) {
		return this.toModuleStates(robot_state, output, no_recenter);
	}

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


	/** The output linear velocities and accelerations are in the robot's coordinate space. */
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
