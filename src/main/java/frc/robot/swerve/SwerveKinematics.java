package frc.robot.swerve;

import java.util.Arrays;

import org.ejml.simple.SimpleMatrix;

import edu.wpi.first.math.geometry.*;

import frc.robot.swerve.SwerveUtils.*;


public final class SwerveKinematics {

	protected final Translation2d[]
		module_locations;
	protected final SimpleMatrix
		inv_kinematics, fwd_kinematics,
		inv_kinematics2, fwd_kinematics2;
	protected final int SIZE;
	protected SwerveModuleStates[]
		stored_states;
	protected Translation2d
		stored_recenter = new Translation2d();


	public SwerveKinematics(Translation2d... locations) {

		this.SIZE = locations.length;
		this.module_locations = locations;
		this.stored_states = new SwerveModuleStates[this.SIZE];
		Arrays.fill(this.stored_states, new SwerveModuleStates());

		this.inv_kinematics = new SimpleMatrix(this.SIZE * 2, 3);
		this.inv_kinematics2 = new SimpleMatrix(this.SIZE * 2, 4);

		for(int i = 0; i < this.SIZE; i++) {
			final double x = this.module_locations[i].getX(), y = this.module_locations[i].getY();
			this.inv_kinematics.setRow(i * 2 + 0, 0, 1, 0, -y);
			this.inv_kinematics.setRow(i * 2 + 1, 0, 0, 1, +x);
			this.inv_kinematics2.setRow(i * 2 + 0, 0, 1, 0, -x, -y);
			this.inv_kinematics2.setRow(i * 2 + 1, 0, 0, 1, -y, +x);
		}

		this.fwd_kinematics = this.inv_kinematics.pseudoInverse();
		this.fwd_kinematics2 = this.inv_kinematics2.pseudoInverse();
		
	}


	public SwerveModuleStates[] toModuleStates(ChassisStates robot_state, Translation2d recenter) {

		// remember to make buffer to ensure that if the robot isn't moving, the kineamics should stay the same
		// setting wheels to x state

		if(this.stored_recenter.equals(recenter)) {

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
		
		this.stored_states = new SwerveModuleStates[this.SIZE];
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
				omega = (-sin * x_a + cos * y_a) / v;

			this.stored_states[i] = SwerveModuleStates.makeSecondOrder(angle, v, omega, a);
		}

		return this.stored_states;

	}

	public SwerveModuleStates[] toModuleStates(ChassisStates robot_state) {
		return this.toModuleStates(robot_state, new Translation2d());
	}


	public ChassisStates toChassisStates(SwerveModuleStates... states) {

		// might need to check length of states

		final SimpleMatrix
			module_states_order1 = new SimpleMatrix(this.SIZE * 2, 1),
			module_states_order2 = new SimpleMatrix(this.SIZE * 2, 1);

		for(int i = 0; i < this.SIZE; i++) {
			SwerveModuleStates state = states[i];
			final double
				lv = state.linear_velocity,
				av = state.angular_velocity,
				la = state.linear_acceleration,
				sin = state.angle.getSin(),
				cos = state.angle.getCos(),
				la_x = (cos * la - sin * lv * av),
				la_y = (sin * la + cos * lv * av);
			module_states_order1.set(i * 2 + 0, 0, lv * sin);
			module_states_order1.set(i * 2 + 1, 0, lv * cos);
			module_states_order2.set(i * 2 + 0, 0, la_x);
			module_states_order2.set(i * 2 + 1, 0, la_y);
		}

		final SimpleMatrix
			chassis_states_order1 = this.fwd_kinematics.mult(module_states_order1),
			chassis_states_order2 = this.fwd_kinematics2.mult(module_states_order2);
			
		return new ChassisStates(
			chassis_states_order1.get(0, 0),
			chassis_states_order1.get(1, 0),
			chassis_states_order1.get(2, 0),
			chassis_states_order2.get(0, 0),
			chassis_states_order2.get(1, 0),
			chassis_states_order2.get(2, 0)
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
