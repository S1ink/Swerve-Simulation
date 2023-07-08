package frc.robot.swerve;

import java.util.Arrays;

import org.ejml.simple.SimpleMatrix;

import edu.wpi.first.math.geometry.*;

import frc.robot.swerve.SwerveUtils.*;


public final class SwerveKinematics {

	protected final SimpleMatrix
		inv_kinematics, fwd_kinematics, inv_kinematics2, fwd_kinematics2;
	protected final int SIZE;
	protected final Translation2d[] module_locations;
	protected SwerveModuleStates[] stored_states;
	protected Translation2d previous_rotation_center;

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


	public SwerveModuleStates[] toModuleStates(ChassisStates robot_state, Translation2d rotational_center) {

		// remember to make buffer to ensure that if the robot isn't moving, the kineamics should stay the same
		// setting wheels to x state

		if(this.previous_rotation_center.equals(rotational_center)) {

			for(int i = 0; i < this.SIZE; i++) {
				final double
					x = this.module_locations[i].getX() - rotational_center.getX(),
					y = this.module_locations[i].getY() - rotational_center.getY();
				this.inv_kinematics.setRow(i * 2 + 0, 0, 1, 0, -y);
				this.inv_kinematics.setRow(i * 2 + 1, 0, 0, 1, +x);
				this.inv_kinematics2.setRow(i * 2 + 0, 0, 1, 0, -x, -y);
				this.inv_kinematics2.setRow(i * 2 + 1, 0, 0, 1, -y, +x);
			}
			this.previous_rotation_center = rotational_center;

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

			this.stored_states[i] = new SwerveModuleStates(angle, v, omega, a);
		}

		return this.stored_states;

	}

	public SwerveModuleStates[] toModuleStates(ChassisStates robot_state) {
		return this.toModuleStates(robot_state, new Translation2d());
	}

}
