package frc.robot.swerve;

import java.util.function.Function;

import org.ejml.simple.SimpleMatrix;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.*;


public final class SwerveUtils {

	public static class ChassisStates {
		
		public double
			x_velocity,
			y_velocity,
			angular_velocity,
			x_acceleration,
			y_acceleration,
			angular_acceleration;

		public ChassisStates() {}
		public ChassisStates(double x_v, double y_v, double r_v) {
			this(x_v, y_v, r_v, 0.0, 0.0, 0.0);
		}
		public ChassisStates(double x_v, double y_v, double r_v, double x_a, double y_a, double r_a) {
			this.x_velocity = x_v;
			this.y_velocity = y_v;
			this.angular_velocity = r_v;
			this.x_acceleration = x_a;
			this.y_acceleration = y_a;
			this.angular_acceleration = r_a;
		}

		public static ChassisStates fromFieldRelative(
			double x_v, double y_v, double r_v, double x_a, double y_a, double r_a,
			Rotation2d robot_heading
		) {
			final double sin = robot_heading.getSin(), cos = robot_heading.getCos();
			return new ChassisStates(
				x_v * cos + y_v * sin,
				x_v * -sin + y_v * cos,
				r_v,
				x_a * cos + y_a * sin,
				x_a * -sin + y_a * cos,
				r_a
			);
		}
		public static ChassisStates fromFieldRelative(ChassisStates states, Rotation2d robot_heading) {
			return fromFieldRelative(
				states.x_velocity,
				states.y_velocity,
				states.angular_velocity,
				states.x_acceleration,
				states.y_acceleration,
				states.angular_acceleration,
				robot_heading
			);
		}

	}



	public static class SwerveModuleStates {

		public Rotation2d
			angle;
		public double
			linear_velocity,
			angular_velocity,
			linear_acceleration;

		public SwerveModuleStates() { this.angle = new Rotation2d(); }
		public SwerveModuleStates(Rotation2d angle, double linear_vel, double angular_vel, double linear_acc) {
			this.angle = angle;
			this.linear_velocity = linear_vel;
			this.angular_velocity = angular_vel;
			this.linear_acceleration = linear_acc;
		}

		public static SwerveModuleStates optimize(SwerveModuleStates desired_states, Rotation2d currentAngle) {
			var delta = desired_states.angle.minus(currentAngle);
			if (Math.abs(delta.getDegrees()) > 90.0) {
				return new SwerveModuleStates(
					desired_states.angle.rotateBy(Rotation2d.fromDegrees(180.0)),
					-desired_states.linear_velocity,
					desired_states.angular_velocity,
					desired_states.linear_acceleration
				);
			} else {
				return new SwerveModuleStates(
					desired_states.angle,
					desired_states.linear_velocity,
					desired_states.angular_velocity,
					desired_states.linear_acceleration
				);
			}
		}

	}



	public static class SwerveDriveKinematics2 extends SwerveDriveKinematics {

		private final SimpleMatrix inv_kinematics, fwd_kinematics;
		private final Translation2d[] locations;
		private final int SIZE;

		public SwerveDriveKinematics2(Translation2d... module_locations) {
			super(module_locations);
			this.locations = module_locations;
			this.SIZE = module_locations.length;
			this.inv_kinematics = new SimpleMatrix(this.SIZE * 2, 4);
			for(int i = 0; i < this.SIZE; i++) {
				final double x = this.locations[i].getX(), y = this.locations[i].getY();
				this.inv_kinematics.setRow(i * 2 + 0, 0, 1, 0, -x, -y);
				this.inv_kinematics.setRow(i * 2 + 1, 0, 0, 1, -y, +x);
			}
			this.fwd_kinematics = this.inv_kinematics.pseudoInverse();
		}


		public SwerveModuleVector[] toModuleVectors(ChassisVectors state) {

			final SwerveModuleState[] first_order = super.toSwerveModuleStates(state);
			final SimpleMatrix
				acc_vec = new SimpleMatrix(4, 1),
				transform_mat = new SimpleMatrix(this.SIZE * 2, 2);

			acc_vec.setColumn(0, 0,
				state.axMetersPerSec2,
				state.ayMetersPerSec2,
				state.omegaRadiansPerSecond * state.omegaRadiansPerSecond,
				state.alphaRadPerSec2);

			for(int i = 0; i < this.SIZE; i++) {
				final double sin = first_order[i].angle.getSin(), cos = first_order[i].angle.getCos();
				transform_mat.setRow(i * 2 + 0, 0, +cos, +sin);
				transform_mat.setRow(i * 2 + 1, 0, -sin, +cos);
			}

			final SimpleMatrix second_order = this.inv_kinematics.mult(acc_vec).mult(transform_mat);

			final SwerveModuleVector[] modules = new SwerveModuleVector[this.SIZE];
			for(int i = 0; i < this.SIZE; i++) {
				modules[i] = new SwerveModuleVector(
					first_order[i],
					second_order.get(i * 2 + 0, 0),
					second_order.get(i * 2 + 1, 0) / first_order[i].speedMetersPerSecond
				);
			}

			return modules;

		}


	}




	public static class SwerveVisualization {

		public final Translation3d[] MODULE_LOCATIONS_3D;	// within the robot coord system

		public SwerveVisualization(Translation3d... locations) { this.MODULE_LOCATIONS_3D = locations; }
		public SwerveVisualization(Translation2d... locations) { this(0, locations); }
		public SwerveVisualization(double z, Translation2d... locations) {
			this.MODULE_LOCATIONS_3D = new Translation3d[locations.length];
			int i = 0;
			for(Translation2d t : locations) {
				this.MODULE_LOCATIONS_3D[i] = new Translation3d(t.getX(), t.getY(), z);
				i++;
			}
		}


		public <T> Pose3d[] getWheelPoses3d(Function<T, Rotation2d> extractor_f, T... states) {
			final Pose3d[] poses = new Pose3d[states.length];
			for(int i = 0; (i < states.length && i < MODULE_LOCATIONS_3D.length); i++) {
				poses[i] = new Pose3d(
					MODULE_LOCATIONS_3D[i],
					new Rotation3d(0, 0, extractor_f.apply(states[i]).getRadians())
				);
			}
			return poses;
		}
		public Pose3d[] getWheelPoses3d(SwerveModuleState... states) {
			return this.getWheelPoses3d(
				(SwerveModuleState s)->{ return s.angle; },
				states
			);
		}
		public Pose3d[] getWheelPoses3d(SwerveModulePosition... states) {
			return this.getWheelPoses3d(
				(SwerveModulePosition s)->{ return s.angle; },
				states
			);
		}
		public Pose3d[] getWheelPoses3d(Rotation2d... states) {
			return this.getWheelPoses3d(
				(Rotation2d a)->{ return a; },
				states
			);
		}


		/* For the 'Swerve' AdvantageScope tab -- 2d vector representation */
		public static double[] getComponents2d(SwerveModuleState... states) {
			final double[] data = new double[states.length * 2];
			for(int i = 0; i < states.length; i++) {
				data[i * 2 + 0] = states[i].angle.getRadians();
				data[i * 2 + 1] = states[i].speedMetersPerSecond;
			}
			return data;
		}

	}


}
