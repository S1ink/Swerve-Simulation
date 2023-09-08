package frc.robot.swerve;

import java.util.function.Function;

import org.ejml.simple.SimpleMatrix;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.*;


/** SwerveUtils holds many utility classes used for swerve control/simulation */
public final class SwerveUtils {

	/** ChassisStates represents the robot's 1st and 2nd order movement in the robot coordinate system (by default). */
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

		/** Convert from a movement in the field coordinate system to one in the Robot's coordinate system given the robot's heading. */
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
		/** Convert from a movement in the field coordinate system to one in the Robot's coordinate system given the robot's heading. */
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

		/** Populate the ChassisStates' second order properties using deltas between 2 of the 1st order properties and a delta time. */
		public static ChassisStates accFromDelta(ChassisStates from, ChassisStates to, double dt, ChassisStates buff) {
			if(buff == null) { buff = new ChassisStates(); }
			buff.x_velocity = to.x_velocity;
			buff.y_velocity = to.y_velocity;
			buff.angular_velocity = to.angular_velocity;
			buff.x_acceleration = (to.x_velocity - from.x_velocity) / dt;
			buff.y_acceleration = (to.y_velocity - from.y_velocity) / dt;
			buff.angular_acceleration = (to.angular_velocity - from.angular_acceleration) / dt;
			return buff;
		}

	}



	/** SwerveModuleStates has a bunch of information about the module's states
	 * (not geographical states) -- can be in robot or field coordinate system depending on the context. */
	public static class SwerveModuleStates {

		public Rotation2d
			angle;
		public double
			linear_displacement,
			linear_velocity,
			angular_velocity,
			linear_acceleration;

		public SwerveModuleStates()
			{ this.angle = new Rotation2d(); }
		public SwerveModuleStates(double angle_rad, double linear_pos, double linear_vel, double angular_vel, double linear_acc)
			{ this(Rotation2d.fromRadians(angle_rad), linear_pos, linear_vel, angular_vel, linear_acc); }
		public SwerveModuleStates(Rotation2d angle, double linear_pos, double linear_vel, double angular_vel, double linear_acc) {
			this.angle = angle;
			this.linear_displacement = linear_pos;
			this.linear_velocity = linear_vel;
			this.angular_velocity = angular_vel;
			this.linear_acceleration = linear_acc;
		}

		public static SwerveModuleStates makePosition(double angle_rad, double linear_pos) {
			return makePosition(Rotation2d.fromRadians(angle_rad), linear_pos);
		}
		public static SwerveModuleStates makePosition(Rotation2d angle, double linear_pos) {
			return new SwerveModuleStates(angle, linear_pos, 0.0, 0.0, 0.0);
		}
		public static SwerveModuleStates makeVelocity(double angle_rad, double linear_vel) {
			return makeVelocity(Rotation2d.fromRadians(angle_rad), linear_vel);
		}
		public static SwerveModuleStates makeVelocity(Rotation2d angle, double linear_vel) {
			return new SwerveModuleStates(angle, 0.0, linear_vel, 0.0, 0.0);
		}
		public static SwerveModuleStates makeSecondOrder(double angle_rad, double linear_vel, double angular_vel, double linear_acc) {
			return makeSecondOrder(Rotation2d.fromRadians(angle_rad), linear_vel, angular_vel, linear_acc);
		}
		public static SwerveModuleStates makeSecondOrder(Rotation2d angle, double linear_vel, double angular_vel, double linear_acc) {
			return new SwerveModuleStates(angle, 0.0, linear_vel, angular_vel, linear_vel);
		}
		public static SwerveModuleStates makeFrom(SwerveModuleState state) {
			return new SwerveModuleStates(state.angle, 0.0, state.speedMetersPerSecond, 0.0, 0.0);
		}
		public static SwerveModuleStates makeFrom(SwerveModulePosition position) {
			return new SwerveModuleStates(position.angle, position.distanceMeters, 0.0, 0.0, 0.0);
		}

		public SwerveModulePosition toPosition() {
			return new SwerveModulePosition(this.linear_displacement, this.angle);
		}
		public SwerveModuleState toVelocityState() {
			return new SwerveModuleState(this.linear_velocity, this.angle);
		}

		// >> utilities for converting/acting on arrays of states <<


		/** Optimizes the target state in case the module is attempting to turn more than 90 degrees - in this case we can just flip the direction and turn to a less extreme angle. */
		public static SwerveModuleStates optimize(SwerveModuleStates desired_states, Rotation2d currentAngle) {
			var delta = desired_states.angle.minus(currentAngle);
			if (Math.abs(delta.getDegrees()) > 90.0) {
				return SwerveModuleStates.makeSecondOrder(
					desired_states.angle.rotateBy(Rotation2d.fromDegrees(180.0)),
					-desired_states.linear_velocity,
					desired_states.angular_velocity,
					desired_states.linear_acceleration
				);
			} else {
				return SwerveModuleStates.makeSecondOrder(
					desired_states.angle,
					desired_states.linear_velocity,
					desired_states.angular_velocity,
					desired_states.linear_acceleration
				);
			}
		}

	}







	/** Allows for easy data conversion/management for viewing a swerve model in 3D using AdvantageScope. */
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
			final int len = Math.min(states.length, this.MODULE_LOCATIONS_3D.length);
			final Pose3d[] poses = new Pose3d[len];
			for(int i = 0; i < len; i++) {
				poses[i] = new Pose3d(
					this.MODULE_LOCATIONS_3D[i],
					new Rotation3d(0, 0, extractor_f.apply(states[i]).getRadians())
				);
			}
			return poses;
		}
		public Pose3d[] getWheelPoses3d(SwerveModuleStates... states) {
			return this.getWheelPoses3d(
				(SwerveModuleStates ss)->{ return ss.angle; },
				states
			);
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
		public Pose3d[] getWheelPoses3d(double... rotations) {
			final int len = Math.min(rotations.length, this.MODULE_LOCATIONS_3D.length);
			final Pose3d[] poses = new Pose3d[len];
			for(int i = 0; i < len; i++) {
				poses[i] = new Pose3d(
					this.MODULE_LOCATIONS_3D[i],
					new Rotation3d(0, 0, rotations[i])
				);
			}
			return poses;
		}


		/* For the 'Swerve' AdvantageScope tab -- 2d vector representation */
		public static double[] getVecComponents2d(SwerveModuleState... states) {
			final double[] data = new double[states.length * 2];
			for(int i = 0; i < states.length; i++) {
				data[i * 2 + 0] = states[i].angle.getRadians();
				data[i * 2 + 1] = states[i].speedMetersPerSecond;
			}
			return data;
		}
		public static double[] getVecComponents2d(SwerveModuleStates... states) {
			final double[] data = new double[states.length * 2];
			for(int i = 0; i < states.length; i++) {
				data[i * 2 + 0] = states[i].angle.getRadians();
				data[i * 2 + 1] = states[i].linear_velocity;
			}
			return data;
		}

	}




	/** Generate the module locations for a robot chassis that is perfectly square, given the orthoganl distance from a module (either x or y) to the robot's center. */
	public static Translation2d[] makeSquareLocationsCW(double ortho_center_dist) {
		final double d = ortho_center_dist;
		return new Translation2d[]{
			new Translation2d(+d, -d),
			new Translation2d(-d, -d),
			new Translation2d(-d, +d),
			new Translation2d(+d, +d)
		};
	}
	/** Generate the module locations for a robot chassis that is a rectangle given the module-to-center distances along the width and length of the robot's frame. */
	public static Translation2d[] makeRectLocationsCW(double wwidth, double wlength) {
		final double w = wwidth, l = wlength;
		return new Translation2d[] {
			new Translation2d(+w, -l),
			new Translation2d(-w, -l),
			new Translation2d(-w, +l),
			new Translation2d(+w, +l)
		};
	}


}
