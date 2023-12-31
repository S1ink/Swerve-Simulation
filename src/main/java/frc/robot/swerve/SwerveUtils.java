package frc.robot.swerve;

import java.util.function.Function;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.*;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;


/** SwerveUtils holds many utility classes used for swerve control/simulation */
public final class SwerveUtils {

	/** A static getter for a property of a specific module type. */
	public static interface ModulePropertySupplier<Module_T extends SwerveModule, T> {
		public T get(Module_T module);
	}
	/** A static setter for a property of a specific module type. */
	public static interface ModulePropertyConsumer<Module_T extends SwerveModule, T> {
		public void set(Module_T module, T value);
	}
	/** A static getter for a double property of a specific module type. */
	public static interface ModuleDoubleSupplier<Module_T extends SwerveModule> {
		public double get(Module_T module);
	}
	/** A static setter for a double property of a specific module type. */
	public static interface ModuleDoubleConsumer<Module_T extends SwerveModule> {
		public void set(Module_T module, double value);
	}

	/** ChassisStates represents the robot's 1st and 2nd order movement in the robot coordinate system (by default). */
	public static class ChassisStates implements Sendable {
		
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
		public ChassisStates(ChassisSpeeds speeds) {
			this(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond, speeds.omegaRadiansPerSecond);
		}
		public ChassisStates(double x_v, double y_v, double r_v, double x_a, double y_a, double r_a) {
			this.x_velocity = x_v;
			this.y_velocity = y_v;
			this.angular_velocity = r_v;
			this.x_acceleration = x_a;
			this.y_acceleration = y_a;
			this.angular_acceleration = r_a;
		}
		public ChassisStates(ChassisStates other) {
			this.x_velocity = other.x_velocity;
			this.y_velocity = other.y_velocity;
			this.angular_velocity = other.angular_velocity;
			this.x_acceleration = other.x_acceleration;
			this.y_acceleration = other.y_acceleration;
			this.angular_acceleration = other.angular_acceleration;
		}


		public void zero() {
			this.x_velocity
				= this.y_velocity
				= this.angular_velocity
				= this.x_acceleration
				= this.y_acceleration
				= this.angular_acceleration
				= 0;
		}
		public boolean isStopped() {
			return (
				this.x_velocity == 0 &&
				this.y_velocity == 0 &&
				this.angular_velocity == 0 &&
				this.x_acceleration == 0 &&
				this.y_acceleration == 0 &&
				this.angular_acceleration == 0
			);
		}

		public ChassisStates rotate(double radians) {
			final double
				sin = Math.sin(radians),
				cos = Math.cos(radians);
			this.x_velocity = this.x_velocity * cos - this.y_velocity * sin;
			this.y_velocity = this.x_velocity * sin + this.y_velocity * cos;
			this.x_acceleration = this.x_acceleration * cos - this.y_acceleration * sin;
			this.y_acceleration = this.x_acceleration * sin + this.y_acceleration * cos;
			return this;
		}
		public ChassisStates toFieldRelative(double robot_heading_rad) {
			return this.rotate(robot_heading_rad);
		}
		public ChassisStates fromFieldRelative(double robot_heading_rad) {
			return this.rotate(-robot_heading_rad);
		}

		public Twist2d integrate(double dt) {
			return this.integrate(dt, null);
		}
		/** Create a twist of the chassis' path taken during the provided time interval */
		public Twist2d integrate(double dt, Twist2d buff) {
			if(buff == null) { buff = new Twist2d(); }
			buff.dx = this.x_velocity * dt;
			buff.dy = this.y_velocity * dt;
			buff.dtheta = this.angular_velocity * dt;
			return buff;
		}
		/** See the static method descretizeCurvature() */
		public void descretizeVelocities(double dt) {
			ChassisStates.descretizeCurvature(this, dt);
		}

		/** Treat the integrated pose from the chassis' velocities as a target and work backwards to find the correct "constant curvature" velocities which should be used */
		public static void descretizeCurvature(ChassisStates states, double dt) {
			final Pose2d delta = new Pose2d(
				states.x_velocity * dt,
				states.y_velocity * dt,
				Rotation2d.fromRadians(states.angular_velocity * dt)
			);
			final Twist2d curvature = new Pose2d().log(delta);
			states.x_velocity = curvature.dx / dt;
			states.y_velocity = curvature.dy / dt;
			states.angular_velocity = curvature.dtheta / dt;
		}

		/** Normalize target chassis speeds based on the maximum possible wheel speed and maximum allowable wheel speed */
		public static void normalizeByMaximumV(ChassisStates target, double max_module_radius, double max_module_velocity) {
			final double maxv = Math.hypot(target.x_velocity, target.y_velocity) + (max_module_radius * target.angular_velocity);
			if(maxv > max_module_velocity) {
				final double scale = max_module_velocity / maxv;
				target.x_velocity *= scale;
				target.y_velocity *= scale;
				target.angular_velocity *= scale;
			}
		}
		/** Clamp the linear and rotational velocities based on their relative accelerations since the last target (velocity rate limit = acceleration limit) */
		public static void rateLimitVelocities(ChassisStates target, ChassisStates last, double dt,
			double linear_acc_limit, double rotational_acc_limit)
		{
			final double
				dvlim = linear_acc_limit * dt,
				drlim = rotational_acc_limit * dt,
				dvx = target.x_velocity - last.x_velocity,
				dvy = target.y_velocity - last.y_velocity,
				dvr = target.angular_velocity - last.angular_velocity,
				dv = Math.hypot(dvx, dvy),
				_dv = MathUtil.clamp(dv, -dvlim, dvlim),
				_dr = MathUtil.clamp(dvr, -drlim, drlim),
				scale = dv == 0.0 ? 1.0 : _dv / dv,		// protected against div by 0 when target vel is (0, 0)
				_dvx = dvx * scale,
				_dvy = dvy * scale;
			target.x_velocity = last.x_velocity + _dvx;
			target.y_velocity = last.y_velocity + _dvy;
			target.angular_velocity = last.angular_velocity + _dr;
		}

		/** Convert from a movement in the field coordinate system to one in the Robot's coordinate system given the robot's heading. */
		public static ChassisStates fromFieldRelative(
			double x_v, double y_v, double r_v, double x_a, double y_a, double r_a,
			double robot_heading_rad
		) {
			return new ChassisStates( x_v, x_v, r_v, x_a, x_a, r_a ).fromFieldRelative(robot_heading_rad);
		}
		/** Convert from a movement in the field coordinate system to one in the Robot's coordinate system given the robot's heading. */
		public static ChassisStates fromFieldRelative(ChassisStates states, double robot_heading_rad) {
			return new ChassisStates( states ).fromFieldRelative(robot_heading_rad);
		}

		public static ChassisStates rotate(ChassisStates states, double radians) {
			return new ChassisStates( states ).rotate(radians);
		}
		public static ChassisStates rotate(
			double x_v, double y_v, double r_v, double x_a, double y_a, double r_a,
			double radians
		) {
			return new ChassisStates( x_v, x_v, r_v, x_a, x_a, r_a ).rotate(radians);
		}

		/** Populate the ChassisStates' second order properties using deltas between 2 of the 1st order properties and a delta time. */
		public static ChassisStates accFromDelta(ChassisStates from, ChassisStates to, double dt, ChassisStates buff) {
			if(buff == null) { buff = new ChassisStates(); }
			buff.x_acceleration = (to.x_velocity - from.x_velocity) / dt;
			buff.y_acceleration = (to.y_velocity - from.y_velocity) / dt;
			buff.angular_acceleration = (to.angular_velocity - from.angular_velocity) / dt;
			buff.x_velocity = to.x_velocity;
			buff.y_velocity = to.y_velocity;
			buff.angular_velocity = to.angular_velocity;
			return buff;
		}


		@Override
		public void initSendable(SendableBuilder b) {
			b.addDoubleProperty("X Velocity", ()->this.x_velocity, null);
			b.addDoubleProperty("Y Velocity", ()->this.y_velocity, null);
			b.addDoubleProperty("Angular Velocity", ()->this.angular_velocity, null);
			b.addDoubleProperty("X Acceleration", ()->this.x_acceleration, null);
			b.addDoubleProperty("Y Acceleration", ()->this.y_acceleration, null);
			b.addDoubleProperty("Angular Acceleration", ()->this.angular_acceleration, null);
		}

	}



	/** SwerveModuleStates has a bunch of information about the module's states
	 * (not geographical states) -- can be in robot or field coordinate system depending on the context. */
	public static class SwerveModuleStates implements Sendable {

		public double
			rotation,
			linear_displacement,
			linear_velocity,
			angular_velocity,
			linear_acceleration;

		public SwerveModuleStates() {}
		public SwerveModuleStates(SwerveModuleStates states)
			{ this.copy(states); }
		public SwerveModuleStates(Rotation2d angle, double linear_pos, double linear_vel, double angular_vel, double linear_acc)
			{ this(angle.getRadians(), linear_pos, linear_vel, angular_vel, linear_acc); }
		public SwerveModuleStates(double angle_rad, double linear_pos, double linear_vel, double angular_vel, double linear_acc) {
			this.rotation = angle_rad;
			this.linear_displacement = linear_pos;
			this.linear_velocity = linear_vel;
			this.angular_velocity = angular_vel;
			this.linear_acceleration = linear_acc;
		}

		public static SwerveModuleStates makePosition(double angle_rad, double linear_pos) {
			return new SwerveModuleStates(angle_rad, linear_pos, 0.0, 0.0, 0.0);
		}
		public static SwerveModuleStates makePosition(Rotation2d angle, double linear_pos) {
			return SwerveModuleStates.makePosition(angle.getRadians(), linear_pos);
		}
		public static SwerveModuleStates makeVelocity(double angle_rad, double linear_vel) {
			return new SwerveModuleStates(angle_rad, 0.0, linear_vel, 0.0, 0.0);
		}
		public static SwerveModuleStates makeVelocity(Rotation2d angle, double linear_vel) {
			return SwerveModuleStates.makeVelocity(angle.getRadians(), linear_vel);
		}
		public static SwerveModuleStates makeSecondOrder(double angle_rad, double linear_vel, double angular_vel, double linear_acc) {
			return new SwerveModuleStates(angle_rad, 0.0, linear_vel, angular_vel, linear_acc);
		}
		public static SwerveModuleStates makeSecondOrder(Rotation2d angle, double linear_vel, double angular_vel, double linear_acc) {
			return SwerveModuleStates.makeSecondOrder(angle.getRadians(), linear_vel, angular_vel, linear_acc);
		}
		public static SwerveModuleStates makeFrom(SwerveModuleState state) {
			return new SwerveModuleStates(state.angle, 0.0, state.speedMetersPerSecond, 0.0, 0.0);
		}
		public static SwerveModuleStates makeFrom(SwerveModulePosition position) {
			return new SwerveModuleStates(position.angle, position.distanceMeters, 0.0, 0.0, 0.0);
		}
		public static SwerveModuleStates makeFrom(SwerveModuleStates states) {
			return new SwerveModuleStates(states);
		}

		public void zero() {
			this.rotation =
			this.linear_displacement =
			this.linear_velocity =
			this.angular_velocity =
			this.linear_acceleration = 0.0;
		}

		public Rotation2d getRotation2d() {
			return Rotation2d.fromRadians(this.rotation);
		}

		public SwerveModulePosition toPosition() {
			return new SwerveModulePosition(this.linear_displacement, this.getRotation2d());
		}
		public SwerveModuleState toVelocityState() {
			return new SwerveModuleState(this.linear_velocity, this.getRotation2d());
		}

		public SwerveModuleStates copy() {								// copy OUT
			return SwerveModuleStates.makeFrom(this);
		}
		public SwerveModuleStates copy(SwerveModuleStates states) {		// copy IN
			this.rotation = states.rotation;
			this.linear_displacement = states.linear_displacement;
			this.linear_velocity = states.linear_velocity;
			this.angular_velocity = states.angular_velocity;
			this.linear_acceleration = states.linear_acceleration;
			return this;
		}


		/** Optimizes the target state in case the module is attempting to turn more than 90 degrees - in this case we can just flip the direction and turn to a less extreme angle. */
		public static SwerveModuleStates optimize(
			SwerveModuleStates target_states, double current_rotation, SwerveModuleStates result)
		{
			if(result == null) { result = target_states.copy(); }
			else if(result != target_states) { result.copy(target_states); }
			if(Math.abs(target_states.rotation - current_rotation) > Math.PI / 2) {
				result.rotation = (target_states.rotation + Math.PI) % (Math.PI * 2);
				result.linear_velocity *= -1;
			}
			return result;
		}
		public static SwerveModuleStates optimize(SwerveModuleStates target_states, double current_rotation) {
			return SwerveModuleStates.optimize(target_states, current_rotation, null);
		}

		public static SwerveModuleStates optimize(
			SwerveModuleStates target_states, SwerveModuleStates current_states, SwerveModuleStates result)
		{
			return SwerveModuleStates.optimize(target_states, current_states.rotation, result);	// temporary. evenually compare momentum as well
		}
		public static SwerveModuleStates optimize(
			SwerveModuleStates target_states, SwerveModuleStates current_states)
		{
			return SwerveModuleStates.optimize(target_states, current_states, null);
		}


		// >> utilities for converting/acting on arrays of states <<


		@Override
		public void initSendable(SendableBuilder b) {
			b.addDoubleProperty("Rotation", ()->this.rotation, null);
			b.addDoubleProperty("Linear Displacement", ()->this.linear_displacement, null);
			b.addDoubleProperty("Linear Velocity", ()->this.linear_velocity, null);
			b.addDoubleProperty("Angular Velocity", ()->this.angular_velocity, null);
			b.addDoubleProperty("Linear Acceleration", ()->this.linear_acceleration, null);
		}


	}



	// public static interface SwerveModuleController {

	// 	public double 

	// }







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
				(SwerveModuleStates ss)->{ return Rotation2d.fromRadians(ss.rotation); },
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
				data[i * 2 + 0] = states[i].rotation;
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
