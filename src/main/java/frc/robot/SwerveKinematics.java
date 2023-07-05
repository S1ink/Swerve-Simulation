package frc.robot;

import java.util.function.Function;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.*;


public final class SwerveKinematics {

	public static final double
		WHEEL_TO_CENTER_ORTHOGANAL = 0.263525,	// meters
		FRAME_OUTER_WIDTH = 0.660400,
		WHEEL_TREAD_WIDTH = 0.038100,
		WHEEL_DIAMETER = 0.09652;

	public static final Translation2d[]
		WHEEL_CORNER_LOCATIONS_2D = new Translation2d[]{
			new Translation2d(WHEEL_TO_CENTER_ORTHOGANAL, -WHEEL_TO_CENTER_ORTHOGANAL),
			new Translation2d(-WHEEL_TO_CENTER_ORTHOGANAL, -WHEEL_TO_CENTER_ORTHOGANAL),
			new Translation2d(-WHEEL_TO_CENTER_ORTHOGANAL, WHEEL_TO_CENTER_ORTHOGANAL),
			new Translation2d(WHEEL_TO_CENTER_ORTHOGANAL, WHEEL_TO_CENTER_ORTHOGANAL)
		};

	public static final Translation3d[]
		WHEEL_CORNER_LOCATIONS = new Translation3d[]{
			new Translation3d(WHEEL_TO_CENTER_ORTHOGANAL, -WHEEL_TO_CENTER_ORTHOGANAL, 0),
			new Translation3d(-WHEEL_TO_CENTER_ORTHOGANAL, -WHEEL_TO_CENTER_ORTHOGANAL, 0),
			new Translation3d(-WHEEL_TO_CENTER_ORTHOGANAL, WHEEL_TO_CENTER_ORTHOGANAL, 0),
			new Translation3d(WHEEL_TO_CENTER_ORTHOGANAL, WHEEL_TO_CENTER_ORTHOGANAL, 0)
		};

	public static final SwerveDriveKinematics KINEMATICS = new SwerveDriveKinematics(WHEEL_CORNER_LOCATIONS_2D);

	public static enum CornerIdx {
		FRONT_LEFT		(3),
		FRONT_RIGHT		(0),
		BACK_LEFT		(2),
		BACK_RIGHT		(1);

		public final int value;

		private CornerIdx(int v) { this.value = v; }
	}

	// public static final Translation3d front_left = WHEEL_CORNER_LOCATIONS[CornerIdx.FRONT_LEFT.value]; --> example usage of enum

	public static Pose3d[] getWheelPoses3d(Pose2d robot, SwerveModuleState[] wheel_states) {
		final Pose3d[] poses = new Pose3d[4];
		// check parameters

		for(int i = 0; i < 4; i++) {
			poses[i] = new Pose3d(
				WHEEL_CORNER_LOCATIONS[i],
				new Rotation3d(0.0, 0.0, wheel_states[i].angle.getRadians())
			);
		}

		return poses;
	}

	public static double[] toComponentData(Pose3d[] poses) {
		final double[] values = new double[poses.length * 7];
		for(int i = 0; i < poses.length; i++) {
			int offset = i * 7;
			Quaternion q = poses[i].getRotation().getQuaternion();
			values[offset + 0] = poses[i].getX();
			values[offset + 1] = poses[i].getY();
			values[offset + 2] = poses[i].getZ();
			values[offset + 3] = q.getW();
			values[offset + 4] = q.getX();
			values[offset + 5] = q.getY();
			values[offset + 6] = q.getZ();
		}
		return values;
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
