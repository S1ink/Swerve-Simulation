package frc.robot;

import edu.wpi.first.math.geometry.*;


public final class SwerveKinematics {

	public static final double
		WHEEL_TO_CENTER_ORTHOGANAL = 0.263525,	// meters
		FRAME_OUTER_WIDTH = 0.660400,
		WHEEL_TREAD_WIDTH = 0.038100,
		WHEEL_DIAMETER = 0.09652;

	public static final Translation3d[]
		WHEEL_CORNER_LOCATIONS = new Translation3d[]{
			new Translation3d(WHEEL_TO_CENTER_ORTHOGANAL, -WHEEL_TO_CENTER_ORTHOGANAL, 0),
			new Translation3d(-WHEEL_TO_CENTER_ORTHOGANAL, -WHEEL_TO_CENTER_ORTHOGANAL, 0),
			new Translation3d(-WHEEL_TO_CENTER_ORTHOGANAL, WHEEL_TO_CENTER_ORTHOGANAL, 0),
			new Translation3d(WHEEL_TO_CENTER_ORTHOGANAL, WHEEL_TO_CENTER_ORTHOGANAL, 0)
		};

	public static enum CornerIdx {
		FRONT_LEFT		(3),
		FRONT_RIGHT		(0),
		BACK_LEFT		(2),
		BACK_RIGHT		(1);

		public final int value;

		private CornerIdx(int v) { this.value = v; }
	}

	// public static final Translation3d front_left = WHEEL_CORNER_LOCATIONS[CornerIdx.FRONT_LEFT.value]; --> example usage of enum

	public static Pose3d[] getWheelPoses3d(Pose2d robot, Rotation2d[] wheel_rotations) {
		final Pose3d[] poses = new Pose3d[4];
		final Pose3d center = new Pose3d(robot);
		// check parameters

		for(int i = 0; i < 4; i++) {
			Translation3d rotated = WHEEL_CORNER_LOCATIONS[i].rotateBy(center.getRotation());
			poses[i] = new Pose3d(rotated,
				new Rotation3d(0.0, 0.0,
					wheel_rotations[i].getRadians() + robot.getRotation().getRadians()));
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


}
