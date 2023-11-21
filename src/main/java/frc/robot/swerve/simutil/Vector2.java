package frc.robot.swerve.simutil;

import edu.wpi.first.math.*;
import edu.wpi.first.math.numbers.*;
import edu.wpi.first.math.geometry.*;


public class Vector2 extends Vector<N2> {

	/** Create a <0, 0> vector. */
	public Vector2()
		{ this(0, 0); }
	/** Create a vector from the provided components. */
	public Vector2(double x, double y) {
		super(Nat.N2());
		this.set(x, y);
	}
	/** Create a vector from the components within a Translation2d. */
	public Vector2(Translation2d v) {
		this(v.getX(), v.getY());
	}
	/** Create a vector from the components within a compatible Matrix. */
	public Vector2(Matrix<N2, N1> mat) {
		super(mat);
	}

	/** Access the x component of the vector. */
	public double x() {
		return this.get(0, 0);
	}
	/** Access the y component of the vector. */
	public double y() {
		return this.get(1, 0);
	}
	/** Set the components of the vector and return the instance for chaining. */
	public Vector2 set(double x, double y) {
		this.set(0, 0, x);
		this.set(1, 0, y);
		return this;
	}
	/** Copy the components of another vector. */
	public Vector2 set(Vector2 v) {
		return this.set(v.x(), v.y());
	}
	/** Copy the components of a compatible Matrix. */
	public Vector2 set(Matrix<N2, N1> m) {
		return this.set(m.get(0, 0), m.get(1, 0));
	}
	/** Reset the vector to be <0, 0>. */
	public Vector2 clear() {
		return this.set(0.0, 0.0);
	}

	/** Negate both components of the vector. */
	public Vector2 negate() {
		this.set(-x(), -y());
		return this;
	}
	/** Add a vector to this instance. */
	public Vector2 append(Vector2 v) {
		this.set(x() + v.x(), y() + v.y());
		return this;
	}
	/** Subtract a vector from this instance. */
	public Vector2 sub(Vector2 v) {
		this.set(x() - v.x(), y() - v.y());
		return this;
	}
	/** Convert this instance to be of unit length. */
	public Vector2 normalize() {
		final double n = this.length();
		this.set(x() / n, y() / n);
		return this;
	}
	/** Divide this instance's components by a scalar. */
	public Vector2 div(double v) {
		this.set(x() / v, y() / v);
		return this;
	}
	/** Multiply this instance's components by a scalar. */
	public Vector2 times(double v) {
		this.set(x() * v, y() * v);
		return this;
	}

	/** Rotate this instance (radians). */
	public Vector2 rotate(double radians) {
		final double
			sin = Math.sin(radians),
			cos = Math.cos(radians);
		this.set(
			this.x() * cos - this.y() * sin,
			this.x() * sin + this.y() * cos
		);
		return this;
	}
	/** Apply a transform to this instance given a transformation matrix. */
	public Vector2 transform(Matrix<N2, N2> rmat) {
		this.set(rmat.times(this));
		return this;
	}
	/** Compute the dot product between this instance and another vector. */
	public double dot(Vector2 v) {
		return this.x() * v.x() + this.y() * v.y();
	}
	/** Get the z-projection for the (3d) cross product of this instance with another vector. */
	public double cross(Vector2 v) {
		return (this.x() * v.y()) - (this.y() * v.x());
	}
	/** Set this instance to the cross product of itself and a z-projection vector (passed as a scalar). */
	public Vector2 cross(double z_projection) {		// takes the cross product between the current vector and a z-projected vector -- thus rotating the current vector about z by 90 degrees (CCW+)
		this.set(-z_projection * this.y(), z_projection * this.x());
		return this;
	}
	/** Set this instance to the cross product of itself and a z-projection vector in a CW+ coordinate system. */
	public Vector2 cross_CW(double z_projection) {	// the opposite of the normal CCW+ cross (CW+) -- this is the same as applying the above cross product backwards
		this.set(z_projection * this.y(), -z_projection * this.x());
		return this;
	}
	/** Get the magnitude of the vector. */
	public double length() {
		return Math.sqrt(this.dot(this));
	}
	/** Get the sine of the angle between this instance and another vector. */
	public double sin(Vector2 v) {
		return this.cross(v) / (this.length() * v.length());
	}
	/** Get the cosine of the angle between this instance and another vector. */
	public double cos(Vector2 v) {
		return this.dot(v) / (this.length() * v.length());
	}
	/** Get the angle of this instance compared to the coordinate system implied by it's components (the x-axis, CCW+). */
	public double theta() {
		return Math.atan2(y(), x());
	}


	/** Static functionality (sometimes this syntax is more explainatory than just instance operations) */

	/** Sum two vectors into a newly allocated instance. */
	public static Vector2 add(Vector2 a, Vector2 b) {
		return new Vector2(a.x() + b.x(), a.y() + b.y());
	}
	/** Subtract two vectors into a newly allocated instance. */
	public static Vector2 sub(Vector2 a, Vector2 b) {
		return new Vector2(a.x() - b.x(), a.y() - b.y());
	}
	/** Scale a vector and emplace into a new instance. */
	public static Vector2 mult(Vector2 a, double s) {
		return new Vector2(a.x() * s, a.y() * s);
	}
	/** Create a new instance from a magnitude and offset angle (CCW+, from the x-axis). */
	public static Vector2 fromPolar(double mag, double radians) {
		return new Vector2(mag * Math.cos(radians), mag * Math.sin(radians));
	}
	/** Rotate a vector and emplace into a new instance. */
	public static Vector2 rotate(Vector2 v, double radians) {
		return new Vector2(v).rotate(radians);
	}
	/** Translate a vector and emplace into a new instance. */
	public static Vector2 transform(Vector2 v, Matrix<N2, N2> t) {
		return new Vector2(v).transform(t);
	}
	/** Compute the dot product between two vectors. */
	public static double dot(Vector2 a, Vector2 b) {
		return a.dot(b);
	}
	/** Compute the z-projection for the (3d) cross product between two vectors. */
	public static double cross(Vector2 a, Vector2 b) {
		return a.cross(b);
	}
	/** Compute the cross product between a vector and a 3d z-projected vector, emplaced into a newly allocated instance. */
	public static Vector2 cross(double z_projection, Vector2 v) {
		return new Vector2(v).cross(z_projection);
	}
	/** Compute the cross product between a 3d z-projected vector and a vector, emplaced into a newly allocated instance. */
	public static Vector2 cross(Vector2 v, double z_projection) {
		return new Vector2(v).cross_CW(z_projection);
	}
	/** Compute the cross product between a 3d z-projected vector and a vector, but scaled by the inverse of the magnitude of the 2d vector (Torque at a position calculation). */
	public static Vector2 invCross(double z, Vector2 r) {	// perform a cross product but the result is equal to |tq|sin(theta)/|r| not |tq||r|sin(theta) -- used to find a resultant force vector from a given torque and applicant radius vector
		return new Vector2(r).cross(z).div(Vector2.dot(r, r));
	}
	/** Get a newly allocated vector of unit length for the provided one. */
	public static Vector2 unitVec(Vector2 v) {
		return new Vector2(v).normalize();
	}
	/** Get the sine of the angle between two vectors. */
	public static double sin(Vector2 a, Vector2 b) {
		return a.sin(b);
	}
	/** Get the cosine of the angle between two vectors. */
	public static double cos(Vector2 a, Vector2 b) {
		return a.cos(b);
	}
	/** Get the component of a vector in a given direction. This is the same as rotating the vector by the negative angle and taking the x-component. */
	public static double angleComponent(Vector2 v, double angle) {
		return v.x() * Math.cos(-angle) - v.y() * Math.sin(-angle);
	}
	/** Calculate the net force given the active and friction force vectors along with a momentum vector. */
	public static Vector2 applyFriction(Vector2 f_app, Vector2 p_sys, Vector2 f_frict, double dt) {
		final double
			offset = f_app.theta(),
			f_app_mag = f_app.length(),
			sin = Math.sin(-offset),
			cos = Math.cos(-offset),
			p_para = p_sys.x() * cos - p_sys.y() * sin,
			p_poip = p_sys.x() * sin + p_sys.y() * cos,
			fr_para = f_frict.x() * cos - f_frict.y() * sin,
			fr_poip = f_frict.x() * sin + f_frict.y() * cos,
			net_para = FrictionModel.applyFriction(f_app_mag, p_para, fr_para, dt),
			net_poip = FrictionModel.applyFriction(0.0, p_poip, fr_poip, dt);
		return new Vector2(net_para, net_poip).rotate(offset);
	}

	/** Create a 2d rotation matrix for rotating by the set amount of radians. */
	public static Matrix<N2, N2> rmat(double radians) {
		final double
			sin = Math.sin(radians),
			cos = Math.cos(radians);
		return Matrix.mat(Nat.N2(), Nat.N2()).fill(
			+cos, -sin,
			+sin, +cos
		);
	}


}