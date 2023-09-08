package frc.robot.swerve.simutil;

import edu.wpi.first.math.*;
import edu.wpi.first.math.numbers.*;
import edu.wpi.first.math.geometry.*;


public class Vector2 extends Vector<N2> {

	public Vector2()
		{ this(0, 0); }
	public Vector2(double x, double y) {
		super(Nat.N2());
		this.set(x, y);
	}
	public Vector2(Translation2d v) {
		this(v.getX(), v.getY());
	}
	public Vector2(Matrix<N2, N1> mat) {
		super(mat);
	}

	public double x() {
		return this.get(0, 0);
	}
	public double y() {
		return this.get(1, 0);
	}
	public Vector2 set(double x, double y) {
		this.set(0, 0, x);
		this.set(1, 0, y);
		return this;
	}
	public Vector2 set(Matrix<N2, N1> m) {
		super.m_storage.setTo(m.getStorage());
		return this;
	}

	public Vector2 negate() {
		this.set(-x(), -y());
		return this;
	}
	public Vector2 append(Vector2 v) {
		this.set(x() + v.x(), y() + v.y());
		return this;
	}
	public Vector2 sub(Vector2 v) {
		this.set(x() - v.x(), y() - v.y());
		return this;
	}
	public Vector2 normalize() {
		final double n = this.length();
		this.set(x() / n, y() / n);
		return this;
	}
	public Vector2 div(double v) {
		this.set(x() / v, y() / v);
		return this;
	}
	public Vector2 times(double v) {
		this.set(x() * v, y() * v);
		return this;
	}

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
	public Vector2 transform(Matrix<N2, N2> rmat) {
		this.set(rmat.times(this));
		return this;
	}
	public double dot(Vector2 v) {
		return this.x() * v.x() + this.y() * v.y();
	}
	public double cross(Vector2 v) {
		return (this.x() * v.y()) - (this.y() * v.x());
	}
	public Vector2 cross(double z_projection) {		// takes the cross product between the current vector and a z-projected vector -- thus rotating the current vector about z by 90 degrees (CCW+)
		this.set(-z_projection * this.y(), z_projection * this.x());
		return this;
	}
	public Vector2 cross_CW(double z_projection) {	// the opposite of the normal CCW+ cross (CW+) -- this is the same as applying the above cross product backwards
		this.set(z_projection * this.y(), -z_projection * this.x());
		return this;
	}
	public double length() {
		return Math.sqrt(this.dot(this));
	}
	public double sin(Vector2 v) {
		return this.cross(v) / (this.length() * v.length());
	}
	public double cos(Vector2 v) {
		return this.dot(v) / (this.length() * v.length());
	}
	public double theta() {
		return Math.atan2(y(), x());
	}


	/** Static functionality (sometimes this syntax is more explainatory than just instance operations) */

	public static Vector2 add(Vector2 a, Vector2 b) {
		return new Vector2(a.x() + b.x(), a.y() + b.y());
	}
	public static Vector2 sub(Vector2 a, Vector2 b) {
		return new Vector2(a.x() - b.x(), a.y() - b.y());
	}
	public static Vector2 mult(Vector2 a, double s) {
		return new Vector2(a.x() * s, a.y() * s);
	}
	public static Vector2 fromPolar(double mag, double radians) {
		return new Vector2(mag * Math.cos(radians), mag * Math.sin(radians));
	}
	public static Vector2 rotate(Vector2 v, double radians) {
		return new Vector2(v).rotate(radians);
	}
	public static Vector2 transform(Vector2 v, Matrix<N2, N2> t) {
		return new Vector2(v).transform(t);
	}
	public static double dot(Vector2 a, Vector2 b) {
		return a.dot(b);
	}
	public static double cross(Vector2 a, Vector2 b) {
		return a.cross(b);
	}
	public static Vector2 cross(double z_projection, Vector2 v) {
		return new Vector2(v).cross(z_projection);
	}
	public static Vector2 cross(Vector2 v, double z_projection) {
		return new Vector2(v).cross_CW(z_projection);
	}
	public static Vector2 invCross(double z, Vector2 r) {	// perform a cross product but the result is equal to |tq|sin(theta)/|r| not |tq||r|sin(theta) -- used to find a resultant force vector from a given torque and applicant radius vector
		return new Vector2(r).cross(z).div(Vector2.dot(r, r));
	}
	public static Vector2 unitVec(Vector2 v) {
		return new Vector2(v).normalize();
	}
	public static double sin(Vector2 a, Vector2 b) {
		return a.sin(b);
	}
	public static double cos(Vector2 a, Vector2 b) {
		return a.cos(b);
	}
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