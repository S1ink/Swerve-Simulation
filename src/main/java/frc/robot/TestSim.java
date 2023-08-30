package frc.robot;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.Timer;

import frc.robot.swerve.*;
import frc.robot.swerve.SwerveUtils.*;
import frc.robot.team3407.Util;


public class TestSim extends CommandBase {

	public static interface FrictionModel {

		/** calculate the maximum applicant friction given the orthogonal force, velocity, and sum independant force */
		public double calc(double f_norm, double vel, double f);

	}

	public static class StribeckFriction implements FrictionModel {

		public final double
			stb_vel,		// reference velocity which coincides with the transition between stiction and coulomb friction
			coulomb_coeff,	// the coefficient of kinetic friciton
			stiction_coeff,	// the coefficient of static friction
			viscous_coeff;	// the coefficient of viscous friction

		public StribeckFriction(double stb_vel, double static_coeff, double kinetic_coeff, double viscous_coeff) {
			this.stb_vel = stb_vel;
			this.stiction_coeff = static_coeff;
			this.coulomb_coeff = kinetic_coeff;
			this.viscous_coeff = viscous_coeff;
		}
		public StribeckFriction(double static_coeff, double kinetic_coeff) {
			this.stb_vel = 0.01;
			this.stiction_coeff = static_coeff;
			this.coulomb_coeff = kinetic_coeff;
			this.viscous_coeff = 0.0;
		}

		@Override
		public double calc(double f_norm, double vel, double f) {
			if(vel == 0.0)
				return stiction_coeff * f_norm * -Math.signum(f);
			final double
				fc = coulomb_coeff * f_norm,
				fs = stiction_coeff * f_norm,
				fv = viscous_coeff * f_norm * -vel,
				s = Util.sgnnz(vel),
				c = 1.0 / (1.0 + Math.pow(vel / stb_vel, 2));
			return (fc + (fs - fc) * c) * -s + fv;
		}


	}
	public static class GearTrainSim {

		/** A Node represents a gear or pulley in a geartrain. */
		private static class Node {
			final double
				inertia,	// rotational -- about the gear's pivot axis
				input_rad,
				output_rad;
			final int
				input_teeth,
				output_teeth;
			final boolean
				invert_next;	// false if pulley

			public Node(double ri, double i_rad, double o_rad, int i_teeth, int o_teeth, boolean invert_next) {
				this.inertia = ri;
				this.input_rad = i_rad;
				this.output_rad = o_rad;
				this.input_teeth = i_teeth;
				this.output_teeth = o_teeth;
				this.invert_next = invert_next;
			}

			public static Node makeGear(double ri, double radius, int teeth) {
				return new Node(ri, radius, radius, teeth, teeth, true);
			}
			public static Node makeDoubleGear(double ri, double input_radius, double output_radius, int input_teeth, int output_teeth) {
				return new Node(ri, input_radius, output_radius, input_teeth, output_teeth, true);
			}
			public static Node makeGearPulley(double ri, double gear_radius, double pulley_radius, int gear_teeth, int pulley_teeth) {
				return new Node(ri, gear_radius, pulley_radius, gear_teeth, pulley_teeth, false);
			}

			/** The FORWARD gear ratio between A and B when A comes before B -- TO INVERT USE 1/ratio(A, B) NOT ratio(B, A)!!! */
			public static double ratio(Node a, Node b) {
				return (double)a.output_teeth / b.input_teeth;
			}
			/** Convert A's angular velocity to B's angular velocity -- inverting A and B WILL NOT produce the reverse velocity calculation >> use avB2avA() */
			public static double avA2avB(Node a, double av, Node b) {
				return av * ratio(a, b);
			}
			/** Convert B's angular velocity to A's angular velocity (when A comes before B) */
			public static double avB2avA(Node a, Node b, double av) {
				return av / ratio(a, b);
			}
			/** Convert the torque on A to the torque on B -- inverting A and B WILL NOT produce the reverse torque calculation >> use tqB2tqA() */
			public static double tqA2tqB(Node a, double tq, Node b) {
				return tq / ratio(a, b);
			}
			/** Conver the torque on B to the torque on A (when A comes before B) */
			public static double tqB2tqA(Node a, Node b, double tq) {
				return tq * ratio(a, b);
			}

			public static int rotDirMod(ArrayList<Node> nodes) {
				int v = 1;
				for(int i = 0; i < nodes.size() - 1; i++) {		// don't access the last node; it's output does not connect to anything
					if(nodes.get(i).invert_next) {
						v *= -1;
					}
				}
				return v;
			}
			public static double sumRatio(ArrayList<Node> nodes) {
				double ratio = 1.0;
				int last_output_teeth = nodes.size() > 0 ? nodes.get(0).input_teeth : 1;
				for(Node n : nodes) {
					ratio *= n.input_teeth / last_output_teeth;
					last_output_teeth = n.output_teeth;
				}
				return ratio;
			}
			public static double sumRatioRev(ArrayList<Node> nodes) {
				return 1.0 / sumRatio(nodes);
			}
			public static double sumInertia(ArrayList<Node> nodes) {
				double sum = 0.0;
				int prev_teeth = 1;
				for(int i = nodes.size() - 1; i >= 0; i--) {
					final double I = nodes.get(i).inertia;
					final int teeth = nodes.get(i).output_teeth;
					sum = I + sum * ((double)(teeth * teeth) / (prev_teeth * prev_teeth));
					prev_teeth = nodes.get(i).input_teeth;
				}
				return sum;
			}
			public static double sumInertiaRev(ArrayList<Node> nodes) {
				double sum = 0.0;
				int prev_teeth = 1;
				for(int i = 0; i < nodes.size(); i++) {
					final double I = nodes.get(i).inertia;
					final int teeth = nodes.get(i).input_teeth;
					sum = I + sum * ((double)(teeth * teeth) / (prev_teeth * prev_teeth));
					prev_teeth = nodes.get(i).output_teeth;
				}
				return sum;
			}
			public static double sumInertiaL(ArrayList<Node> nodes) {
				final double r = nodes.get(0).input_rad;
				return sumInertia(nodes) / (r * r);
			}
			public static double sumInertiaRevL(ArrayList<Node> nodes) {
				final double r = nodes.get(nodes.size() - 1).output_rad;
				return sumInertiaRev(nodes) / (r * r);
			}


		}
		/** LinkedNode allows for Nodes to be connected in a linked list. */
		private static class LinkedNode {

			private final Node self;
			private LinkedNode pre = null, post = null;

			public LinkedNode(Node self) {
				this.self = self;
			}

			public static LinkedNode from(Node n) { return new LinkedNode(n); }

			private void _prepend(LinkedNode pre) { this.pre = pre; }
			private boolean _vpre() { return this.pre != null; }
			private boolean _vpost() { return this.post != null; }

			public LinkedNode append(LinkedNode n) {
				this.post = n;
				n._prepend(this);
				return this;
			}
			public LinkedNode link(Node n) {
				this.append(LinkedNode.from(n));
				return this.post;
			}


			/** The ratio between this node an the next one in the list */
			public double nextRatio() {
				if(_vpost()) {
					return Node.ratio(this.self, this.post.self);
				}
				return 1.0;
			}
			/** The ratio between the previous node and this one */
			public double prevRatio() {
				if(_vpre()) {
					return Node.ratio(this.pre.self, this.self);
				}
				return 1.0;
			}
			/** The inverse of the ratio between the previous node and this one (ratio between this one and the previous, working backwards) */
			public double revRatio() {
				if(_vpre()) {
					return 1.0 / Node.ratio(this.pre.self, this.self);
				}
				return 1.0;
			}

			/** Get the next node's angular velocity given this node's angular velocity */
			public double nextAVel(double avel) {
				if(_vpost()) {
					return Node.avA2avB(this.self, avel, this.post.self);
				}
				return avel;
			}
			/** Get the current node's angular velocity given the next node's angular velocity */
			public double fromNextAVel(double avel) {
				if(_vpost()) {
					return Node.avB2avA(this.self, this.post.self, avel);
				}
				return avel;
			}
			/** Get the current node's angular velocity given the previous node's angular velocity */
			public double fromPrevAVel(double prev_avel) {
				if(_vpre()) {
					return Node.avA2avB(this.pre.self, prev_avel, this.self);
				}
				return prev_avel;
			}
			/** Get the previous node's angular velocity given this node's angular velocity */
			public double prevAVel(double avel) {
				if(_vpre()) {
					return Node.avB2avA(this.pre.self, this.self, avel);
				}
				return avel;
			}

			/** Get the torque applied about the next node given the torque applied on this node */
			public double nextTq(double tq) {
				if(_vpost()) {
					return Node.tqA2tqB(this.self, tq, this.post.self);
				}
				return tq;
			}
			/** Get the torque begin applied on this node given the torque applied on the next node */
			public double fromNextTq(double tq) {
				if(_vpost()) {
					return Node.tqB2tqA(this.self, this.post.self, tq);
				}
				return tq;
			}
			/** Get the torque being applied on this node given the torque applied on the previous node */
			public double fromPrevTq(double prev_tq) {
				if(_vpre()) {
					return Node.tqA2tqB(this.pre.self, prev_tq, this.self);
				}
				return prev_tq;
			}
			/** Get the toruqe applied on the previous node given the torque being applied on this node */
			public double prevTq(double tq) {
				if(_vpre()) {
					return Node.tqB2tqA(this.pre.self, this.self, tq);
				}
				return tq;
			}


			public static double sumChainInertia(LinkedNode beg) {
				LinkedNode end = beg;
				while(end.post != null) {
					end = end.post;
				}
				return _sumChainInertia(end);
			}
			public static double _sumChainInertia(LinkedNode end) {
				double sum = end.self.inertia;
				LinkedNode current = end;
				while((current = current.pre) != null) {
					final double R = current.nextRatio();
					sum = current.self.inertia + sum * (R * R);
				}
				return sum;
			}
			public static double sumChainInertiaRev(LinkedNode beg) {
				double sum = beg.self.inertia;
				LinkedNode current = beg;
				while((current = current.post) != null) {
					final double R = current.revRatio();
					sum = current.self.inertia + sum * (R * R);
				}
				return sum;
			}


		}



		private LinkedNode beg = null, end = null;
		private double
			sum_ratio = 1.0,
			sum_inertia_beg = 0.0,
			sum_inertia_end = 0.0;
		private int length = 0;

		public GearTrainSim(Node... nodes) {
			for(Node n : nodes) {
				this._append(n);
				this._recalc();
			}
		}

		private void _recalc() {
			this.sum_inertia_beg = LinkedNode.sumChainInertia(this.end);
			this.sum_inertia_end = LinkedNode.sumChainInertiaRev(this.beg);
		}
		private void _append(Node n) {
			if(this.beg == null) {
				this.beg = this.end = LinkedNode.from(n);
			} else {
				this.end = this.end.link(n);
				this.sum_ratio *= this.end.prevRatio();
			}
			this.length++;
		}

		public GearTrainSim addGear(double ri, double radius, int teeth) {
			this._append(
				Node.makeGear(ri, radius, teeth) );
			this._recalc();
			return this;
		}
		// only use this when the gear is not the first or last one in the chain
		public GearTrainSim addDualGear(double ri, int input_teeth, int output_teeth) {
			this.addDualGear(ri, 1.0, 1.0, input_teeth, output_teeth);
			return this;
		}
		public GearTrainSim addDualGear(double ri, double input_rad, double output_rad, int input_teeth, int output_teeth) {
			this._append(
				Node.makeDoubleGear(ri, input_rad, output_rad, input_teeth, output_teeth) );
			this._recalc();
			return this;
		}
		// only use this when the gear is not the first or last in the chain
		public GearTrainSim addGearPulley(double ri, int gear_teeth, int pulley_teeth) {
			this.addGearPulley(ri, 1.0, 1.0, gear_teeth, pulley_teeth);
			return this;
		}
		public GearTrainSim addGearPulley(double ri, double gear_rad, double pulley_rad, int gear_teeth, int pulley_teeth) {
			this._append(
				Node.makeGearPulley(ri, gear_rad, pulley_rad, gear_teeth, pulley_teeth) );
			this._recalc();
			return this;
		}

		public int numNodes() {
			return this.length;
		}
		public double propegateVel(double a_vel) {
			return a_vel * this.sum_ratio;
		}
		public double propegateVelRev(double a_vel) {
			return a_vel / this.sum_ratio;
		}
		public double propegateTq(double tq) {
			return tq / this.sum_ratio;
		}
		public double propegateTqRev(double tq) {
			return tq * this.sum_ratio;
		}
		/** Apply the friction model to all nodes in the chain to get the net friction */
		public double sumFrictionFwd(double omega, double src_torque, FrictionModel m) {
			double
				_v = omega,
				_tq = src_torque,
				tq_scalar = 1.0,
				friction = 0.0;
			LinkedNode n = this.beg;
			while(n != null) {
				_v = n.fromPrevAVel(_v);	// new angular velocity and torque acting on the current node
				_tq = n.fromPrevTq(_tq);
				tq_scalar *= n.prevTq(1.0);
				friction += m.calc(1.0, _v, _tq) * tq_scalar;
				n = n.post;
			}
			return friction;
		}
		/** Apply the friction model to all nodes in the chain to get the net friction -- applied from end to start (reverse) */
		public double sumFrictionRev(double omega, double src_torque, FrictionModel m) {
			double
				_v = omega,
				_tq = src_torque,
				tq_scalar = 1.0,
				friction = 0.0;
			LinkedNode n = this.end;
			while(n != null) {
				_v = n.fromNextAVel(_v);
				_tq = n.fromNextTq(_tq);
				tq_scalar *= n.nextTq(1.0);
				friction += m.calc(1.0, _v, _tq) * tq_scalar;
				n = n.pre;
			}
			return friction;
		}

	}

	public static class TestModuleSim implements SwerveSimulator.SwerveModuleSim {

		private static final double WHEEL_RADIUS = 0.04699;
		private static final DCMotor
			steer_motor = DCMotor.getFalcon500(1),
			drive_motor = DCMotor.getFalcon500(1);
		private static final GearTrainSim
			steer_gt = new GearTrainSim()
				.addGear(2e-5, 0.0, 14)
				.addGearPulley(5.433e-5, 50, 10)
				.addGear(6.403e-4, 0.0, 60),
			drive_gt = new GearTrainSim()
				.addGear(2e-5, 0.0, 14)
				.addGear(5.382e-5, 0.0, 50)
				.addDualGear(7.389e-5, 50, 27)
				.addDualGear(2e-5, 19, 15)
				.addGear(4.416e-4, WHEEL_RADIUS, 45);		// the radius is the wheel's radius
		private static final FrictionModel
			steer_gt_frict = new StribeckFriction(0, 0, 0, 0),
			drive_gt_frict = new StribeckFriction(0, 0, 0, 0),
			steer_floor_frict = new StribeckFriction(0, 0, 0, 0),
			wheel_side_frict = new StribeckFriction(0, 0, 0, 0);
		private static final double
			MODULE_STATIC_RI = 0.013,		// about the steer axis, or wherever the module's measured center is
			MODULE_STATIC_LI = 2.115,
			STEER_GT_RI = steer_gt.sum_inertia_end,
			DRIVE_GT_RI = drive_gt.sum_inertia_end,
			STEER_GT_RATIO = steer_gt.sum_ratio,	// the FWD ratio -- from motor to output -> use the inverse in reverse operations
			DRIVE_GT_RATIO = drive_gt.sum_ratio;	// the FWD ratio -- from motor to wheel -> use the inverse in reverse operations

		@Override
		public double steerAAccel(double a_volts, double b_volts, double steer_rate, double wheel_vel_linear, double f_norm) {
			final double
				m_av = steer_gt.propegateVelRev(steer_rate),	// turn rate to motor speed via gearing
				o_tq = steer_gt.propegateTq(steer_motor.getTorque(steer_motor.getCurrent(m_av, a_volts))),	// motor torque geared @ output
				f_tq = steer_gt.sumFrictionRev(steer_rate, o_tq, steer_gt_frict) + steer_floor_frict.calc(f_norm, steer_rate, o_tq),
				s_tq = o_tq - f_tq;		// applyFriction() here
			return s_tq / STEER_GT_RI;

		}
		@Override
		public double wheelForceM(double a_volts, double b_volts, double steer_rate, double wheel_vel_linear) {
			final double
				m_av = (wheel_vel_linear / WHEEL_RADIUS) / DRIVE_GT_RATIO,
				o_tq = drive_motor.getTorque(drive_motor.getCurrent(m_av, b_volts)) / DRIVE_GT_RATIO;
			return o_tq / WHEEL_RADIUS;
		}
		@Override
		public double wheelSideFriction(double f_src, double momentum, double f_norm) {
			return 0.0;
		}
		@Override
		public double wheelGearFriction(double f_src, double momentum, double a_volts, double b_volts, double steer_rate, double wheel_vel_linear, double f_norm) {
			return 0.0;
		}
		@Override
		public double moduleMass() {
			return MODULE_STATIC_LI;
		}
		@Override
		public double effectiveLinearInertia(double vec_wheel_dtheta) {
			// checks for max/min shortcuts
			final double
				cos = Math.cos(vec_wheel_dtheta),
				cos2 = cos * cos,
				IR_l = drive_gt.sum_inertia_end / (WHEEL_RADIUS * WHEEL_RADIUS),
				IL2 = MODULE_STATIC_LI * MODULE_STATIC_LI;
			return Math.sqrt(IL2 + (2.0 * MODULE_STATIC_LI + IR_l) * IR_l * cos2);
		}
		@Override
		public double effectiveRotationalInertia(double vec_wheel_dtheta, double module_radius) {
			final double LI_v = this.effectiveLinearInertia(vec_wheel_dtheta);
			return MODULE_STATIC_RI + LI_v * module_radius * module_radius;
		}

	}
	public static class TestModule extends SwerveModule {

		public TestModule() {
			super(null);
		}

		@Override
		public void setState(double linear_vel, double steer_angle_rad, double linear_acc, double steer_angular_vel) {}
		@Override
		public double getSteeringAngle() { return 0.0; }
		@Override
		public double getWheelDisplacement() { return 0.0; }
		@Override
		public double getMotorAVolts() { return 0.0; }
		@Override
		public double getMotorBVolts() { return 0.0; }
		@Override
		public void setSimulatedSteeringAngle(double angle) {}
		@Override
		public void setSimulatedSteeringRate(double omega) {}
		@Override
		public void setSimulatedWheelPosition(double angle) {}
		@Override
		public void setSimulatedWheelVelocity(double omega) {}

	}

	private final DoubleSupplier x_speed, y_speed, turn_speed;	// in meters per second and degrees per second
	private Pose2d robot_pose2d = new Pose2d();
	private Timer timer = new Timer();
	private ChassisStates robot_vec = new ChassisStates();

	private SwerveKinematics kinematics;
	private SwerveVisualization visualization;
	private SwerveModuleStates[] wheel_states = new SwerveModuleStates[4];

	public TestSim(
		DoubleSupplier xspeed, DoubleSupplier yspeed, DoubleSupplier trnspeed,
		Translation2d... modules
	) {
		this.kinematics = new SwerveKinematics(modules);
		this.visualization = new SwerveVisualization(modules);

		this.x_speed = xspeed;
		this.y_speed = yspeed;
		this.turn_speed = trnspeed;

		Arrays.fill(this.wheel_states, new SwerveModuleStates());
	}

	@Override
	public void initialize() {
		this.robot_vec = new ChassisStates();
		this.timer.reset();
		this.timer.start();
	}

	@Override
	public void execute() {

		final double dt = this.timer.get();
		this.timer.reset();
		this.timer.start();

		final double
			vx = x_speed.getAsDouble(),
			vy = y_speed.getAsDouble(),
			vtheta = turn_speed.getAsDouble();

		ChassisStates.accFromDelta(this.robot_vec, new ChassisStates(vx, vy, vtheta), dt, this.robot_vec);
		this.wheel_states = this.kinematics.toModuleStates(this.robot_vec);

		this.robot_pose2d = this.robot_pose2d.exp(new Twist2d(
			this.robot_vec.x_velocity * dt,
			this.robot_vec.y_velocity * dt,
			this.robot_vec.angular_velocity * dt
		));

	}

	@Override
	public boolean isFinished() {
		return false;
	}
	
	@Override
	public void end(boolean i) {
		
	}

	@Override
	public void initSendable(SendableBuilder builder) {
		builder.addDoubleArrayProperty("Robot Pose", ()->Util.toComponents2d(this.robot_pose2d), null);
		builder.addDoubleArrayProperty("Wheel Poses", ()->Util.toComponents3d(this.visualization.getWheelPoses3d(this.wheel_states)), null);
		builder.addDoubleArrayProperty("Wheel Vectors", ()->SwerveVisualization.getVecComponents2d(this.wheel_states), null);
	}

}
