package frc.robot.swerve.simutil;

import java.util.List;


/** GearTrainModel model's a geartrain's inerta, torque, velocity, gear ratio, and sum frictional properties. */
public class GearTrainModel {

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

		/** The resultant sum rotational motion direction modifier of the geartrain (integer) from a list of nodes */
		public static int rotDirMod(List<Node> nodes) {
			int v = 1;
			for(int i = 0; i < nodes.size() - 1; i++) {		// don't access the last node; it's output does not connect to anything
				if(nodes.get(i).invert_next) {
					v *= -1;
				}
			}
			return v;
		}
		/** Sum the gear ratio of a list of nodes. */
		public static double sumRatio(List<Node> nodes) {
			double ratio = 1.0;
			int last_output_teeth = nodes.size() > 0 ? nodes.get(0).input_teeth : 1;
			for(Node n : nodes) {
				ratio *= n.input_teeth / last_output_teeth;
				last_output_teeth = n.output_teeth;
			}
			return ratio;
		}
		/** Sum the gear ratio of a list of nodes -- as viewed from end to start */
		public static double sumRatioRev(List<Node> nodes) {
			return 1.0 / sumRatio(nodes);
		}
		/** Sum the rotational inertia of the geartrain as "felt" about the geartrain's first node */
		public static double sumInertia(List<Node> nodes) {
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
		/** Sum the rotational inertia of the geartrain as "felt" about the geartrain's last node */
		public static double sumInertiaRev(List<Node> nodes) {
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
		/** The geartrain's inertia but as a point mass at the first node's radius */
		public static double sumInertiaL(List<Node> nodes) {
			final double r = nodes.get(0).input_rad;
			return sumInertia(nodes) / (r * r);
		}
		/** The geartrain's reverse inertia but as a point mass at the last node's radius */
		public static double sumInertiaRevL(List<Node> nodes) {
			final double r = nodes.get(nodes.size() - 1).output_rad;
			return sumInertiaRev(nodes) / (r * r);
		}


	}
	/** LinkedNode allows for Nodes to be connected in a doubly linked list. */
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

		/** Connect a linkednode after the current node. */
		public LinkedNode append(LinkedNode n) {
			this.post = n;
			n._prepend(this);
			return this;
		}
		/** Convert a node to a linkednode and connect it after the current node */
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


		/** Find the sum rotational inertia of the entire linked list of nodes */
		public static double sumChainInertia(LinkedNode beg) {
			final LinkedNode end = getLast(beg);
			return _sumChainInertia(end);
		}
		/** Worker method for sumChainInertia -- the computation works backwards so we actually need to start with the last node */
		public static double _sumChainInertia(LinkedNode end) {
			double sum = end.self.inertia;
			LinkedNode current = end;
			while((current = current.pre) != null) {
				final double R = current.nextRatio();
				sum = current.self.inertia + sum * (R * R);
			}
			return sum;
		}
		/** Find the sum rotational inertia of the linked list of nodes, but from the last node's perspective */
		public static double sumChainInertiaRev(LinkedNode beg) {
			double sum = beg.self.inertia;
			LinkedNode current = beg;
			while((current = current.post) != null) {
				final double R = current.revRatio();
				sum = current.self.inertia + sum * (R * R);
			}
			return sum;
		}



		public static LinkedNode getFirst(LinkedNode n) {
			LinkedNode ret = n;
			while(ret.pre != null) {
				ret = ret.pre;
			}
			return ret;
		}
		public static LinkedNode getLast(LinkedNode n) {
			LinkedNode ret = n;
			while(ret.post != null) {
				ret = ret.post;
			}
			return ret;
		}


	}



	/* GEARTRAINMODEL CLASS MEMBERS */

	private LinkedNode beg = null, end = null;
	private double
		sum_ratio = 1.0,
		sum_inertia_beg = 0.0,
		sum_inertia_end = 0.0;
	private int length = 0;


	public GearTrainModel(Node... nodes) {
		for(Node n : nodes) {
			this._append(n);
			this._recalc();
		}
	}
	public static GearTrainModel create() { return new GearTrainModel(); }

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

	/** Add a single staged gear to the geartrain. Only use this method if the gear is not first or last in the chain (no radius param). */
	public GearTrainModel addGear(double ri, int teeth) {
		return this.addGear(ri, 1.0, teeth);
	}
	/** Add a single staged gear to the geartrain. */
	public GearTrainModel addGear(double ri, double radius, int teeth) {
		this._append(
			Node.makeGear(ri, radius, teeth) );
		this._recalc();
		return this;
	}
	/** Add a dual staged gear to the geartrain. Only use this method if the gear is not the first or last in the chain. */
	public GearTrainModel addDualGear(double ri, int input_teeth, int output_teeth) {
		return this.addDualGear(ri, 1.0, 1.0, input_teeth, output_teeth);
	}
	/** Add a dual staged gear to the geartrain. The "input" values are for the side that connects to the previous node (FWD perspective), and the "output" value are the other side. */
	public GearTrainModel addDualGear(double ri, double input_rad, double output_rad, int input_teeth, int output_teeth) {
		this._append(
			Node.makeDoubleGear(ri, input_rad, output_rad, input_teeth, output_teeth) );
		this._recalc();
		return this;
	}
	/** Add a node to the chain with a gear input and pulley output. For the opposite (pulley input, etc.) use addDualGear(). */
	public GearTrainModel addGearPulley(double ri, int gear_teeth, int pulley_teeth) {
		return this.addGearPulley(ri, 1.0, 1.0, gear_teeth, pulley_teeth);
	}
	/** Add a node to the chain with a gear input and pulley output. For the opposite (pulley input, etc.) use addDualGear(). */
	public GearTrainModel addGearPulley(double ri, double gear_rad, double pulley_rad, int gear_teeth, int pulley_teeth) {
		this._append(
			Node.makeGearPulley(ri, gear_rad, pulley_rad, gear_teeth, pulley_teeth) );
		this._recalc();
		return this;
	}

	public int numNodes() {
		return this.length;
	}
	public double fwdRatio() {
		return this.sum_ratio;
	}
	public double revRatio() {
		return 1.0 / this.sum_ratio;
	}
	public double begInertia() {
		return this.sum_inertia_beg;
	}
	public double endInertia() {
		return this.sum_inertia_end;
	}
	/** Find the angular velocity of the last node given the angular velocity of the first node. */
	public double propegateVel(double a_vel) {
		return a_vel * this.sum_ratio;
	}
	/** Find the angular velocity of the first node given the angular velocity of the last node. */
	public double propegateVelRev(double a_vel) {
		return a_vel / this.sum_ratio;
	}
	/** Find the torque "felt" by the last node given the torque applied to the first node. */
	public double propegateTq(double tq) {
		return tq / this.sum_ratio;
	}
	/** Find the torque "felt" by the first node given the torque applied to the last node. */
	public double propegateTqRev(double tq) {
		return tq * this.sum_ratio;
	}
	/** Apply a friction model to all nodes in the chain to get the chain's net frictional torque as "felt" at the first node. */
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
			friction += m.calcNormalized(_v, _tq) * tq_scalar;
			n = n.post;
		}
		return friction;
	}
	/** Apply a friction model to all nodes in the chain to get the net frictional torque as "felt" at the last node (applied end to start) */
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
			friction += m.calcNormalized(_v, _tq) * tq_scalar;
			n = n.pre;
		}
		return friction;
	}


}
