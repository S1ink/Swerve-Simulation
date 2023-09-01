package frc.robot.swerve.simutil;


/** {@link SwerveModuleModel} represents/contains all the necessary physical behavior (or approximations) needed
 * for the swerve simulator to be able to simulate the robot's movement as a result of voltage inputs and possible
 * additional external forces. */
public interface SwerveModuleModel {

	/** Get the sum angular acceleration of the wheel assembly about the steer axis.
	 * 
	 * @param a_volts - the voltage applied to motor A in volts
	 * @param b_volts - the voltage applied to motor B in volts
	 * @param steer_rate - the current angular velocity of the wheel assembly about the steer axis in rad/s
	 * @param wheel_vel_linear - the linear velocity of the module along the wheel path in m/s
	 * @param f_norm - the normal force in N applied at the wheel-floor contact as a result of gravity and possible external forces
	 * @param dt - the external summation timestep in seconds
	 * 
	 * @return the angular acceleration of the wheel assembly about the steering axis in rad/s^2
	 */
	public double steerAAccel(
		double a_volts, double b_volts,
		double steer_rate, double wheel_vel_linear,
		double f_norm,
		double dt
	);


	/** Get the output force at the wheel-floor contact in N as a
	 * result of the motor dynamics.
	 * 
	 * @param a_volts - the voltage applied to motor A in volts
	 * @param b_volts - the voltage applied to motor B in volts
	 * @param steer_rate - the angular velocity of the wheel assembly about the steering axis in rad/s
	 * @param wheel_vel_linear - the linear velocity of the module along the wheel path in m/s
	 * @param dt - the external summation timestep in seconds
	 * 
	 * @return the resulting force applied by the wheel (at the floor) in N
	 */
	public double wheelForceM(
		double a_volts, double b_volts,
		double steer_rate, double wheel_vel_linear,
		double dt
	);

	/** Get the friction force in N applied at the wheel-floor contact 
	 * as a result of the wheel being pushed/moved sideways.
	 * 
	 * @param f_norm - the normal force in N applied at the wheel-floor contact as a result of gravity and possible external forces
	 * @param f_app - the sum applicant (source) force acting on the wheel in N, orthoganal to the wheel's heading (CCW -> +y -> LEFT is positive)
	 * @param vel_linear - the linear velocity in m/s that the module is traveling in the direction orthoganal to the wheel's heading (CCW -> +y -> LEFT is positive)
	 * @param dt - the external summation timestep in seconds
	 * 
	 * @return the friciton force in N
	 */
	public double wheelSideFriction(
		double f_norm, double f_app, double vel_linear,
		double dt
	);

	/** Get the friction force in N applied at the wheel radius as a result of the wheel being
	 * pushed/moved inline with its heading.
	 * 
	 * @param f_norm - the normal force in N applied at the wheel-floor contact as a result of gravity and possible external forces
	 * @param f_app - the sum external force acting on the wheel in N, parallel to the wheel's heading
	 * @param a_volts - the voltage applied to motor A in volts
	 * @param b_volts - the voltage applied to motor B in volts
	 * @param steer_rate - the current angular velocity of the wheel assimbly about the steer axis in rad/s
	 * @param wheel_vel_linear - the linear velocity of the module along the wheel path in m/s
	 * 
	 * @return the friction force in N at the wheel radius */
	public double wheelGearFriction(
		double f_norm, double f_app,
		double a_volts, double b_volts,
		double steer_rate, double wheel_vel_linear,
		double dt
	);


	/** Get the static linear inertia of the module (discludes any inertia from the wheel geartrain).
	 * 
	 * @return the mass of the module in Kg
	 */
	public double moduleMass();

	/** Get the effective linear inertia of the module in Kg - meaning the acutal mass plus any
	 * rotational inertia from the wheel being inline with the force applied, translated by the radius squared
	 * to represent a linear effect.
	 * 
	 * @param vec_wheel_dtheta - The delta angle between the wheel heading and the applicant vector.
	 * A delta of 0 means that the vector and wheel are inline.
	 * 
	 * @return the combined linear inertia of the module's mass in Kg and any additional inertia introduced
	 * from the wheel's geartrain being inline with the applied force.
	 */
	public double effectiveLinearInertia(
		double vec_wheel_dtheta
	);

	/** Get the effective rotational inertia in Kgm^2 of the module about the robot's center of gravity.
	 * 
	 * @param vec_wheel_dtheta - the delta angle between the wheel heading and the tangential
	 * direction that the vector is acting in
	 * @param module_radius - the distance from the center of the module to the robot's center of mass,
	 * ie. the radius at which the vector is being applied, and the radius at which the module's inertia is orbiting
	 * 
	 * @return the combined rotatinal inertia in Kgm^2 of the module (about the robot CG) that is a
	 * result of the module's static inertia and any additional inertia introduced by the wheel geartrain.
	 */
	public double effectiveRotationalInertia(
		double vec_wheel_dtheta,
		double module_radius
	);





	// public static final class MotorStates {
	// 	public double
	// 		a_volts,
	// 		b_volts,
	// 		steer_rate,
	// 		wheel_vel_linear;

	// 	public MotorStates(double av, double bv, double srate, double wvel) {
	// 		this.a_volts = av;
	// 		this.b_volts = bv;
	// 		this.steer_rate = srate;
	// 		this.wheel_vel_linear = wvel;
	// 	}
	// }

	// /** Get the torque applied to the wheel assembly about the steering axis in Nm ("felt" by the output of any possible gearing)
	//  * as a result of the motor dynamics. 
	//  * @param a_volts - the voltage applied to motor A in volts
	//  * @param b_volts - the voltage applied to motor B in volts
	//  * @param steer_rate - the angular velocity of the wheel assembly about the steering axis in rad/s
	//  * @param wheel_vel_linear - the linear velocity of the module along the wheel path in m/s
	//  * @return the resulting torque about the steer axis in Nm */
	// public double steerTorqueM(double a_volts, double b_volts, double steer_rate, double wheel_vel_linear);


}
