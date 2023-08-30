package frc.robot.swerve.simutil;

import frc.robot.team3407.Util;


/** FrictionModel represents any model of the friction between 2 surfaces. */
public interface FrictionModel {

	/** calculate the maximum applicant friction given the orthogonal force, velocity, and sum independant force */
	public double calc(double f_norm, double vel, double f_ext);


	/** Sum external forces with a maximum friction component while taking into account the current momentum (friction can only "slow down", not reverse movement)
	 * @param f_app - the sum applicant force at the friction interaction
	 * @param p_sys - the system's relative momentum
	 * @param f_frict - the maximum friction force that can be applied
	 * @param dt - the timestep under which the calculation is being done, if applicable
	*/
	public static double applyFriction(double f_app, double p_sys, double f_frict, double dt) {
		final double vdir = Math.signum(p_sys);	// apply epsilon --> 0.0
		if(vdir != 0.0) {	// friction opposes the movement
			return f_app + Math.min(Math.abs(f_frict), Math.abs(p_sys / dt)) * -vdir;	// make sure that the applied friction cannot reverse the direction based on dt
		} else {	// sum the source forces and friction (applied in opposition)
			return f_app + Math.min(Math.abs(f_frict), Math.abs(f_app)) * -Math.signum(f_app);
		}
	}




	/** StribeckFriction models the static, kinetic, and viscous friciton components of a fricitonal interaction. */
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


}
