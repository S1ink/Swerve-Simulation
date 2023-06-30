package frc.robot;

import edu.wpi.first.math.geometry.*;

import com.ctre.phoenix.sensors.*;
import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.*;

import frc.robot.SwerveDrive.*;
import frc.robot.team3407.drive.Types.*;


public final class SwerveModules {

	public static class FalconCoaxModule implements SwerveModule {

		public static class ModuleConfig {
			
			public final NeutralMode
				STEERING_NEUTRAL_MODE,
				DRIVING_NEUTRAL_MODE;
			
			public final TalonFXInvertType
				STEERING_INVERSION,
				DRIVING_INVERSION;

			public final double
				WHEEL_RADIUS,
				MAX_VOLTAGE_OUTPUT,
				MAX_STEER_VELOCITY,
				MAX_DRIVE_VELOCITY,
				MAX_STEER_ACCELERATION,
				MAX_DRIVE_ACCELERATION,
				STEER_kP,
				STEER_kI,
				STEER_kD,
				DRIVE_kP,
				DRIVE_kI,
				DRIVE_kD;
			
			



			public ModuleConfig(
				NeutralMode steering_neutral_mode,
				NeutralMode driving_neutral_mode,
				TalonFXInvertType steering_inversion,
				TalonFXInvertType driving_inversion,
				double wheel_radius, double max_volrage_output,
				double max_steer_velocity, double max_drive_velocity,
				double max_steer_acceleration, double max_drive_acceleration,
				double steer_kp, double steer_ki, double steer_kd,
				double drive_kp, double drive_ki, double drive_kd
			) {
				this.STEERING_NEUTRAL_MODE = steering_neutral_mode;
				this.DRIVING_NEUTRAL_MODE = driving_neutral_mode;
				this.STEERING_INVERSION = steering_inversion;
				this.DRIVING_INVERSION = driving_inversion;
				this.WHEEL_RADIUS = wheel_radius;
				this.MAX_VOLTAGE_OUTPUT = max_volrage_output;
				this.MAX_STEER_VELOCITY = max_steer_velocity;
				this.MAX_DRIVE_VELOCITY = max_drive_velocity;
				this.MAX_STEER_ACCELERATION = max_steer_acceleration;
				this.MAX_DRIVE_ACCELERATION = max_drive_acceleration;
				this.STEER_kP = steer_kp;
				this.STEER_kI = steer_ki;
				this.STEER_kD = steer_kd;
				this.DRIVE_kP = drive_kp;
				this.DRIVE_kI = drive_ki;
				this.DRIVE_kD = drive_kd;

			}


		}



		public static final int SLOT_IDX = 0;

		private final WPI_TalonFX
			steer_motor, drive_motor;
		private final CANCoder
			steer_encoder;
		public final ModuleConfig
			configs;
		public final Translation2d
			module_location;

		/** 'A' motor should be the steering motor, 'B' motor should be the drive motor */
		public FalconCoaxModule(SwerveModuleMap<WPI_TalonFX> module_map, int cancoder_id, ModuleConfig config, Translation2d mod_location) {
			this.steer_motor = module_map.A;
			this.drive_motor = module_map.B;
			this.steer_encoder = new CANCoder(cancoder_id);
			this.configs = config;
			this.module_location = mod_location;

			this.steer_motor.configFactoryDefault();
			this.drive_motor.configFactoryDefault();

			this.steer_motor.setNeutralMode(NeutralMode.Brake);
			this.drive_motor.setNeutralMode(NeutralMode.Brake);
			this.steer_motor.setInverted(TalonFXInvertType.CounterClockwise);
			this.drive_motor.setInverted(TalonFXInvertType.CounterClockwise);

			this.steer_motor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
			this.drive_motor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
			this.steer_motor.setSelectedSensorPosition(0.0);
			this.drive_motor.setSelectedSensorPosition(0.0);

			this.steer_motor.config_kP(SLOT_IDX, config.STEER_kP);
			this.steer_motor.config_kI(SLOT_IDX, config.STEER_kI);
			this.steer_motor.config_kD(SLOT_IDX, config.STEER_kD);
			this.drive_motor.config_kP(SLOT_IDX, config.DRIVE_kP);
			this.drive_motor.config_kI(SLOT_IDX, config.DRIVE_kI);
			this.drive_motor.config_kD(SLOT_IDX, config.DRIVE_kD);

		}



		public void setState(double angle, double velocity) {
			this.steer_motor.set(ControlMode.Position, angle);
			this.drive_motor.set(ControlMode.Velocity, velocity);
		}

	}

}
