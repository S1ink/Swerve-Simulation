package frc.robot.swerve.modules;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.system.plant.DCMotor;

import com.ctre.phoenix.sensors.*;
import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.*;

import frc.robot.swerve.*;
import frc.robot.team3407.drive.Types.*;


public class FalconMK4i /*extends SwerveModule*/ {

	// public static class ModuleConfig {
		
	// 	public final NeutralMode
	// 		STEERING_NEUTRAL_MODE,
	// 		DRIVING_NEUTRAL_MODE;
		
	// 	public final TalonFXInvertType
	// 		STEERING_INVERSION,
	// 		DRIVING_INVERSION;

	// 	public final double
	// 		WHEEL_RADIUS,
	// 		STEER_GEARING,
	// 		DRIVE_GEARING,
	// 		STEER_GEARTRAIN_RI,
	// 		DRIVE_GEARTRAIN_RI,

	// 		MAX_VOLTAGE_OUTPUT,
	// 		MAX_STEER_VELOCITY,
	// 		MAX_DRIVE_VELOCITY,
	// 		MAX_STEER_ACCELERATION,
	// 		MAX_DRIVE_ACCELERATION,
	// 		STEER_kP,
	// 		STEER_kI,
	// 		STEER_kD,
	// 		DRIVE_kP,
	// 		DRIVE_kI,
	// 		DRIVE_kD;
		
		



	// 	public ModuleConfig(
	// 		NeutralMode steering_neutral_mode,
	// 		NeutralMode driving_neutral_mode,
	// 		TalonFXInvertType steering_inversion,
	// 		TalonFXInvertType driving_inversion,
	// 		double wheel_radius,
	// 		double steer_gearing, double drive_gearing,
	// 		double steer_ri, double drive_ri,
	// 		double max_volrage_output,
	// 		double max_steer_velocity, double max_drive_velocity,
	// 		double max_steer_acceleration, double max_drive_acceleration,
	// 		double steer_kp, double steer_ki, double steer_kd,
	// 		double drive_kp, double drive_ki, double drive_kd
	// 	) {
	// 		this.STEERING_NEUTRAL_MODE = steering_neutral_mode;
	// 		this.DRIVING_NEUTRAL_MODE = driving_neutral_mode;
	// 		this.STEERING_INVERSION = steering_inversion;
	// 		this.DRIVING_INVERSION = driving_inversion;
	// 		this.WHEEL_RADIUS = wheel_radius;
	// 		this.STEER_GEARING = steer_gearing;
	// 		this.DRIVE_GEARING = drive_gearing;
	// 		this.STEER_GEARTRAIN_RI = steer_ri;
	// 		this.DRIVE_GEARTRAIN_RI = drive_ri;
	// 		this.MAX_VOLTAGE_OUTPUT = max_volrage_output;
	// 		this.MAX_STEER_VELOCITY = max_steer_velocity;
	// 		this.MAX_DRIVE_VELOCITY = max_drive_velocity;
	// 		this.MAX_STEER_ACCELERATION = max_steer_acceleration;
	// 		this.MAX_DRIVE_ACCELERATION = max_drive_acceleration;
	// 		this.STEER_kP = steer_kp;
	// 		this.STEER_kI = steer_ki;
	// 		this.STEER_kD = steer_kd;
	// 		this.DRIVE_kP = drive_kp;
	// 		this.DRIVE_kI = drive_ki;
	// 		this.DRIVE_kD = drive_kd;

	// 	}

	// 	public DCMotor getSteeringMotorProperties() {
	// 		return DCMotor.getFalcon500(1).withReduction(this.STEER_GEARING);
	// 	}
	// 	public DCMotor getDrivingMotorProperties() {
	// 		return DCMotor.getFalcon500(1).withReduction(this.DRIVE_GEARING);
	// 	}


	// }



	// public static final int SLOT_IDX = 0;

	// private final WPI_TalonFX
	// 	steer_motor, drive_motor;
	// private final CANCoder
	// 	steer_encoder;
	// public final ModuleConfig
	// 	configs;
	// public final DCMotor
	// 	steer_motor_props,
	// 	drive_motor_props;

	// /** 'A' motor should be the steering motor, 'B' motor should be the drive motor */
	// public FalconMK4i(SwerveModuleMap<WPI_TalonFX> module_map, CANCoder steer_enc, ModuleConfig config, Translation2d mod_location) {
	// 	super(mod_location);
	// 	this.steer_motor = module_map.A;
	// 	this.drive_motor = module_map.B;
	// 	this.steer_encoder = steer_enc;
	// 	this.configs = config;
	// 	this.steer_motor_props = this.configs.getSteeringMotorProperties();
	// 	this.drive_motor_props = this.configs.getDrivingMotorProperties();

	// 	this.steer_motor.configFactoryDefault();
	// 	this.drive_motor.configFactoryDefault();

	// 	this.steer_motor.setNeutralMode(config.STEERING_NEUTRAL_MODE);
	// 	this.drive_motor.setNeutralMode(config.DRIVING_NEUTRAL_MODE);
	// 	this.steer_motor.setInverted(config.STEERING_INVERSION);
	// 	this.drive_motor.setInverted(config.DRIVING_INVERSION);

	// 	this.steer_encoder.setPositionToAbsolute();
	// 	// this.steer_encoder.configFeedbackCoefficient();
	// 	this.steer_encoder.configSensorInitializationStrategy(SensorInitializationStrategy.BootToAbsolutePosition);
	// 	this.steer_motor.configRemoteFeedbackFilter(this.steer_encoder, 0);
	// 	this.steer_motor.configSelectedFeedbackSensor(FeedbackDevice.RemoteSensor0);
	// 	this.drive_motor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
	// 	this.steer_motor.setSelectedSensorPosition(0.0);
	// 	this.drive_motor.setSelectedSensorPosition(0.0);

	// 	this.steer_motor.config_kF(SLOT_IDX, 0);		// need to figure out the units and how to store the config
	// 	this.drive_motor.config_kF(SLOT_IDX, 0);

	// 	this.steer_motor.config_kP(SLOT_IDX, config.STEER_kP);
	// 	this.steer_motor.config_kI(SLOT_IDX, config.STEER_kI);
	// 	this.steer_motor.config_kD(SLOT_IDX, config.STEER_kD);
	// 	this.drive_motor.config_kP(SLOT_IDX, config.DRIVE_kP);
	// 	this.drive_motor.config_kI(SLOT_IDX, config.DRIVE_kI);
	// 	this.drive_motor.config_kD(SLOT_IDX, config.DRIVE_kD);

	// }



	// @Override
	// public void setState(double linear_vel, double steer_angle_rad, double linear_acc, double steer_angular_vel) {
	// 	this.steer_motor.set(ControlMode.Position, steer_angle_rad);	// transform to correct units
	// 	this.drive_motor.set(ControlMode.Velocity, linear_vel);			// transform from linear to angular
	// }

	// @Override
	// public double getSteeringAngle() {
	// 	return this.steer_encoder.getAbsolutePosition();	// need to convert
	// }
	// @Override
	// public double getWheelDisplacement() {
	// 	return this.drive_motor.getSelectedSensorPosition();	// need to convert
	// }


	// @Override
	// public double getMotorAVolts() { return this.steer_motor.getBusVoltage(); }
	// @Override
	// public double getMotorBVolts() { return this.drive_motor.getBusVoltage(); }

	// @Override
	// public SwerveSimulator.SwerveModuleSim getSimProperties() { return null; }



	// public static class FalconMK4iSim implements SwerveSimulator.SwerveModuleSim {



	// }


}
