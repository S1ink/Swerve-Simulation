package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

import frc.robot.team3407.commandbased.EventTriggers.TeleopTrigger;
import frc.robot.team3407.controls.ControlSchemeManager;
import frc.robot.team3407.controls.Input.*;
import frc.robot.team3407.SenderNT;
import frc.robot.team3407.Util;
import frc.robot.swerve.*;


public class Robot extends TimedRobot {

	private final ControlSchemeManager controls = new ControlSchemeManager();
	private final EventLoop eloop = new EventLoop();
	private final SenderNT
		robot_nt = new SenderNT("Robot"),
		sim_nt = new SenderNT("Sim Data");


	@Override
	public void robotInit() {

		this.controls.setAmbiguousSolution(ControlSchemeManager.AmbiguousSolution.PREFER_COMPLEX);
		this.controls.setDefault("Xbox Sim Drive", (InputDevice... inputs)->{
			final InputDevice xbox = inputs[0];
			TestSim t;
			TeleopTrigger.makeWithLoop(this.eloop).onTrue(
				Util.send(
					(t = new TestSim(
						Xbox.Analog.RY.getDriveInputSupplier(xbox, 0.05, -4.0, 1.0),
						Xbox.Analog.RX.getDriveInputSupplier(xbox, 0.05, -4.0, 1.0),
						Xbox.Analog.LX.getDriveInputSupplier(xbox, 0.05, -3.0, 1.0),
						SwerveUtils.makeSquareLocationsCW(0.263525)
					)),
					this.robot_nt, "Test Sim"
				)
			);
			this.sim_nt.putData("Test Simulation", t.getSim());
			this.addPeriodic(()->t.getSim().integrate(0.005), 0.005);	// might be an issue if t gets deleted?
			this.addPeriodic(this.sim_nt::updateValues, 0.005);
		}, this.eloop::clear, Xbox.Map);

		this.addPeriodic(
			this.controls.genLoopableRunContinuous(),
			0.5
		);

	}
	@Override
	public void robotPeriodic() {
		this.eloop.poll();
		CommandScheduler.getInstance().run();
		this.robot_nt.updateValues();
	}



	@Override
	public void autonomousInit() {}

	@Override
	public void autonomousPeriodic() {}

	@Override
	public void teleopInit() {}

	@Override
	public void teleopPeriodic() {}

	@Override
	public void disabledInit() {}

	@Override
	public void disabledPeriodic() {}

	@Override
	public void testInit() {}

	@Override
	public void testPeriodic() {}

	@Override
	public void simulationInit() {}

	@Override
	public void simulationPeriodic() {}

}
