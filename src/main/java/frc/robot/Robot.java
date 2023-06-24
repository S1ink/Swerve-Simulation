package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

import frc.robot.team3407.SenderNT;
import frc.robot.team3407.Util;
import frc.robot.team3407.commandbased.EventTriggers.TeleopTrigger;
import frc.robot.team3407.controls.ControlSchemeManager;
import frc.robot.team3407.controls.Input.*;


public class Robot extends TimedRobot {

	private final ControlSchemeManager controls = new ControlSchemeManager();
	private final EventLoop eloop = new EventLoop();
	private final SenderNT robot_nt = new SenderNT("Robot");


	@Override
	public void robotInit() {

		this.controls.setAmbiguousSolution(ControlSchemeManager.AmbiguousSolution.PREFER_COMPLEX);
		this.controls.setDefault("Xbox Sim Drive", (InputDevice... inputs)->{
			InputDevice xbox = inputs[0];
			TeleopTrigger.makeWithLoop(this.eloop).onTrue(
				Util.send(
					new TestSim(
						Xbox.Analog.RY.getDriveInputSupplier(xbox, 0.05, -4.0, 1.0),
						Xbox.Analog.RX.getDriveInputSupplier(xbox, 0.05, -4.0, 1.0),
						Xbox.Analog.LX.getDriveInputSupplier(xbox, 0.05, -3.0, 1.0)
					),
					this.robot_nt, "Test Sim"
				)
			);
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
