package frc.robot.commands.Intake;

import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.VoltageOut;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmSubsystem;

public class OuttakeCommand extends Command {
	private final ArmSubsystem arm;

	public OuttakeCommand(ArmSubsystem arm) {
		this.arm = arm;
		addRequirements(arm);
	}

	@Override
	public void initialize() {
		arm.getIntakeMotor().setControl(new VoltageOut(-3.0));
	}

	@Override
	public void end(boolean interrupted) {
		arm.getIntakeMotor().setControl(new CoastOut());
	}
}
