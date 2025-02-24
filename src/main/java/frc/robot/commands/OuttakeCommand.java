package frc.robot.commands;

import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.VoltageOut;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmSubsystem;

import static edu.wpi.first.units.Units.Amps;

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
