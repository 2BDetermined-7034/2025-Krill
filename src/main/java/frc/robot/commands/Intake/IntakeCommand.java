package frc.robot.commands.Intake;

import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.VoltageOut;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmSubsystem;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RotationsPerSecond;

public class IntakeCommand extends Command {
	private final ArmSubsystem arm;
	private boolean overcameInitial;

	public IntakeCommand(ArmSubsystem arm) {
		this.arm = arm;
	}

	@Override
	public void initialize() {
		overcameInitial = false;
		arm.getIntakeMotor().setControl(new VoltageOut(8.0));
	}

	@Override
	public void execute() {
		if (arm.getIntakeMotor().getVelocity().getValue().in(Units.RotationsPerSecond) >= 30.0) {
			overcameInitial = true;
		}
	}

	@Override
	public void end(boolean interrupted) {
		arm.getIntakeMotor().setControl(new CoastOut());
	}

	@Override
	public boolean isFinished() {
		if (overcameInitial) {
			return arm.getIntakeMotor().getTorqueCurrent().getValue().gt(Amps.of(99));
//				&& arm.getIntakeMotor().getVelocity().getValue().abs(RotationsPerSecond) < 2;
		}

		return false;
	}
}
