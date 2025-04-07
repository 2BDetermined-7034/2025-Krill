package frc.robot.commands.Intake;

import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.VoltageOut;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;

public class OuttakeCommand extends Command {
	private final ArmSubsystem arm;
	private final ElevatorSubsystem elevator;

	public OuttakeCommand(ArmSubsystem arm, ElevatorSubsystem elevator) {
		this.arm = arm;
		this. elevator = elevator;
	}

	@Override
	public void initialize() {
		if (elevator.getElevatorAngle().isNear(ElevatorSubsystem.ElevatorPosition.L1.getAngle(), 0.1)) {
			arm.getIntakeMotor().setControl(new VoltageOut(Constants.Arm.OUTTAKE_VOLTAGE_L1));
		}
		else {
			arm.getIntakeMotor().setControl(new VoltageOut(Constants.Arm.OUTTAKE_VOLTAGE));
		}
	}

	@Override
	public void end(boolean interrupted) {
		arm.getIntakeMotor().setControl(new CoastOut());
	}
}
