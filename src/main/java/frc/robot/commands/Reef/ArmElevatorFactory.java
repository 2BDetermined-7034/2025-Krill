package frc.robot.commands.Reef;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;

import static edu.wpi.first.units.Units.Degrees;

public class ArmElevatorFactory {
	public Command ScoreCoral(ElevatorSubsystem elevator, ArmSubsystem arm, ElevatorSubsystem.ScoringPosition scoringPosition) {
		return new SequentialCommandGroup(
			elevator.setElevatorPosition(scoringPosition)
		);
	}

	public Command IntakeCoral(ElevatorSubsystem elevator, ArmSubsystem arm) {
		return new ParallelCommandGroup(
			elevator.setElevatorPosition(ElevatorSubsystem.ScoringPosition.HOME),
			arm.setPositionCommand(Degrees.of(10))
		);
	}
}
