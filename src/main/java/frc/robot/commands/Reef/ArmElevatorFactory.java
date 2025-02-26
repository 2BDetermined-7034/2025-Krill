package frc.robot.commands.Reef;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.Intake.IntakeCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.ElevatorSubsystem;

public class ArmElevatorFactory {
	public static Command scoreCoral(CommandSwerveDrivetrain swerveDrivetrain, ElevatorSubsystem elevator, ArmSubsystem arm, ElevatorSubsystem.ScoringPosition scoringPosition) {
		return new ParallelCommandGroup(
			Commands.print("Command Starting"),
			elevator.setElevatorPosition(scoringPosition),
			arm.setArmAngle(ArmSubsystem.ScoringPosition.Outtake)
			//OTFPathFinding.goToNearestReef(swerveDrivetrain)
		).andThen(Commands.print("Command Ended"));
	}

	public static Command intakeCoral(ElevatorSubsystem elevator, ArmSubsystem arm) {
		return new ParallelCommandGroup(
			elevator.setElevatorPosition(ElevatorSubsystem.ScoringPosition.HOME),
			arm.setArmAngle(ArmSubsystem.ScoringPosition.IntakeCoralStation),
			new IntakeCommand(arm)
		);
	}
}
