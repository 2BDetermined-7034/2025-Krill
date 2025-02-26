package frc.robot.commands.Reef;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Auto.OTFPathFinding;
import frc.robot.commands.Intake.IntakeCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.ElevatorSubsystem;

public class ArmElevatorFactory {
	public static Command scoreCoral(CommandSwerveDrivetrain swerveDrivetrain, ElevatorSubsystem elevator, ArmSubsystem arm, ElevatorSubsystem.ScoringPosition scoringPosition) {
		return new ParallelCommandGroup(
			elevator.setElevatorPosition(scoringPosition),
			arm.setArmAngle(ArmSubsystem.ScoringPosition.Outtake),
			OTFPathFinding.goToNearestReef(swerveDrivetrain)
		);
	}

	public static Command intakeCoral(ElevatorSubsystem elevator, ArmSubsystem arm) {
		return new ParallelCommandGroup(
			elevator.setElevatorPosition(ElevatorSubsystem.ScoringPosition.HOME),
			arm.setArmAngle(ArmSubsystem.ScoringPosition.IntakeCoralStation),
			new IntakeCommand(arm)
		);
	}
}
