package frc.robot.commands.Reef;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.Intake.IntakeCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.ElevatorSubsystem;

public class ArmElevatorFactory {
	public static Command scoreCoral(ElevatorSubsystem elevator, ArmSubsystem arm, ElevatorSubsystem.ElevatorPosition elevatorPosition) {
		return new ParallelCommandGroup(
			elevator.setElevatorPosition(elevatorPosition),
			arm.setArmAngle(ArmSubsystem.ScoringPosition.Outtake)
		);
	}

	/**
	 *
	 * @param elevator
	 * @param arm
	 * @return
	 */
	public static Command intakeCoral(ElevatorSubsystem elevator, ArmSubsystem arm) {
		return new ParallelCommandGroup(
			Commands.print("intakeCoral Starting"),
			elevator.setElevatorPosition(ElevatorSubsystem.ElevatorPosition.INTAKE),
			arm.setArmAngle(ArmSubsystem.ScoringPosition.IntakeCoralStation),
			new IntakeCommand(arm)
		).andThen(Commands.print("intakeCoral Ended"));
	}

	/**
	 * Same thing as score coral but waits for elevator to reach setpoint
	 * @param elevator
	 * @param arm
	 * @param elevatorPosition
	 * @return
	 */
	public static Command scoreCoralElevatorSetpoint(ElevatorSubsystem elevator, ArmSubsystem arm, ElevatorSubsystem.ElevatorPosition elevatorPosition) {
		return new ParallelCommandGroup(
			elevator.setElevatorPositionSetpoint(elevatorPosition),
			arm.setArmAngle(ArmSubsystem.ScoringPosition.Outtake)
		);
	}
}
