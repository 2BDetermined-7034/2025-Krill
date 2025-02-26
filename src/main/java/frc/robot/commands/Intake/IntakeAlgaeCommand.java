package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmSubsystem;

public class IntakeAlgaeCommand extends Command {
    private final ArmSubsystem arm;

    public IntakeAlgaeCommand(ArmSubsystem arm) {
        this.arm = arm;
        addRequirements(arm);
    }

    @Override
    public void execute() {
        arm.getIntakeMotor().set(0.5);
    }

    @Override
    public void end(boolean interrupted) {
        arm.getIntakeMotor().set(0.0);
    }
}
