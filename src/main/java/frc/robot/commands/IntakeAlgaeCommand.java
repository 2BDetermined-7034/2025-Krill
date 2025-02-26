package frc.robot.commands;

import com.ctre.phoenix6.controls.VoltageOut;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmSubsystem;

public class IntakeAlgaeCommand extends Command {
    private final ArmSubsystem arm;

    public IntakeAlgaeCommand(ArmSubsystem arm) {
        this.arm = arm;
    }

    @Override
    public void execute() {
        arm.getIntakeMotor().setControl(new VoltageOut(3.0));
    }

    @Override
    public void end(boolean interrupted) {
        arm.getIntakeMotor().setControl(new VoltageOut(0.0));
    }
}
