package frc.robot.commands.Swerve;

import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class CoralStationDriveCommand extends Command {
	private CommandSwerveDrivetrain swerve;
	private final SwerveRequest.RobotCentric drive = new SwerveRequest.RobotCentric()
		.withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1)

	public CoralStationDriveCommand(CommandSwerveDrivetrain swerve) {
		this.swerve = swerve;

		addRequirements(swerve);
	}

	@Override
	public void execute() {
		swerve.applyRequest()
	}
}
