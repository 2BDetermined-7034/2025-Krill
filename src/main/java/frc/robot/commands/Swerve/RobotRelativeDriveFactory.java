package frc.robot.commands.Swerve;

import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class RobotRelativeDriveFactory {
	private final static double speed = 1.0;
	private final static SwerveRequest.RobotCentric drive = new SwerveRequest.RobotCentric();

	public static Command robotRelativeDrive(CommandSwerveDrivetrain swerve, double x, double y) {
		return swerve.applyRequest(() -> drive
			.withVelocityX(x * speed)
			.withVelocityY(y * speed)
		);
	}
}
