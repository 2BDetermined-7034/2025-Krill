package frc.robot.commands.Auto;

import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.CommandSwerveDrivetrain;

import static edu.wpi.first.units.Units.Seconds;

public class DriveForward {
	public static Command driveForward(CommandSwerveDrivetrain drivetrain) {
		var request = new SwerveRequest.RobotCentric();
		request.VelocityX = 1.0;
		return drivetrain.applyRequest(() -> request).withTimeout(Seconds.of(1.0));
	}
}
