package frc.robot.commands.Auto;

import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;

import java.util.function.Supplier;

import static edu.wpi.first.units.Units.Rotations;

public class PointAtReef {
	private static final Translation2d redReef = new Translation2d(Units.inchesToMeters(514.13), Units.inchesToMeters(158.5));
	private static final Translation2d blueReef = new Translation2d(Units.inchesToMeters(176.745), Units.inchesToMeters(158.5));

	public static Command pointAtReef(Supplier<Double> velocityX, Supplier<Double> velocityY, double deadband, CommandSwerveDrivetrain drivetrain) {
		SwerveRequest.FieldCentricFacingAngle pointAtReef = new SwerveRequest.FieldCentricFacingAngle()
			.withDeadband(deadband) // Add a 10% deadband
			.withDriveRequestType(SwerveModule.DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors

		return drivetrain.applyRequest(() -> {
			boolean isBlue = DriverStation.getAlliance().get().equals(DriverStation.Alliance.Blue);

			Translation2d reef = isBlue ? blueReef : redReef;

			Rotation2d angleToReef = drivetrain.getPose().getTranslation().minus(reef).getAngle();
			Rotation2d clampedAngle = Rotation2d.fromRotations(Math.round(angleToReef.getRotations() * 6.0) / 6.0);

			return pointAtReef
				.withVelocityX(velocityX.get())
				.withVelocityY(velocityY.get())
				.withTargetDirection(clampedAngle)
				.withHeadingPID(10.0, 1.0, 0.0);
		});
	}
}
