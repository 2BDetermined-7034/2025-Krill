package frc.robot.commands.Auto;

import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;

import java.util.List;
import java.util.function.Supplier;

import static edu.wpi.first.units.Units.Rotations;

public class PointAtCoralStation extends Command {
	private static final Rotation2d blueRightCoralStationRot = new Rotation2d(Units.degreesToRadians(233.5));
	private static final Rotation2d blueLeftCoralStationRot = new Rotation2d(Units.degreesToRadians(-233.5));

	private static final Rotation2d redRightCoralStationRot = new Rotation2d(Units.degreesToRadians(-234)); //0.91629786
	private static final Rotation2d redLeftCoralStationRot = new Rotation2d(Units.degreesToRadians(234)); //5.32325422


	public static Command pointAtCoralStation(Supplier<Double> velocityX, Supplier<Double> velocityY, double deadband, CommandSwerveDrivetrain drivetrain) {
		SwerveRequest.FieldCentricFacingAngle pointAtCoralStation = new SwerveRequest.FieldCentricFacingAngle()
			.withDeadband(deadband) // Add a 10% deadband
			.withDriveRequestType(SwerveModule.DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors

		return drivetrain.applyRequest(() -> {
			boolean isBlue = DriverStation.getAlliance().get().equals(DriverStation.Alliance.Blue);
			Rotation2d rightCoralStationRot = null;
			Rotation2d leftCoralStationRot = null;
			if(isBlue){
				rightCoralStationRot = blueRightCoralStationRot;
				leftCoralStationRot = blueLeftCoralStationRot;
			} else {
				rightCoralStationRot = redRightCoralStationRot;
				leftCoralStationRot = redLeftCoralStationRot;
			}

			Rotation2d coralStationAngle = null;

			if(drivetrain.getPose().getTranslation().getY() < 4){
				coralStationAngle = rightCoralStationRot;
			} else{
				coralStationAngle = leftCoralStationRot;
			}

			return pointAtCoralStation
				.withVelocityX(velocityX.get())
				.withVelocityY(velocityY.get())
				.withTargetDirection(coralStationAngle)
				.withHeadingPID(10.0, 1.0, 0.0);
		});
	}
}
