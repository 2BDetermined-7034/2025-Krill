package frc.robot.commands.Auto;

import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.wpilibj.RobotBase;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import frc.robot.subsystems.CommandSwerveDrivetrain;

import java.util.List;
import java.util.Set;
import java.util.function.Supplier;

import static edu.wpi.first.units.Units.*;

public class OTFPathFinding {

	private static final Translation2d redReef = new Translation2d(Units.inchesToMeters(514.13), Units.inchesToMeters(158.5));
	private static final Translation2d blueReef = new Translation2d(Units.inchesToMeters(176.745), Units.inchesToMeters(158.5));

	private static final Pose2d RC_BLUE = new Pose2d(
		Meters.of(1.114217758178711), Meters.of(0.9747062921524048),
		new Rotation2d(Degrees.of(234)));
	private static final Pose2d LC_BLUE = new Pose2d(
		Meters.of(1.1308588981628418), Meters.of(7.066422462463379),
		new Rotation2d(Degrees.of(-234)));
	private static final Pose2d LC_RED = new Pose2d(
		Meters.of(16.4544677734375), Meters.of(0.9897422194480896),
		new Rotation2d(Degrees.of(-54)));
	private static final Pose2d RC_RED = new Pose2d(
		Meters.of(16.41495132446289), Meters.of(7.076528072357178),
		new Rotation2d(Degrees.of(54)));

	public enum ReefSide {
		LEFT,
		RIGHT,
	}

	// public static Command goToPose(SwerveSubsystem swerve, int scoreLocation) {
	// 	Alliance alliance = DriverStation.getAlliance().get();
	// 	if(alliance == Alliance.Blue) {
	// 		return swerve.driveToPose(bluePoseArray[scoreLocation]);
	// 	} else{
	// 		return swerve.driveToPose(redPoseArray[scoreLocation]);
	// 	}
	// }

	/**
	 * Drive to the nearest scoring location on the reef
	 *
	 * @param drivebase The swerve subsystem
	 * @return command to pathfind to the most optimal scoring location,
	 * trying to align with the driver's intent
	 */
	public static Command goToNearestReef(CommandSwerveDrivetrain drivebase, ReefSide reefSide) {
		return new DeferredCommand(
			() -> drivebase.getPathFromWaypoint(getNearestReefLocation(drivebase, reefSide)),
			Set.of()
		);
	}




	/**
	 * Finds the nearest branch location based on the current pose of the drivebase.
	 *
	 * @param drivebase The swerve subsystem
	 * @param reefSide An enum indicating whether to find the left branch or right branch
	 * @return The nearest branch location as a Pose2d object
	 */
	public static Pose2d getNearestReefLocation(CommandSwerveDrivetrain drivebase, ReefSide reefSide) {
		boolean isBlue = DriverStation.getAlliance().get().equals(Alliance.Blue); // default to red alliance in the Sim

		Translation2d reef = isBlue ? blueReef : redReef;

		Distance distFromReef = Centimeters.of(44.5).plus(Inches.of(32.75)).plus(Inches.of(0 /* Arbitrary */));
		Distance distTangent = Inches.of(6.5).plus(Inches.of(0 /*Arbitrary */)); // 12.93775566 / 2

		Angle angleToReef = Rotations.of(drivebase.getPose().getTranslation().minus(reef).getAngle().getRotations());
		Angle clampedAngle = Rotations.of(Math.round(angleToReef.in(Rotations) * 6.0) / 6.0);

		Translation2d translationFromReef = new Translation2d(
				distFromReef.in(Meters) * Math.cos(clampedAngle.in(Radians)),
				distFromReef.in(Meters) * Math.sin(clampedAngle.in(Radians)));

		Translation2d tangentRight = new Translation2d(
				distTangent.in(Meters) * -Math.sin(clampedAngle.in(Radians)),
				distTangent.in(Meters) * Math.cos(clampedAngle.in(Radians))
		);
		Translation2d tangentLeft = tangentRight.times(-1);

		Pose2d poseLeft = new Pose2d(
				reef.plus(translationFromReef).plus(tangentLeft),
				new Rotation2d(clampedAngle).rotateBy(Rotation2d.fromDegrees(180))
		);
		Pose2d poseRight = new Pose2d(
				reef.plus(translationFromReef).plus(tangentRight),
				new Rotation2d(clampedAngle).rotateBy(Rotation2d.fromDegrees(180))
		);

		if (drivebase.getPose().getX() > blueReef.getX() && drivebase.getPose().getX() < redReef.getX()) {
			return reefSide.equals(ReefSide.LEFT) ? poseRight : poseLeft;
		} else {
			return reefSide.equals(ReefSide.LEFT) ? poseLeft : poseRight;
		}
	}


	public static Command goToNearestCoralStation(CommandSwerveDrivetrain drivebase) {
		return new DeferredCommand(
			() -> goToNearestCoralStationUndeferred(drivebase),
			Set.of()
		);
	}

	private static Command goToNearestCoralStationUndeferred(CommandSwerveDrivetrain drivebase) {
		boolean isBlue = RobotBase.isReal() && DriverStation.getAlliance().get().equals(Alliance.Blue); // default to red alliance in the Sim
		Pose2d currentPose = drivebase.getPose();

		if (isBlue) {
			Pose2d nearest = currentPose.nearest(List.of(RC_BLUE, LC_BLUE));
			return drivebase.getPathFromWaypoint(nearest);
		}
		Pose2d nearest = currentPose.nearest(List.of(RC_RED, LC_RED));
		return drivebase.getPathFromWaypoint(nearest);
	}

	public static Command pathfindToPose(CommandSwerveDrivetrain drivetrain, Supplier<Pose2d> setpointSupplier) {
//		ProfiledPIDController xController = new ProfiledPIDController(
//			2.0, 0.0, 0.0,
//			new TrapezoidProfile.Constraints(4.0, 3.5)
//		);
//		ProfiledPIDController yController = new ProfiledPIDController(
//			2.0, 0.0, 0.0,
//			new TrapezoidProfile.Constraints(4.0, 3.5)
//		);
		PIDController xController = new PIDController(3, 0, 0);
		PIDController yController = new PIDController(3, 0, 0);
//		ProfiledPIDController thetaController = new ProfiledPIDController(
//			5, 0.0, 0.0,
//			new TrapezoidProfile.Constraints(2.0, 1.0)
//		);
		PIDController thetaController = new PIDController(5, 0, 0);
		thetaController.enableContinuousInput(-0.5, 0.5);

		var swerveRequest = new SwerveRequest.FieldCentric();

		return drivetrain.applyRequest(() -> {


			var vx = -xController.calculate(drivetrain.getPose().getX(), setpointSupplier.get().getX());
			var vy = -yController.calculate(drivetrain.getPose().getY(), setpointSupplier.get().getY());
			var w = thetaController.calculate(drivetrain.getPose().getRotation().getRotations(), setpointSupplier.get().getRotation().getRotations());

			SmartDashboard.putNumber("OTF pathfind error", drivetrain.getPose().getTranslation().getDistance(setpointSupplier.get().getTranslation()));

			return swerveRequest.withVelocityX(vx).withVelocityY(vy).withRotationalRate(w);
		});
	}
}