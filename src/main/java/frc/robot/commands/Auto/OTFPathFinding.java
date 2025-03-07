package frc.robot.commands.Auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import frc.robot.subsystems.CommandSwerveDrivetrain;

import java.util.List;
import java.util.Set;

import static edu.wpi.first.units.Units.*;

public class OTFPathFinding {

	//edit to the positions that we want the robot to go to
	//They are currently the positions of the april tags

	// private static final Pose2d[] bluePoseArray = new Pose2d[]
	// 	{
	// 		new Pose2d(3.920,3.305,new Rotation2d(240)),
	// 		new Pose2d(3.458,4.967,new Rotation2d(180)),
	// 		new Pose2d(4.920,4.750,new Rotation2d(120)),
	// 		new Pose2d(4.902,4.950,new Rotation2d(60)),
	// 		new Pose2d(5.535,4.233,new Rotation2d(0)),
	// 		new Pose2d(5.102,3.305,new Rotation2d(300)),

	// 		new Pose2d(4.080,3.105,new Rotation2d(240)),
	// 		new Pose2d(3.258,4.233,new Rotation2d(180)),
	// 		new Pose2d(4.080,4.950,new Rotation2d(120)),
	// 		new Pose2d(5.102,4.750,new Rotation2d(60)),
	// 		new Pose2d(5.535,4.967,new Rotation2d(0)),
	// 		new Pose2d(4.902,3.105,new Rotation2d(300))
	// 	};

	// private static final Pose2d[] redPoseArray = new Pose2d[]
	// 	{
	// 		new Pose2d(13.693,3.305,new Rotation2d(240)),
	// 		new Pose2d(14.102,4.233,new Rotation2d(180)),
	// 		new Pose2d(13.493,4.950,new Rotation2d(120)),
	// 		new Pose2d(12.450,4.950,new Rotation2d(60)),
	// 		new Pose2d(12.033,4.967,new Rotation2d(0)),
	// 		new Pose2d(12.650,3.105,new Rotation2d(300)),

	// 		new Pose2d(13.493,3.205,new Rotation2d(240)),
	// 		new Pose2d(14.102,3.967,new Rotation2d(180)),
	// 		new Pose2d(13.893,4.750,new Rotation2d(120)),
	// 		new Pose2d(12.650,4.950,new Rotation2d(60)),
	// 		new Pose2d(12.033,4.233,new Rotation2d(0)),
	// 		new Pose2d(12.450,3.305,new Rotation2d(300))
	// 	};

	private static final Translation2d redReef = new Translation2d(Units.inchesToMeters(514.13), Units.inchesToMeters(158.5));
	private static final Translation2d blueReef = new Translation2d(Units.inchesToMeters(176.745), Units.inchesToMeters(158.5));

	private static final Pose2d RC_BLUE = new Pose2d(
		Meters.of(1.114217758178711), Meters.of(0.9747062921524048),
		new Rotation2d(Degrees.of(233.5)));
	private static final Pose2d LC_BLUE = new Pose2d(
		Meters.of(1.1308588981628418), Meters.of(7.066422462463379),
		new Rotation2d(Degrees.of(-233.5)));
	private static final Pose2d LC_RED = new Pose2d(
		Meters.of(16.4544677734375), Meters.of(0.9897422194480896),
		new Rotation2d(Degrees.of(-53.5)));
	private static final Pose2d RC_RED = new Pose2d(
		Meters.of(16.41495132446289), Meters.of(7.076528072357178),
		new Rotation2d(Degrees.of(53.5)));

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
	public static Command goToNearestReef(CommandSwerveDrivetrain drivebase) {
		return new DeferredCommand(
			() -> drivebase.driveToPose(getNearestReefLocation(drivebase)),
			Set.of()
		);
	}

	private static Pose2d getNearestReefLocation(CommandSwerveDrivetrain drivebase) {
		boolean isBlue = DriverStation.getAlliance().get().equals(Alliance.Blue);

		Translation2d reef = isBlue ? blueReef : redReef;

		Distance distFromReef = Inches.of(32.75 + 14);
		Distance distTangent = Inches.of(7);

		Angle angleToReef = Rotations.of(drivebase.getPose().getTranslation().minus(reef).getAngle().getRotations());
		Angle clampedAngle = Rotations.of(Math.round(angleToReef.in(Rotations) * 6.0) / 6.0);

		Translation2d translationFromReef = new Translation2d(
			distFromReef.in(Meters) * Math.cos(clampedAngle.in(Radians)),
			distFromReef.in(Meters) * Math.sin(clampedAngle.in(Radians)));

		Translation2d tangentLeft = new Translation2d(
			distTangent.in(Meters) * -Math.sin(clampedAngle.in(Radians)),
			distTangent.in(Meters) * Math.cos(clampedAngle.in(Radians))
		);
		Translation2d tangentRight = tangentLeft.times(-1);

		Pose2d poseLeft = new Pose2d(
			reef.plus(translationFromReef).plus(tangentLeft),
			new Rotation2d(clampedAngle).rotateBy(Rotation2d.fromDegrees(180))
		);
		Pose2d poseRight = new Pose2d(
			reef.plus(translationFromReef).plus(tangentRight),
			new Rotation2d(clampedAngle).rotateBy(Rotation2d.fromDegrees(180))
		);

		return drivebase.getPose().nearest(List.of(poseLeft, poseRight));
	}


	public static Command goToNearestCoralStation(CommandSwerveDrivetrain drivebase) {
		return new DeferredCommand(
			() -> goToNearestCoralStationUndeferred(drivebase),
			Set.of()
		);
	}

	private static Command goToNearestCoralStationUndeferred(CommandSwerveDrivetrain drivebase) {
		boolean isBlue = DriverStation.getAlliance().get().equals(Alliance.Blue);
		Pose2d currentPose = drivebase.getPose();

		if (isBlue) {
			Pose2d nearest = currentPose.nearest(List.of(RC_BLUE, LC_BLUE));
			return drivebase.driveToPose(nearest);
		}
		Pose2d nearest = currentPose.nearest(List.of(RC_RED, LC_RED));
		return drivebase.driveToPose(nearest);
	}
}