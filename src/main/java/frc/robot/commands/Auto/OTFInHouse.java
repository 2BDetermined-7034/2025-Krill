package frc.robot.commands.Auto;

import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

import java.util.concurrent.atomic.AtomicReference;
import java.util.function.Supplier;

import static edu.wpi.first.units.Units.MetersPerSecond;

public class OTFInHouse {

	public static Command pathFindToPose(CommandSwerveDrivetrain drivetrain, Supplier<Pose2d> setpoint) {
		PIDController xController = new PIDController(3.0, 0.0, 0.0);
		PIDController yController = new PIDController(3.0, 0.0, 0.0);
		PIDController omegaController = new PIDController(10.0, 0.0, 0.0);
		SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric();

		xController.setTolerance(0.03);
		yController.setTolerance(0.03);
		omegaController.setTolerance(0.1);
		omegaController.enableContinuousInput(-0.5, 0.5);

		AtomicReference<Translation2d> previousVelocity = new AtomicReference<>(new Translation2d(0.0, 0.0));
		AtomicReference<Transform2d> tagLocalTransform = new AtomicReference<>();

		return Commands.runOnce(() -> {
			tagLocalTransform.set(new Transform2d(setpoint.get(), new Pose2d(0.0, 0.0, new Rotation2d(0.0))));
			Pose2d transformedSetpoint = setpoint.get().transformBy(tagLocalTransform.get());
			xController.setSetpoint(transformedSetpoint.getX());
			yController.setSetpoint(transformedSetpoint.getY());
			omegaController.setSetpoint(transformedSetpoint.getRotation().getRotations());
			SmartDashboard.putNumberArray("Pose setpoint", new double[] {setpoint.get().getX(), setpoint.get().getY(), setpoint.get().getRotation().getDegrees()});
			SmartDashboard.putNumberArray("Transformed pose setpoint", new double[] {transformedSetpoint.getX(), transformedSetpoint.getY(), transformedSetpoint.getRotation().getDegrees()});
		}).andThen(drivetrain.applyRequest(() -> {
			Pose2d transformedPose = drivetrain.getPose().transformBy(tagLocalTransform.get());
			Translation2d velocity = new Translation2d(
				-xController.calculate(transformedPose.getX()),
				-yController.calculate(transformedPose.getY())
			);

			Translation2d deltaVelocity = velocity.minus(previousVelocity.get());
			if (deltaVelocity.getNorm() > 3.5 * 0.02) {
				deltaVelocity.div(deltaVelocity.getNorm()).times(3.5 * 0.02);
			}

			velocity = previousVelocity.get().plus(deltaVelocity);

			double velocityMagnitude = velocity.getNorm() + 0.2;
			if (velocityMagnitude > TunerConstants.kSpeedAt12Volts.in(MetersPerSecond)) {
				velocityMagnitude = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
			}
			velocity.div(velocity.getNorm()).times(velocityMagnitude);

			previousVelocity.set(velocity);

			double omegaVelocity = omegaController.calculate(transformedPose.getRotation().getRotations());

			return drive
				.withVelocityX(velocity.getX())
				.withVelocityY(velocity.getY())
				.withRotationalRate(omegaVelocity + Math.signum(omegaVelocity) * 0.3);
			}
		));
	}
}
