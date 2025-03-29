package frc.robot.commands.Auto;

import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.*;
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

		return Commands.runOnce(() -> {
			xController.setSetpoint(setpoint.get().getX());
			yController.setSetpoint(setpoint.get().getY());
			omegaController.setSetpoint(setpoint.get().getRotation().getRotations());
			SmartDashboard.putNumberArray("Pose setpoint", new double[] {setpoint.get().getX(), setpoint.get().getY(), setpoint.get().getRotation().getDegrees()});
		}).andThen(drivetrain.applyRequest(() -> {
			Translation2d velocity = new Translation2d(
				-xController.calculate(drivetrain.getPose().getX()),
				-yController.calculate(drivetrain.getPose().getY())
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

			double omegaVelocity = omegaController.calculate(drivetrain.getPose().getRotation().getRotations());

			return drive
				.withVelocityX(velocity.getX())
				.withVelocityY(velocity.getY())
				.withRotationalRate(omegaVelocity + Math.signum(omegaVelocity) * 0.2);
			}
		)).andThen(Commands.print("OTF Pathfind Finished")).andThen(DriveForward.driveForward(drivetrain));
	}
}
