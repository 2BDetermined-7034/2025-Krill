// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;

import frc.robot.commands.Intake.ArmAlgaeCommand;
import frc.robot.commands.Intake.IntakeCommand;
import frc.robot.commands.Intake.OuttakeCommand;
import frc.robot.commands.Reef.ArmElevatorFactory;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.ElevatorSubsystem.ScoringPosition;


public class RobotContainer {
	private final double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
	private final double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

	/* Setting up bindings for necessary control of the swerve drive platform */
	private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
		.withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
		.withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
	private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
	private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

	private final Telemetry logger = new Telemetry(MaxSpeed);

	//private final CommandXboxController joystick = new CommandXboxController(0);
	private final CommandPS5Controller joystick = new CommandPS5Controller(0);

	public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
	public final ArmSubsystem arm = new ArmSubsystem();
	public final ElevatorSubsystem elevator = new ElevatorSubsystem();

	public RobotContainer() {
		configureBindings();
	}

	private void configureBindings() {
		// Note that X is defined as forward according to WPILib convention,
		// and Y is defined as to the left according to WPILib convention.
		drivetrain.setDefaultCommand(
			// Drivetrain will execute this command periodically
			drivetrain.applyRequest(() ->
				drive.withVelocityX(joystick.getLeftY() * MaxSpeed * -0.5) // Drive forward with negative Y (forward)
					.withVelocityY(joystick.getLeftX() * MaxSpeed * -0.5) // Drive left with negative X (left)
					.withRotationalRate(-joystick.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
			)
		);

		joystick.options().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

        /*joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
        joystick.b().whileTrue(drivetrain.applyRequest(() ->
            point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))
        ));*/


//		joystick.circle().onTrue(arm.setPositionCommand(Degrees.of(30.0)));
//		joystick.cross().onTrue(arm.coastOutCommand());

		joystick.R2().whileTrue(new IntakeCommand(arm));
		joystick.L2().whileTrue(new OuttakeCommand(arm));
//		joystick.L1().and(joystick.L2()).whileTrue(new ArmAlgaeCommand(arm));

		//joystick.triangle().onTrue(elevator.setElevatorPosition(Rotations.of(1.5)));
		//joystick.square().onTrue(elevator.setElevatorPosition(Rotations.of(0)));

		/*
		joystick.povUp().and(joystick.triangle()).whileTrue(drivetrain.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
		joystick.povDown().and(joystick.triangle()).whileTrue(drivetrain.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
		joystick.povUp().and(joystick.cross()).whileTrue(drivetrain.sysIdDynamic(SysIdRoutine.Direction.kForward));
		joystick.povDown().and(joystick.cross()).whileTrue(drivetrain.sysIdDynamic(SysIdRoutine.Direction.kReverse));
		*/

		joystick.povDown().whileTrue(elevator.setElevatorPosition(ScoringPosition.HOME));
		joystick.povUp().whileTrue(elevator.setElevatorPosition(ScoringPosition.L4));
		joystick.povLeft().whileTrue(elevator.setElevatorPosition(ScoringPosition.L3));
		joystick.povRight().whileTrue(elevator.setElevatorPosition(ScoringPosition.L2));
		joystick.povRight().and(joystick.cross()).whileTrue(elevator.setElevatorPosition(ScoringPosition.L1));

		joystick.L1().whileTrue(ArmElevatorFactory.IntakeAlgae(elevator, arm, ScoringPosition.AlgaeBottom));
		joystick.R1().whileTrue(ArmElevatorFactory.IntakeAlgae(elevator, arm, ScoringPosition.AlgaeTop));






		drivetrain.registerTelemetry(logger::telemeterize);
	}

	public Command getAutonomousCommand() {
		return Commands.print("No autonomous command configured");
	}
}
