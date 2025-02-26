// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import frc.robot.commands.Auto.OTFPathFinding;
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

	private SendableChooser<Command> autoChooser;

	//private final CommandXboxController joystick = new CommandXboxController(0);
	private final CommandPS5Controller joystick = new CommandPS5Controller(0);

	public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
	public final ArmSubsystem arm = new ArmSubsystem();
	public final ElevatorSubsystem elevator = new ElevatorSubsystem();

	public RobotContainer() {

		NamedCommands.registerCommand("L4", ArmElevatorFactory.scoreCoral(drivetrain, elevator, arm, ScoringPosition.L4));
		NamedCommands.registerCommand("Outtake", new OuttakeCommand(arm));
		NamedCommands.registerCommand("1s Outtake", new OuttakeCommand(arm).withTimeout(Seconds.of(1)));
		NamedCommands.registerCommand("intakeCoral", ArmElevatorFactory.intakeCoral(elevator, arm));


		boolean isCompetition = false;

		// Build an auto chooser. This will use Commands.none() as the default option.
		// As an example, this will only show autos that start with "comp" while at
		// competition as defined by the programmer
		autoChooser = AutoBuilder.buildAutoChooserWithOptionsModifier(
			(stream) -> isCompetition
				? stream.filter(auto -> auto.getName().startsWith("comp"))
				: stream
		);


		autoChooser.addOption("2m", new PathPlannerAuto("2m"));
		SmartDashboard.putData("Auto Chooser", autoChooser);

		configureBindings();


	}

	private void configureBindings() {
		// Note that X is defined as forward according to WPILib convention,
		// and Y is defined as to the left according to WPILib convention.
		drivetrain.setDefaultCommand(
			// Drivetrain will execute this command periodically
			drivetrain.applyRequest(() ->
				drive.withVelocityX(joystick.getLeftY() * MaxSpeed * -0.75) // Drive forward with negative Y (forward)
					.withVelocityY(joystick.getLeftX() * MaxSpeed * -0.75) // Drive left with negative X (left)
					.withRotationalRate(-joystick.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
			)
		);

		joystick.options().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));


		joystick.R2().whileTrue(new IntakeCommand(arm));
		joystick.L2().whileTrue(new OuttakeCommand(arm));


		joystick.triangle().whileTrue(ArmElevatorFactory.intakeCoral(elevator, arm));
		joystick.povUp().whileTrue(ArmElevatorFactory.scoreCoral(drivetrain, elevator, arm, ScoringPosition.L4));
		joystick.povLeft().whileTrue(ArmElevatorFactory.scoreCoral(drivetrain, elevator, arm, ScoringPosition.L3));
		joystick.povRight().whileTrue(ArmElevatorFactory.scoreCoral(drivetrain, elevator, arm, ScoringPosition.L2));
		joystick.povRight().and(joystick.cross()).whileTrue(ArmElevatorFactory.scoreCoral(drivetrain, elevator, arm, ScoringPosition.L1));

		joystick.square().whileTrue(OTFPathFinding.goToNearestReef(drivetrain));

		drivetrain.registerTelemetry(logger::telemeterize);
	}

	public Command getAutonomousCommand() {
		return autoChooser.getSelected();
	}
}
