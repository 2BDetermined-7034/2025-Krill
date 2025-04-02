// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.util.PathPlannerLogging;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.Auto.OTFPathFinding;
import frc.robot.commands.Auto.PointAtCoralStation;
import frc.robot.commands.Auto.PointAtReef;
import frc.robot.commands.Intake.IntakeCommand;
import frc.robot.commands.Intake.OuttakeCommand;
import frc.robot.commands.Reef.ArmElevatorFactory;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.ElevatorSubsystem.ElevatorPosition;

import static edu.wpi.first.units.Units.*;


public class RobotContainer {
	public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
	public final ArmSubsystem arm = new ArmSubsystem();
	public final ElevatorSubsystem elevator = new ElevatorSubsystem();
	public final ClimbSubsystem climb = new ClimbSubsystem();
	private final double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
	private final double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity
	/* Setting up bindings for necessary control of the swerve drive platform */
	private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
		.withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
		.withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
	private final SwerveRequest.RobotCentric driveCentric = new SwerveRequest.RobotCentric()
		.withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
		.withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
	private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
	private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
	private final Telemetry logger = new Telemetry(MaxSpeed);
	//private final CommandXboxController joystick = new CommandXboxController(0);
	private final CommandPS5Controller driverController = new CommandPS5Controller(0);
	private final CommandPS5Controller operatorController = new CommandPS5Controller(1);

	private final SendableChooser<Command> autoChooser;
	private final Field2d field;

	public RobotContainer() {


		namedCommands();
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

		field = new Field2d();
		SmartDashboard.putData("Field", field);
		// Logging callback for target robot pose
		PathPlannerLogging.setLogTargetPoseCallback((pose) -> {
			// Do whatever you want with the pose here
			field.getObject("target pose").setPose(pose);
		});
		// Logging callback for the active path, this is sent as a list of poses
		PathPlannerLogging.setLogActivePathCallback((poses) -> {
			// Do whatever you want with the poses here
			field.getObject("path").setPoses(poses);
		});
	}

	private void namedCommands() {

		NamedCommands.registerCommand("L4", ArmElevatorFactory.scoreCoral(elevator, arm, ElevatorPosition.L4));
		NamedCommands.registerCommand("L4 Elevator Setpoint", ArmElevatorFactory.scoreCoralElevatorSetpoint(elevator, arm, ElevatorPosition.L4).andThen(new WaitCommand(.05)));

		NamedCommands.registerCommand("L2", ArmElevatorFactory.scoreCoral(elevator, arm, ElevatorPosition.L2));
		NamedCommands.registerCommand("Elevator Ground", elevator.setElevatorPosition(ElevatorPosition.INTAKE).andThen(arm.setArmAngle(ArmSubsystem.ScoringPosition.INTAKE)));

		NamedCommands.registerCommand("Outtake", new OuttakeCommand(arm));
		NamedCommands.registerCommand("1s Outtake", new OuttakeCommand(arm).withTimeout(Seconds.of(0.8)));
		NamedCommands.registerCommand("0.5s Outtake", new OuttakeCommand(arm).withTimeout(Seconds.of(0.5)));
		NamedCommands.registerCommand("0.3s Outtake", new OuttakeCommand(arm).withTimeout(Seconds.of(0.3)));

		NamedCommands.registerCommand("intakeCoral", ArmElevatorFactory.intakeCoral(elevator, arm));
		NamedCommands.registerCommand("Spin Intake", arm.spinIntakeCommand());
		NamedCommands.registerCommand("Flick Outtake", arm.setArmAngle(ArmSubsystem.ScoringPosition.OUTTAKE_FLICK).andThen(new WaitCommand(0.5)));
		NamedCommands.registerCommand("Ram", drivetrain.applyRequest(() -> driveCentric.withVelocityX(0.6)));

	}

	private void configureBindings() {
		// Note that X is defined as forward according to WPILib convention,
		// and Y is defined as to the left according to WPILib convention.
		drivetrain.setDefaultCommand(
			// Drivetrain will execute this command periodically
			drivetrain.applyRequest(() ->
				drive.withVelocityX(driverController.getLeftY() * MaxSpeed * -0.75) // Drive forward with negative Y (forward)
					.withVelocityY(driverController.getLeftX() * MaxSpeed * -0.75) // Drive left with negative X (left)
					.withRotationalRate(-driverController.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
			)
		);

		driverController.L1().whileTrue(PointAtReef.pointAtReef(
			() -> driverController.getLeftY() * MaxSpeed * -0.75,
			() -> driverController.getLeftX() * MaxSpeed * -0.75,
			MaxSpeed * 0.1,
			drivetrain
		));
		driverController.R1().whileTrue(PointAtCoralStation.pointAtCoralStation(
			() -> driverController.getLeftY() * MaxSpeed * -0.75,
			() -> driverController.getLeftX() * MaxSpeed * -0.75,
			MaxSpeed * 0.1,
			drivetrain
		));

		driverController.options().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

		driverController.square().whileTrue(OTFPathFinding.goToNearestReef(drivetrain, OTFPathFinding.ReefSide.LEFT));
		driverController.circle().whileTrue(OTFPathFinding.goToNearestReef(drivetrain, OTFPathFinding.ReefSide.RIGHT));

		driverController.triangle().whileTrue(OTFPathFinding.goToNearestCoralStation(drivetrain));
		driverController.PS().whileTrue(arm.zero());

		driverController.povUp().whileTrue(drivetrain.applyRequest(() -> driveCentric.withVelocityX(0.5).withVelocityY(0)));
		driverController.povDown().whileTrue(drivetrain.applyRequest(() -> driveCentric.withVelocityX(-0.5).withVelocityY(0)));
		driverController.povLeft().whileTrue(drivetrain.applyRequest(() -> driveCentric.withVelocityX(0.1).withVelocityY(0.5)));
		driverController.povRight().whileTrue(drivetrain.applyRequest(() -> driveCentric.withVelocityX(0.1).withVelocityY(-0.5)));


		operatorController.povUp().whileTrue(elevator.setElevatorVoltage(Volts.of(2.0)));
		operatorController.povLeft().whileTrue(ArmElevatorFactory.intakeCoral(elevator, arm));
		operatorController.povDown().whileTrue(elevator.setElevatorVoltage(Volts.of(-1)));
		operatorController.create().onTrue(arm.setArmAngle(ArmSubsystem.ScoringPosition.INTAKE));

		operatorController.L2().whileTrue(climb.setClimbVoltage(Volts.of(7)));
		operatorController.R2().whileTrue(climb.setClimbVoltage(Volts.of(-7)));

		operatorController.PS().and(operatorController.L2()).onTrue(
			climb.climbUntil(Constants.Climb.ClimbDirection.POSITIVE, Constants.Climb.ClimbPositions.EXTENDED, Volts.of(7.0)));
		operatorController.PS().and(operatorController.R2()).onTrue(
			climb.climbUntil(Constants.Climb.ClimbDirection.NEGATIVE, Constants.Climb.ClimbPositions.RETRACTED, Volts.of(-7.0)));

		operatorController.R1().whileTrue(new IntakeCommand(arm));
		operatorController.L1().whileTrue(new OuttakeCommand(arm));


		operatorController.povRight().onTrue(ArmElevatorFactory.scoreCoral(elevator, arm, ElevatorPosition.L1));
		operatorController.triangle().onTrue(ArmElevatorFactory.scoreCoral(elevator, arm, ElevatorPosition.L4));
		operatorController.square().onTrue(ArmElevatorFactory.scoreCoral(elevator, arm, ElevatorPosition.L3));
		operatorController.circle().onTrue(elevator.setElevatorPosition(ElevatorPosition.HOME));
		operatorController.cross().onTrue(ArmElevatorFactory.scoreCoral(elevator, arm, ElevatorPosition.L2));
		operatorController.options().whileTrue(ArmElevatorFactory.intakeCoralGap(elevator, arm));

		// Drivebase Tip Detection.
		new Trigger(() -> {

			double pitch = Math.abs(drivetrain.getPitch().in(Degrees));
			pitch = Math.min(pitch, 180 - pitch);

			double roll = Math.abs(drivetrain.getRoll().in(Degrees));
			roll = Math.min(roll, 180 - roll);

			return pitch > Constants.Elevator.AUTO_RETRACT_THRESHOLD.in(Degrees)
				|| roll > Constants.Elevator.AUTO_RETRACT_THRESHOLD.in(Degrees);
		}
		).onTrue(elevator.setElevatorPosition(ElevatorPosition.HOME));
//
		drivetrain.registerTelemetry(logger::telemeterize);
	}

	public Command getAutonomousCommand() {
		return autoChooser.getSelected();
	}
}
