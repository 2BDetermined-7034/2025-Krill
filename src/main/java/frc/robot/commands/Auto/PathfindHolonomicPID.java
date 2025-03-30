package frc.robot.commands.Auto;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.trajectory.PathPlannerTrajectoryState;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.CommandSwerveDrivetrain;

import java.util.Set;
import java.util.function.Supplier;

import static edu.wpi.first.units.Units.*;

/**
 * This should be deferred, use {@link PathfindHolonomicPID#generateCommand} as a constructor instead
 */
public class PathfindHolonomicPID extends Command {
    private static final Distance POSITION_TOLERANCE = Centimeters.of(1);
    private static final Angle ROTATION_TOLERANCE = Degrees.of(2);
    private static final LinearVelocity SPEED_TOLERANCE = MetersPerSecond.of(0.2);
    private final CommandSwerveDrivetrain drivetrain;
    /**
     * Swerve request to apply during robot-centric path following
     */
    private static final SwerveRequest.ApplyRobotSpeeds m_pathApplyRobotSpeeds = new SwerveRequest.ApplyRobotSpeeds();
    private final Pose2d goalPose;
    private final Trigger endTrigger;
    private final Trigger endTriggerDebounced;
    private final PPHolonomicDriveController driveController = new PPHolonomicDriveController(
            new PIDConstants(10, 0, 0),
            new PIDConstants(7, 0, 0)
    );


    public PathfindHolonomicPID(CommandSwerveDrivetrain drivetrain, Pose2d goalPose) {
        this.drivetrain = drivetrain;
        this.goalPose = goalPose;
        this.endTrigger = new Trigger(() -> {
            Pose2d diff = drivetrain.getPose().relativeTo(goalPose);
            boolean rotation = MathUtil.isNear(
                    0.0,
                    diff.getRotation().getRotations(),
                    ROTATION_TOLERANCE.in(Rotations),
                    0.0,
                    1.0
            );
            boolean position = diff.getTranslation().getNorm() < POSITION_TOLERANCE.in(Meters);

            var speeds = drivetrain.getState().Speeds;
            boolean speed = Math.sqrt(Math.pow(speeds.vxMetersPerSecond, 2) + Math.pow(speeds.vyMetersPerSecond, 2)) < SPEED_TOLERANCE.in(MetersPerSecond);

            return rotation && position && speed;
        });
        endTriggerDebounced = endTrigger.debounce(0.1);
    }

    public static Command generateCommand(CommandSwerveDrivetrain drivetrain, Supplier<Pose2d> goalPoseSupplier, Time timeout) {
        return Commands.defer(() ->
                new PathfindHolonomicPID(drivetrain, goalPoseSupplier.get()).withTimeout(timeout).finallyDo(() ->
                        drivetrain.setControl(m_pathApplyRobotSpeeds.withSpeeds(new ChassisSpeeds()))), Set.of());

    }

    @Override
    public void execute() {
        PathPlannerTrajectoryState goalState = new PathPlannerTrajectoryState();
        goalState.pose = goalPose;

        drivetrain.setControl(m_pathApplyRobotSpeeds.withSpeeds(
                driveController.calculateRobotRelativeSpeeds(drivetrain.getPose(), goalState)
        ));
    }

    @Override
    public void end(boolean interrupted) {

    }

    @Override
    public boolean isFinished() {
        return endTriggerDebounced.getAsBoolean();
    }
}
