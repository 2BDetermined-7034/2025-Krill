package frc.robot.commands.elevator;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;

import java.awt.*;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;

public class RaiseElevator extends Command {

	private final TrapezoidProfile profile = new TrapezoidProfile(
		new TrapezoidProfile.Constraints(4, 8)
	);

	private TrapezoidProfile.State goal;

	private final MotionMagicVoltage request;

	public RaiseElevator(Angle positionSetpoint, AngularVelocity velocitySetpoint) {
		request = new MotionMagicVoltage(0).withSlot(0);
		goal = new TrapezoidProfile.State(positionSetpoint.in(Radians), velocitySetpoint.in(RadiansPerSecond));



		TrapezoidProfile.State setpoint = new TrapezoidProfile.State();
	}
}
