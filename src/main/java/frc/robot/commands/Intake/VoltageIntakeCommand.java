package frc.robot.commands.Intake;

import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import edu.wpi.first.hal.HALUtil;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmSubsystem;

import static edu.wpi.first.units.Units.RotationsPerSecond;
import static frc.robot.Constants.Arm.INTAKE_CURRENT;
import static frc.robot.Constants.Arm.INTAKE_VOLTAGE;

public class VoltageIntakeCommand extends Command {
	private final ArmSubsystem arm;
	private boolean overcameInitial;
	private long time;
	private Voltage voltage;

	public VoltageIntakeCommand(ArmSubsystem arm, Voltage voltage) {
		this.arm = arm;
		this.voltage = voltage;
	}

	@Override
	public void initialize() {
		overcameInitial = false;
		arm.getIntakeMotor().setControl(new VoltageOut(voltage));
		time = 0L;
	}

	@Override
	public void execute() {
		if (arm.getIntakeMotor().getVelocity().getValue().in(Units.RotationsPerSecond) >= 10) {
			overcameInitial = true;
		}
	}

	@Override
	public void end(boolean interrupted) {
		arm.getIntakeMotor().setControl(new CoastOut());
	}

	@Override
	public boolean isFinished() {
		if (overcameInitial) {
			if (arm.getIntakeMotor().getVelocity().getValue().in(RotationsPerSecond) < 2.0) {
				if (time == 0.0) {
					time = HALUtil.getFPGATime();
				} else if (HALUtil.getFPGATime() - time >= 0.1e6) {
					time = 0L;
					return true;
				}
			} else {
				time = 0L;
			}
////			return arm.getIntakeMotor().getTorqueCurrent().getValue().gt(Amps.of(99));
////				&& arm.getIntakeMotor().getVelocity().getValue().abs(RotationsPerSecond) < 2;
		}

		return false;
	}
}
