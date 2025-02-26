package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.Constants.Climb;

public class ClimbSubsystem extends SubsystemBase {
	TalonFX climbMotor = new TalonFX(Climb.CLIMB_MOTOR_ID, "Drivebase");

	public ClimbSubsystem() {
		TalonFXConfiguration talonFXConfigs = new TalonFXConfiguration();

		CurrentLimitsConfigs currentLimitsConfigs = talonFXConfigs.CurrentLimits;
		currentLimitsConfigs.SupplyCurrentLimit = Climb.CURRENT_LIMIT.in(Amps);
		currentLimitsConfigs.StatorCurrentLimitEnable = true;

		MotorOutputConfigs motorOutputConfigs = talonFXConfigs.MotorOutput;
		motorOutputConfigs.NeutralMode = NeutralModeValue.Brake;
		motorOutputConfigs.Inverted = InvertedValue.CounterClockwise_Positive;

		climbMotor.getConfigurator().apply(talonFXConfigs);

	}

	public Command Climb(Voltage voltageOut) {
		return new FunctionalCommand(
			() -> climbMotor.setControl(new VoltageOut(voltageOut)),
			() -> {},
			(interrupted) -> climbMotor.setControl(new VoltageOut(Volts.of(0.0))),
			() -> false
		);
	}

}
