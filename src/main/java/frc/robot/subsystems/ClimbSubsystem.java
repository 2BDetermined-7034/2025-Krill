package frc.robot.subsystems;

import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.*;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static edu.wpi.first.units.Units.*;
import static frc.robot.Constants.Climb;





public class ClimbSubsystem extends SubsystemBase {
	TalonFX climbMotor = new TalonFX(Climb.CLIMB_MOTOR_ID, "rio");

	public ClimbSubsystem() {

		TalonFXConfiguration talonFXConfigs = new TalonFXConfiguration();

		CurrentLimitsConfigs currentLimitsConfigs = talonFXConfigs.CurrentLimits;
		currentLimitsConfigs.SupplyCurrentLimit = Climb.CURRENT_LIMIT.in(Amps);
		currentLimitsConfigs.StatorCurrentLimitEnable = true;

		MotorOutputConfigs motorOutputConfigs = talonFXConfigs.MotorOutput;
		motorOutputConfigs.NeutralMode = NeutralModeValue.Brake;
		motorOutputConfigs.Inverted = InvertedValue.Clockwise_Positive;

		FeedbackConfigs ffConfigs = talonFXConfigs.Feedback;
		ffConfigs.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
		ffConfigs.RotorToSensorRatio = 1;
		ffConfigs.SensorToMechanismRatio = 25;

		Slot0Configs slot0Configs = talonFXConfigs.Slot0;
		slot0Configs.StaticFeedforwardSign = StaticFeedforwardSignValue.UseVelocitySign;
		slot0Configs.GravityType = GravityTypeValue.Arm_Cosine;
		slot0Configs.kS = 0;
		slot0Configs.kG = 0;
		slot0Configs.kV = 0;
		slot0Configs.kA = 0;
		slot0Configs.kP = 0;
		slot0Configs.kI = 0;
		slot0Configs.kD = 0;

		climbMotor.getConfigurator().apply(talonFXConfigs);
		climbMotor.setPosition(Rotations.of(0));

	}

	public Angle getClimbPosition() {
		return climbMotor.getPosition().getValue();
	}

	/**
	 * Returns a command which supplies the climb motor with a constant voltage
	 *
	 * @param voltageOut the voltage to supply
	 * @return the command to run
	 */
	public Command setClimbVoltage(Voltage voltageOut) {
		return new FunctionalCommand(
			() -> climbMotor.setControl(new VoltageOut(voltageOut)),
			() -> climbMotor.setControl(new VoltageOut(voltageOut)),
			(interrupted) -> climbMotor.setControl(new VoltageOut(Volts.of(0.0))),
			() -> false,
			this
		);
	}

	public Command climbUntil(Climb.ClimbDirection direction, Angle angle, Voltage voltage) {
		return new FunctionalCommand(
			() -> climbMotor.setControl((new VoltageOut(voltage))),
			() -> {
			},
			(interrupted) -> climbMotor.setControl(new VoltageOut(0.0)),
			() -> {
				if (direction == Climb.ClimbDirection.POSITIVE && getClimbPosition().gt(angle)) {
					return true;
				} else if (direction == Climb.ClimbDirection.NEGATIVE && getClimbPosition().lt(angle)) {
					return true;
				}
				return false;

			},
			this
		);
	}

	public Command climbUntil(Climb.ClimbDirection direction, Climb.ClimbPositions position, Voltage voltage) {
		return climbUntil(direction, position.getAngle(), voltage);
	}


}
