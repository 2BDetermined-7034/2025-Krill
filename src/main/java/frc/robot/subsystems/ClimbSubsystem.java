package frc.robot.subsystems;

import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
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

	public enum ClimbPositions {
		HOME(Rotations.of(0.0)),
		MATCH_START(Rotations.of(0.0)),
		EXTENDED(Rotations.of(0.0)),
		FINAL(Rotations.of(0.0));

		private final Angle climbPosition;

		ClimbPositions(Angle climbPosition) {
			this.climbPosition = climbPosition;
		}

		public Angle getAngle() {
			return climbPosition;
		}
	}

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

	}

	public Command Climb(Voltage voltageOut) {
		return new FunctionalCommand(
			() -> climbMotor.setControl(new VoltageOut(voltageOut)),
			() -> climbMotor.setControl(new VoltageOut(voltageOut)),
			(interrupted) -> climbMotor.setControl(new VoltageOut(Volts.of(0.0))),
			() -> false
		);
	}

	public Command climbToPosition(Angle angle) {
		return new FunctionalCommand(
			() -> climbMotor.setControl(new MotionMagicVoltage(angle)),
			() -> {},
			(interrupted) -> {},
			() -> climbMotor.getPosition().getValue().isNear(angle, Climb.CLIMB_TOLERANCE)
		);
	}

	public Command climbToPosition(ClimbPositions climbPosition) {
		return climbToPosition(climbPosition.getAngle());
	}

}
