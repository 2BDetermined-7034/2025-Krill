package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.*;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.*;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static edu.wpi.first.units.Units.*;
import static frc.robot.Constants.Arm.*;

public class ArmSubsystem extends SubsystemBase {
	private TalonFX armMotor;
	private TalonFX intakeMotor;
	private CANcoder canCoder;


	public enum ScoringPosition {
		Outtake(Rotations.of(-0.056895)),
		IntakeCoralStation(Rotations.of(0.076));
		private final Angle armAngle;

		ScoringPosition(Angle armAngle) {
			this.armAngle = armAngle;
		}

		public Angle getArmAngle() {
			return armAngle;
		}
	}


	public ArmSubsystem() {
		armMotor = new TalonFX(ARM_MOTOR_ID, "rio");
		intakeMotor = new TalonFX(INTAKE_MOTOR_ID, "rio");
		canCoder = new CANcoder(CANCODER_ID, "rio");

		var talonFXConfiguration = new TalonFXConfiguration();


		Slot0Configs slotConfigs = talonFXConfiguration.Slot0;
		slotConfigs.GravityType = GravityTypeValue.Arm_Cosine;
		slotConfigs.StaticFeedforwardSign = StaticFeedforwardSignValue.UseVelocitySign;
		slotConfigs.kA = 0;
		slotConfigs.kG = 0.4;
		slotConfigs.kV = 0;
		slotConfigs.kS = 0.35;
		slotConfigs.kP = 24;
		slotConfigs.kI = 15;
		slotConfigs.kD = 1;

		talonFXConfiguration.Feedback.FeedbackRemoteSensorID = CANCODER_ID;
		talonFXConfiguration.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
		talonFXConfiguration.Feedback.RotorToSensorRatio = GEAR_RATIO;
		talonFXConfiguration.Feedback.SensorToMechanismRatio = 1.0;
		talonFXConfiguration.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
		talonFXConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Brake;
		talonFXConfiguration.CurrentLimits.SupplyCurrentLimit = ARM_CURRENT_LIMIT.in(Amps);
		armMotor.getConfigurator().apply(talonFXConfiguration);


		var ccConfig = new CANcoderConfiguration();
		ccConfig.MagnetSensor.MagnetOffset = CANCODER_OFFSET;
		ccConfig.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;
		canCoder.getConfigurator().apply(ccConfig);

	}

	public Angle getPosition() {
		return armMotor.getPosition().getValue();
	}

	public Command coastOutCommand() {
		return Commands.runOnce(
			() -> armMotor.setControl(new CoastOut())
		);
	}

	private Angle clamp(Angle angle, Angle min, Angle max) {
		final double rotationsAngle = angle.in(Units.Rotations);
		if (rotationsAngle < min.in(Units.Rotations)) return min;
		else if (rotationsAngle > max.in(Units.Rotations)) return max;
		return angle;
	}

	public Command setArmAngle(Angle position) {
		return new FunctionalCommand(
			() -> armMotor.setControl(new PositionVoltage(clamp(position, MIN_POSITION, MAX_POSITION))),
			() -> {},
			(interrupted) -> {},
			() -> true
		);
	}

	public TalonFX getIntakeMotor() {
		return intakeMotor;
	}

	public Command setArmAngle(ScoringPosition scoringPosition) {
		return setArmAngle(scoringPosition.getArmAngle());
	}


	/**
	 * Spins the intake with a timeout for autos
	 * @return
	 */
	public Command spinIntakeCommand() {
		return new FunctionalCommand(
			() -> intakeMotor.setControl(new VoltageOut(2.0)),
			() -> {},
			(interrupted) -> {},
			() -> false
		).withTimeout(Seconds.of(0.5));
	}
}
