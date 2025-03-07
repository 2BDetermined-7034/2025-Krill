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
	private final TalonFX armMotor;
	private final TalonFX intakeMotor;
	private final CANcoder canCoder;


	public enum ScoringPosition {
		Outtake(Degrees.of(-20.0)),
		OuttakeFlick(Degrees.of(0)),
		IntakeCoralStation(Degrees.of(40));
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

		var talonFXConfigs = new TalonFXConfiguration();


		Slot0Configs slotConfigs = talonFXConfigs.Slot0;
		slotConfigs.GravityType = GravityTypeValue.Arm_Cosine;
		slotConfigs.StaticFeedforwardSign = StaticFeedforwardSignValue.UseVelocitySign;
		slotConfigs.kA = 0.01;
		slotConfigs.kG = 0.40;
		slotConfigs.kV = 0.44;
		slotConfigs.kS = 0.35;
		slotConfigs.kP = 10;
		slotConfigs.kI = 0;
		slotConfigs.kD = 1;

		var motionMagicConfigs = talonFXConfigs.MotionMagic;
		motionMagicConfigs.MotionMagicCruiseVelocity = 1.5;
		motionMagicConfigs.MotionMagicAcceleration = 3;
		motionMagicConfigs.MotionMagicJerk = 0;

		talonFXConfigs.Feedback.FeedbackRemoteSensorID = CANCODER_ID;
		talonFXConfigs.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.SyncCANcoder;
		talonFXConfigs.Feedback.RotorToSensorRatio = GEAR_RATIO;
		talonFXConfigs.Feedback.SensorToMechanismRatio = 1.0;
		talonFXConfigs.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
		talonFXConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;
		talonFXConfigs.CurrentLimits.SupplyCurrentLimit = ARM_CURRENT_LIMIT.in(Amps);
		armMotor.getConfigurator().apply(talonFXConfigs);


//		var ccConfig = new CANcoderConfiguration();
////		ccConfig.MagnetSensor.MagnetOffset = CANCODER_OFFSET - /*Degrees.of(24.2).in(Rotations)*/;
//		ccConfig.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;
//		canCoder.getConfigurator().apply(ccConfig);

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
			() -> armMotor.setControl(new MotionMagicVoltage(position)),
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
