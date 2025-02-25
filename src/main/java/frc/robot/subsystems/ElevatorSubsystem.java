package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.*;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.*;

import static edu.wpi.first.units.Units.*;
import static frc.robot.Constants.Elevator.*;

public class ElevatorSubsystem extends SubsystemBase {
	private final TalonFX masterMotor, slaveMotor;
	private final CANcoder canCoder;
	private final MotionMagicVoltage forwardMotionMagic;
	private final MotionMagicVoltage reverseMotionMagic;

	public enum ScoringPosition {
		HOME(Rotations.of(0)),
		L1(Rotations.of(0.103027)),
		L2(Rotations.of(0.541016)),
		L3(Rotations.of(1.2012)),
		L4(Rotations.of(2.15332));

		private final Angle scoringPosition;

		ScoringPosition(Angle scoringPosition) {
			this.scoringPosition = scoringPosition;
		}

		public Angle getAngle() {
			return scoringPosition;
		}
	}

	public ElevatorSubsystem() {
		masterMotor = new TalonFX(MASTER_MOTOR_ID, "Drivebase");
		slaveMotor = new TalonFX(SLAVE_MOTOR_ID, "Drivebase");

		slaveMotor.setControl(new Follower(MASTER_MOTOR_ID, true));

		TalonFXConfiguration talonFXConfigs = new TalonFXConfiguration();

		CurrentLimitsConfigs currentLimitsConfigs = talonFXConfigs.CurrentLimits;
		currentLimitsConfigs.SupplyCurrentLimit = CURRENT_LIMIT.in(Amps);
		currentLimitsConfigs.StatorCurrentLimitEnable = true;

		Slot0Configs slot0Config = talonFXConfigs.Slot0;
		slot0Config.GravityType = GravityTypeValue.Elevator_Static;
		slot0Config.StaticFeedforwardSign = StaticFeedforwardSignValue.UseVelocitySign;
		slot0Config.kS = 0.38;
		slot0Config.kV = 0.45;
		slot0Config.kA = 0.0;
		slot0Config.kG = 0.65;
		slot0Config.kP = 8;
		slot0Config.kI = 0.0;
		slot0Config.kD = 1.5;

		Slot1Configs slot1Config = talonFXConfigs.Slot1;
		slot1Config.GravityType = GravityTypeValue.Elevator_Static;
		slot1Config.StaticFeedforwardSign = StaticFeedforwardSignValue.UseVelocitySign;
		slot1Config.kS = 0.38;
		slot1Config.kV = 0.3;
		slot1Config.kA = 0.0;
		slot1Config.kG = 0.65;
		slot1Config.kP = 5;
		slot1Config.kI = 0.0;
		slot1Config.kD = 1;


		MotorOutputConfigs moConfig = talonFXConfigs.MotorOutput;
		moConfig.Inverted = InvertedValue.CounterClockwise_Positive;
		moConfig.NeutralMode = NeutralModeValue.Brake;

		FeedbackConfigs ffConfig = talonFXConfigs.Feedback;
		ffConfig.FeedbackRemoteSensorID = CANCODER_ID;
		ffConfig.FeedbackSensorSource = FeedbackSensorSourceValue.SyncCANcoder;
		ffConfig.RotorToSensorRatio = MOTOR_TO_SENSOR;
		ffConfig.SensorToMechanismRatio = SENSOR_TO_MECHANISM;
		ffConfig.FeedbackRotorOffset = 0.0;

		var motionMagicConfigs = talonFXConfigs.MotionMagic;
		motionMagicConfigs.MotionMagicCruiseVelocity = 6;
		motionMagicConfigs.MotionMagicAcceleration = 12;
		motionMagicConfigs.MotionMagicJerk = 0;

		masterMotor.getConfigurator().apply(talonFXConfigs);



		canCoder = new CANcoder(CANCODER_ID, "Drivebase");

		var ccConfig = new CANcoderConfiguration();
		ccConfig.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
		ccConfig.MagnetSensor.MagnetOffset = -0.186767578125;
		canCoder.getConfigurator().apply(ccConfig);

		forwardMotionMagic = new MotionMagicVoltage(0).withSlot(0);
		reverseMotionMagic = new MotionMagicVoltage(0).withSlot(1);

	}

	@Override
	public void periodic() {

	}


	public Angle getElevatorAngle() {
		return masterMotor.getPosition().getValue();
	}

	/**
	 *
	 * @param angle angle setpoint of the elevator
	 * @return the command
	 */
	public Command setElevatorPosition(Angle angle) {

		return new FunctionalCommand(
			() -> masterMotor.setControl(getElevatorAngle().lt(angle)
					? forwardMotionMagic.withPosition(angle)
					: reverseMotionMagic.withPosition(angle)),
			() -> {},
			(interrupted) -> {},
			() -> false
		);
	}

	public Command setElevatorPosition(ScoringPosition scoringPosition) {
		return setElevatorPosition(scoringPosition.getAngle());
	}

}