package frc.robot.subsystems;

import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.*;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import static edu.wpi.first.units.Units.*;
import static frc.robot.Constants.Elevator.*;

public class ElevatorSubsystem extends SubsystemBase {
	private final TalonFX masterMotor, slaveMotor;
	private final CANcoder canCoder;

	public enum ScoringPosition {
		HOME(Rotations.of(0)),
		L1(Rotations.of(0.103027)),
		INTAKE(Rotations.of(0.1)),
		L2(Rotations.of(0.553223)),
		L3(Rotations.of(1.222168)),
		L4(Rotations.of(2.24));

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

		Slot0Configs upConfig = talonFXConfigs.Slot0;
		upConfig.GravityType = GravityTypeValue.Elevator_Static;
		upConfig.StaticFeedforwardSign = StaticFeedforwardSignValue.UseVelocitySign;
		upConfig.kS = 0.38;
		upConfig.kV = 0.45;
		upConfig.kA = 0.0;
		upConfig.kG = 0.45;
		upConfig.kP = 3;
		upConfig.kI = 0.7;
		upConfig.kD = 0.5;

		Slot1Configs downConfig = talonFXConfigs.Slot1;
		downConfig.GravityType = GravityTypeValue.Elevator_Static;
		downConfig.StaticFeedforwardSign = StaticFeedforwardSignValue.UseVelocitySign;
		downConfig.kS = 0.38;
		downConfig.kV = 0.3;
		downConfig.kA = 0.0;
		downConfig.kG = 0.65;
		downConfig.kP = 5;
		downConfig.kI = 0.0;
		downConfig.kD = 1;

		Slot2Configs level4Config = talonFXConfigs.Slot2;
		level4Config.GravityType = GravityTypeValue.Elevator_Static;
		level4Config.StaticFeedforwardSign = StaticFeedforwardSignValue.UseVelocitySign;
		level4Config.kS = 0.5;
		level4Config.kV = 0.45;
		level4Config.kA = 0.0;
		level4Config.kG = 0.45;
		level4Config.kP = 3;
		level4Config.kI = 0.7;
		level4Config.kD = 0.5;


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
		ccConfig.MagnetSensor.MagnetOffset = -0.1845703125;
		canCoder.getConfigurator().apply(ccConfig);

		new Trigger(DriverStation::isEnabled).onTrue(Commands.runOnce(() -> masterMotor.setControl(new CoastOut())));

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
			() -> {
				if (angle.equals(ScoringPosition.L4.getAngle())) {
					masterMotor.setControl(new MotionMagicVoltage(angle).withSlot(2));
				} else if (angle.lt(getElevatorAngle())) {
					masterMotor.setControl(new MotionMagicVoltage(angle).withSlot(1));
				} else {
					masterMotor.setControl(new MotionMagicVoltage(angle).withSlot(0));
				}
			},
			() -> {},
			(interrupted) -> {},
			() -> true
		);
	}

	public Command setElevatorPosition(ScoringPosition scoringPosition) {
		return setElevatorPosition(scoringPosition.getAngle());
	}

}