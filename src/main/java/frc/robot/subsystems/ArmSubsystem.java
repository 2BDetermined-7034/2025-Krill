package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.*;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

import static frc.robot.Constants.Arm.*;

public class ArmSubsystem extends SubsystemBase {
	private TalonFX motor;
	private CANcoder canCoder;
	private SysIdRoutine routine;

	public ArmSubsystem() {
		motor = new TalonFX(MOTOR_ID, "rio");
		canCoder = new CANcoder(CANCODER_ID, "rio");

		Slot0Configs slotConfigs = new Slot0Configs();
		slotConfigs.GravityType = GravityTypeValue.Arm_Cosine;
		slotConfigs.StaticFeedforwardSign = StaticFeedforwardSignValue.UseVelocitySign;
		slotConfigs.kA = kA;
		slotConfigs.kG = kG;
		slotConfigs.kV = kV;
		slotConfigs.kS = kS;
		slotConfigs.kP = kP;
		slotConfigs.kI = kI;
		slotConfigs.kD = kD;

		var mConfig = new TalonFXConfiguration();
		mConfig.Feedback.FeedbackRemoteSensorID = CANCODER_ID;
		mConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.SyncCANcoder;
		mConfig.Feedback.RotorToSensorRatio = GEAR_RATIO;
		mConfig.Feedback.SensorToMechanismRatio = 1.0;
		mConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

		mConfig.CurrentLimits.SupplyCurrentLimit = CURRENT_LIMIT.in(Units.Amps);
		motor.getConfigurator().apply(mConfig);

		var ccConfig = new CANcoderConfiguration();
		ccConfig.MagnetSensor.MagnetOffset = CANCODER_OFFSET;
		ccConfig.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;
		canCoder.getConfigurator().apply(ccConfig);


		routine = new SysIdRoutine(
			new SysIdRoutine.Config(
				RAMP_RATE,
				STEP_CURRENT,
				TEST_DURATION
			),
			new SysIdRoutine.Mechanism(
				this::currentConsumer,
				this::logConsumer,
				this
			)
		);
	}

	private void currentConsumer(Voltage amps) {
		motor.setControl(new TorqueCurrentFOC(amps.in(Units.Volts)));
	}

	private void logConsumer(SysIdRoutineLog log) {
		log.motor("arm")
			.current(motor.getSupplyCurrent().getValue())
			.angularPosition(motor.getPosition().getValue())
			.angularVelocity(motor.getVelocity().getValue());
	}

	public void setPosition(Angle position) {
		motor.setControl(new PositionTorqueCurrentFOC(position));
	}

	public Angle getPosition() {
		return motor.getPosition().getValue();
	}

	public Command sysIDQuasistatic(SysIdRoutine.Direction direction) {
		return routine.quasistatic(direction);
	}

	public Command sysIDDynamic(SysIdRoutine.Direction direction) {
		return routine.dynamic(direction);
	}
}
