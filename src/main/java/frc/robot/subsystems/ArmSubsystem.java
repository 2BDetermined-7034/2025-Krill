package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.*;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.*;
import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.Arm.*;

public class ArmSubsystem extends SubsystemBase {
	private TalonFX motor;
	private CANcoder canCoder;

	public ArmSubsystem() {
		motor = new TalonFX(MOTOR_ID, "rio");
		canCoder = new CANcoder(CANCODER_ID, "rio");


		var mConfig = new TalonFXConfiguration();
		mConfig.Feedback.FeedbackRemoteSensorID = CANCODER_ID;
		mConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.SyncCANcoder;
		mConfig.Feedback.RotorToSensorRatio = GEAR_RATIO;
		mConfig.Feedback.SensorToMechanismRatio = 1.0;
		mConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
		mConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
		mConfig.CurrentLimits.SupplyCurrentLimit = CURRENT_LIMIT.in(Units.Amps);
		motor.getConfigurator().apply(mConfig);

		Slot0Configs slotConfigs = new Slot0Configs();
		slotConfigs.GravityType = GravityTypeValue.Arm_Cosine;
		slotConfigs.StaticFeedforwardSign = StaticFeedforwardSignValue.UseVelocitySign;
		slotConfigs.kA = 0;
		slotConfigs.kG = 0.4;
		slotConfigs.kV = 0;
		slotConfigs.kS = 0.35;
		slotConfigs.kP = 14;
		slotConfigs.kI = 0;
		slotConfigs.kD = 0;
		motor.getConfigurator().apply(slotConfigs);

		var ccConfig = new CANcoderConfiguration();
		ccConfig.MagnetSensor.MagnetOffset = CANCODER_OFFSET;
		ccConfig.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;
		canCoder.getConfigurator().apply(ccConfig);

	}

	public Angle getPosition() {
		return motor.getPosition().getValue();
	}

	public Command coastOutCommand() {
		return Commands.runOnce(
			() -> motor.setControl(new CoastOut())
		);
	}

	private Angle clamp(Angle angle, Angle min, Angle max) {
		final double rotationsAngle = angle.in(Units.Rotations);
		if (rotationsAngle < min.in(Units.Rotations)) return min;
		else if (rotationsAngle > max.in(Units.Rotations)) return max;
		return angle;
	}

	public Command setPositionCommand(Angle position) {
		return Commands.runOnce(
			() -> motor.setControl(new PositionVoltage(clamp(position, MIN_POSITION, MAX_POSITION)))
		);
	}

//	public Command sysIDQuasistatic(SysIdRoutine.Direction direction) {
//		return routine.quasistatic(direction);
//	}
//
//	public Command sysIDDynamic(SysIdRoutine.Direction direction) {
//		return routine.dynamic(direction);
//	}
}
