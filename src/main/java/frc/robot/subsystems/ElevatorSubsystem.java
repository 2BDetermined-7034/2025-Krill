package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import edu.wpi.first.wpilibj.CAN;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

import static edu.wpi.first.units.Units.*;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.Constants.Elevator.*;

public class ElevatorSubsystem extends SubsystemBase {
	private final SysIdRoutine routine;
	private final TalonFX masterMotor, slaveMotor;
	private final CANcoder canCoder;
	private VoltageOut voltageControl;

	public ElevatorSubsystem() {
		masterMotor = new TalonFX(MASTER_MOTOR_ID);
		slaveMotor = new TalonFX(SLAVE_MOTOR_ID);

		slaveMotor.setControl(new Follower(MASTER_MOTOR_ID, true));


		Slot0Configs motorConfig = new Slot0Configs();
		motorConfig.GravityType = GravityTypeValue.Elevator_Static;
		motorConfig.StaticFeedforwardSign = StaticFeedforwardSignValue.UseVelocitySign;
		motorConfig.kS = 0.0;
		motorConfig.kV = 0.0;
		motorConfig.kA = 0.0;
		motorConfig.kG = 0.0;
		motorConfig.kP = 0.0;
		motorConfig.kI = 0.0;
		motorConfig.kD = 0.0;

		masterMotor.getConfigurator().apply(motorConfig);

		FeedbackConfigs ffConfig = new FeedbackConfigs();
		ffConfig.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
		ffConfig.RotorToSensorRatio = MOTOR_TO_SENSOR;
		ffConfig.SensorToMechanismRatio = SENSOR_TO_MECHANISM;
		ffConfig.FeedbackRotorOffset = 0.0;
		masterMotor.getConfigurator().apply(ffConfig);

		canCoder = new CANcoder(CANCODER_ID, "Drivebase");

		var ccConfig = new CANcoderConfiguration();
		ccConfig.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;

		var rampRate = Volts.of(8.0).div(Seconds.of(5));
		routine = new SysIdRoutine(
			new SysIdRoutine.Config(
				rampRate,
				Volts.of(8.0),
				Seconds.of(10),
				null
			),
			new SysIdRoutine.Mechanism(
				(volts) -> {
					masterMotor.setControl(new VoltageOut(volts));
				},
				(log) -> {
					log.motor("elevator")
						.voltage(masterMotor.getMotorVoltage().getValue())
						.angularPosition(masterMotor.getPosition().getValue())
						.angularVelocity(masterMotor.getVelocity().getValue());
				},
				this
			)
		);
	}

	public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
		return routine.quasistatic(direction);
	}

	public Command sysIdDynamic(SysIdRoutine.Direction direction) {
		return routine.dynamic(direction);
	}
}