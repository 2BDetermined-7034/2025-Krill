package frc.robot.subsystems;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.*;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

import java.util.Set;

import static edu.wpi.first.units.Units.*;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.Constants.Elevator.*;

public class ElevatorSubsystem extends SubsystemBase {
	private final SysIdRoutine routine;
	private final TalonFX masterMotor, slaveMotor;
	private final CANcoder canCoder;
	private VoltageOut voltageControl;
	private MotionMagicVoltage forwardMotionMagic;
	private MotionMagicVoltage reverseMotionMagic;

	public ElevatorSubsystem() {
		masterMotor = new TalonFX(MASTER_MOTOR_ID, "Drivebase");
		slaveMotor = new TalonFX(SLAVE_MOTOR_ID, "Drivebase");

		slaveMotor.setControl(new Follower(MASTER_MOTOR_ID, true));

		TalonFXConfiguration talonFXConfigs = new TalonFXConfiguration();

		Slot0Configs motorConfig = talonFXConfigs.Slot0;
		motorConfig.GravityType = GravityTypeValue.Elevator_Static;
		motorConfig.StaticFeedforwardSign = StaticFeedforwardSignValue.UseVelocitySign;
		motorConfig.kS = 0.4;
		motorConfig.kV = 1.2;
		motorConfig.kA = 0.2;
		motorConfig.kG = 0.45;
		motorConfig.kP = 0.0;
		motorConfig.kI = 0.0;
		motorConfig.kD = 0.0;

		MotorOutputConfigs moConfig = talonFXConfigs.MotorOutput;
		moConfig.Inverted = InvertedValue.CounterClockwise_Positive;
		moConfig.NeutralMode = NeutralModeValue.Brake;

		FeedbackConfigs ffConfig = talonFXConfigs.Feedback;;
		ffConfig.FeedbackRemoteSensorID = CANCODER_ID;
		ffConfig.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
		ffConfig.RotorToSensorRatio = MOTOR_TO_SENSOR;
		ffConfig.SensorToMechanismRatio = SENSOR_TO_MECHANISM;
		ffConfig.FeedbackRotorOffset = 0.0;

		var motionMagicConfigs = talonFXConfigs.MotionMagic;
		motionMagicConfigs.MotionMagicCruiseVelocity = 6;
		motionMagicConfigs.MotionMagicAcceleration = 15;
//		motionMagicConfis.MotionMagicJerk = 0;

		masterMotor.getConfigurator().apply(talonFXConfigs);



		canCoder = new CANcoder(CANCODER_ID, "Drivebase");

		var ccConfig = new CANcoderConfiguration();
		ccConfig.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
		ccConfig.MagnetSensor.MagnetOffset = -0.060546875;
		canCoder.getConfigurator().apply(ccConfig);

		var rampRate = Volts.of(1.0).div(Seconds.of(1));
		routine = new SysIdRoutine(
			new SysIdRoutine.Config(
				rampRate,
				Volts.of(4.0),
				Seconds.of(10),
				(state) -> {
					SignalLogger.writeString("Elevator state", state.toString());
				}
			),
			new SysIdRoutine.Mechanism(
				(volts) -> {
					masterMotor.setControl(new VoltageOut(volts));
				},
				null,
//				(log) -> {
//					log.motor("elevator")
//						.voltage(masterMotor.getMotorVoltage().getValue())
//						.angularPosition(masterMotor.getPosition().getValue())
//						.angularVelocity(masterMotor.getVelocity().getValue());
//				},
				this
			)
		);

		forwardMotionMagic = new MotionMagicVoltage(0).withSlot(0);
		reverseMotionMagic = new MotionMagicVoltage(0).withSlot(1);

	}

	@Override
	public void periodic() {
		SmartDashboard.putNumber("Elevator Angle Rotations", getElevatorAngle().in(Rotations));
		SmartDashboard.putNumber("Elevator Angle Degrees", getElevatorAngle().in(Degrees));
		SmartDashboard.putNumber("Elevator Distance Meters", getElevatorPosition().in(Meters));
	}

	public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
		return routine.quasistatic(direction).until(
			() -> direction.equals(SysIdRoutine.Direction.kForward)
				? getElevatorPosition().gt(Meters.of(0.8))
				: getElevatorPosition().lt(Meters.of(0.1))
		);
	}

	public Command sysIdDynamic(SysIdRoutine.Direction direction) {
		return routine.dynamic(direction).until(
			() -> direction.equals(SysIdRoutine.Direction.kForward)
				? getElevatorPosition().gt(Meters.of(0.8))
				: getElevatorPosition().lt(Meters.of(0.1))
		);
	}

	public Angle getElevatorAngle() {
		return masterMotor.getPosition().getValue();
	}

	public Distance getElevatorPosition() {
		return SPOOL_RADIUS.times(getElevatorAngle().in(Radians));
	}

	/**
	 * Needs to be deferred
	 * @param angle
	 * @return
	 */
	public Command setElevatorPosition(Angle angle) {

		return new FunctionalCommand(
			() -> {
				MotionMagicVoltage motionMagicVoltage;
				Angle currentPosition = getElevatorAngle();
				if(currentPosition.gt(angle)) motionMagicVoltage = reverseMotionMagic.withPosition(angle);
				else motionMagicVoltage = forwardMotionMagic.withPosition(angle);
				masterMotor.setControl(motionMagicVoltage);
			},
			() -> {},
			(interrupted) -> {},
			() -> false
		);
	}

}