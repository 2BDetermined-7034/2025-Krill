package frc.robot;

import edu.wpi.first.units.CurrentUnit;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.VoltageUnit;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.units.measure.Voltage;

import static edu.wpi.first.units.Units.*;

public class Constants {
	public static final class Arm {
		public static final int MOTOR_ID = 0;
		public static final int CANCODER_ID = 1;

		public static final double CANCODER_OFFSET = 0.4;

		public static final double GEAR_RATIO = 9.0;

		public static final Voltage STEP_CURRENT = Volts.of(5.0); // Amps
		public static final Time TEST_DURATION = Seconds.of(4.0);
		public static final Velocity<VoltageUnit> RAMP_RATE = STEP_CURRENT.div(TEST_DURATION); // Amps / sec

		public static final Current CURRENT_LIMIT = Amps.of(20.0);
		public static final double kA = 0.0;
		public static final double kG = 0.0;
		public static final double kV = 0.0;
		public static final double kS = 0.0;
		public static final double kP = 0.0;
		public static final double kD = 0.0;
		public static final double kI = 0.0;
	}
}
