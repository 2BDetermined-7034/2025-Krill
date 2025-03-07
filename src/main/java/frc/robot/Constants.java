package frc.robot;

import edu.wpi.first.units.measure.*;

import static edu.wpi.first.units.Units.*;

public class Constants {
	public static final class Arm {
		public static final int ARM_MOTOR_ID = 17;
		public static final int INTAKE_MOTOR_ID = 16;
		public static final int CANCODER_ID = 15;

		public static final double CANCODER_OFFSET = 0.324462890625;

		public static final double GEAR_RATIO = 9.0;

		public static final Current ARM_CURRENT_LIMIT = Amps.of(60.0);
	}

	public static final class Elevator {
		public static final int MASTER_MOTOR_ID = 13;
		public static final int SLAVE_MOTOR_ID = 12;
		public static final int CANCODER_ID = 14;

		public static final double MOTOR_TO_SENSOR = 4.0;
		public static final double SENSOR_TO_MECHANISM = 1.0;

		public static final Current CURRENT_LIMIT = Amps.of(80.0);

	}

	public static final class Climb {
		public static final int CLIMB_MOTOR_ID = 0;
		public static final Current CURRENT_LIMIT = Amps.of(100);
		public static final Angle CLIMB_TOLERANCE = Rotations.of(0.1);
	}
}
