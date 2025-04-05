package frc.robot;

import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.units.measure.*;

import static edu.wpi.first.units.Units.*;

public class Constants {
    public static final class DrivebaseOnTheFly {
        public static final PathConstraints OTF_CONSTRAINTS = new PathConstraints(
                MetersPerSecond.of(2), MetersPerSecondPerSecond.of(2.3),
                DegreesPerSecond.of(720), DegreesPerSecondPerSecond.of(720));
      
        public static final Time OTF_TIMEOUT = Seconds.of(1.5);

        public static final LinearVelocity GOAL_END_VELOCITY = MetersPerSecond.of(0.2);
    }

    public static final class Arm {
        public static final int ARM_MOTOR_ID = 17;
        public static final int INTAKE_MOTOR_ID = 16;
        public static final int CANCODER_ID = 15;

        public static final double CANCODER_OFFSET = -0.410400390625;

        public static final double GEAR_RATIO = 9.0;

        public static final Current ARM_CURRENT_LIMIT = Amps.of(60.0);
        public static final Angle HOME_POSITION = Degrees.of(-24.2);
        public static final Current INTAKE_CURRENT = Amps.of(50);
        public static final Voltage INTAKE_VOLTAGE = Volts.of(5.0);
        public static final AngularVelocity INTAKE_SPEED = RotationsPerSecond.of(20);
    }

    public static final class Elevator {
        public static final int MASTER_MOTOR_ID = 13;
        public static final int SLAVE_MOTOR_ID = 12;
        public static final int CANCODER_ID = 14;

        public static final double MOTOR_TO_SENSOR = 4.0;
        public static final double SENSOR_TO_MECHANISM = 1.0;

        public static final Current CURRENT_LIMIT = Amps.of(80.0);

        public static final Angle AUTO_RETRACT_THRESHOLD = Degrees.of(10);


    }

    public static final class Climb {
        public static final int CLIMB_MOTOR_ID = 0;
        public static final Current CURRENT_LIMIT = Amps.of(100);
        public static final Angle CLIMB_TOLERANCE = Rotations.of(0.1);

        public enum ClimbDirection {
            POSITIVE,
            NEGATIVE
        }

        public static final Voltage CLIMB_VOLTAGE_FORWARD = Volts.of(10.0);
        public static final Voltage CLIMB_VOLTAGE_BACKWARDS= Volts.of(-10.0);

        public enum ClimbPositions {
            CLIMB_HOME(Rotations.of(0)),
            EXTENDED(Rotations.of(4.5)),
            RETRACTED(Rotations.of(-3));

            private final Angle climbPosition;


            ClimbPositions(Angle climbPosition) {
                this.climbPosition = climbPosition;
            }

            public Angle getAngle() {
                return climbPosition;
            }
        }
    }
}
