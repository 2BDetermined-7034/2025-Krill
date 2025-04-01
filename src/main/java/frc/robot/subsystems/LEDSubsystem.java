package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.ElevatorSubsystem.ElevatorPosition;

import static edu.wpi.first.units.Units.Rotations;

public class LEDSubsystem extends SubsystemBase {
	private static final int LENGTH = 36;
	private static final int PORT = 1;

	private static final LEDPattern SOLID_PATTERN = LEDPattern.solid(Color.kGreen);

	private final AddressableLED led;
	private final AddressableLEDBuffer buffer1;



	public LEDSubsystem() {
		this.led = new AddressableLED(PORT);
		this.buffer1 = new AddressableLEDBuffer(LENGTH);
		led.setLength(buffer1.getLength());
		led.start();

		setDefaultCommand(runPattern(SOLID_PATTERN));
	}

	@Override
	public void periodic() {
		// Periodically send the latest LED color data to the LED strip for it to display
		led.setData(buffer1);
	}

	/**
	 * Creates a command that runs a pattern on the entire LED strip.
	 *
	 * @param pattern the LED pattern to run
	 */
	public Command runPattern(LEDPattern pattern) {
		return run(()->pattern.applyTo(buffer1));
	}

	public Command runMaskPattern(LEDPattern pattern, ElevatorSubsystem elevator){
		LEDPattern mask = LEDPattern.progressMaskLayer(() -> elevator.getElevatorAngle().in(Rotations)/(ElevatorPosition.L4.getAngle()).in(Rotations));
		LEDPattern heightDisplay = pattern.mask(mask);
		return runPattern(heightDisplay);
	}
}