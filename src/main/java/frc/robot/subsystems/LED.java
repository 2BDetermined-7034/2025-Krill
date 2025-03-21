package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static edu.wpi.first.units.Units.Rotations;

public class LED extends SubsystemBase {
	private static final int kLength = 18;
	private final AddressableLED m_led1;
	//private final AddressableLED m_led2;
	private final AddressableLEDBuffer m_buffer1;
	//private final AddressableLEDBuffer m_buffer2;
	private final LEDPattern GREEN_AND_GOLD;


	public LED(ElevatorSubsystem elevator) {
		GREEN_AND_GOLD = LEDPattern.gradient(LEDPattern.GradientType.kDiscontinuous, Color.kGold,Color.kGreen);
		m_led1 = new AddressableLED(0);
		//m_led2 = new AddressableLED(1);

		m_buffer1 = new AddressableLEDBuffer(kLength);
		//m_buffer2 = new AddressableLEDBuffer(kLength);
		m_led1.setLength(kLength);
		//m_led2.setLength(kLength);
		m_led1.start();
		//m_led2.start();

		// Set the default command to turn the strip off, otherwise the last colors written by
		// the last command to run will continue to be displayed.
		//setDefaultCommand(runMaskPattern(GREEN_AND_GOLD,elevator));
		setDefaultCommand(runPattern(GREEN_AND_GOLD));
	}

	@Override
	public void periodic() {
		// Periodically send the latest LED color data to the LED strip for it to display
		//GREEN_AND_GOLD.applyTo(m_buffer2);
		m_led1.setData(m_buffer1);
		//m_led2.setData(m_buffer2);
	}

	/**
	 * Creates a command that runs a pattern on the entire LED strip.
	 *
	 * @param pattern the LED pattern to run
	 */
	public Command runPattern(LEDPattern pattern) {
		return run(()->pattern.applyTo(m_buffer1));
	}

	public Command runMaskPattern(LEDPattern pattern, ElevatorSubsystem elevator){
		LEDPattern mask = LEDPattern.progressMaskLayer(() -> elevator.getElevatorAngle().in(Rotations)/(ElevatorSubsystem.ElevatorPosition.L4.getAngle()).in(Rotations));
		LEDPattern heightDisplay = pattern.mask(mask);
		return runPattern(heightDisplay);
	}
}