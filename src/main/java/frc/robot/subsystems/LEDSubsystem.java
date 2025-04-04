package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDSubsystem extends SubsystemBase {

	private static final int PORT = 1;
	private static final int LENGTH = 36;

	private final AddressableLED led;
	private final AddressableLEDBuffer buffer;

	public LEDSubsystem() {
		this.led = new AddressableLED(PORT);
		this.buffer = new AddressableLEDBuffer(LENGTH);
		led.setLength(LENGTH);
		led.setData(buffer);
		led.start();
	}

	@Override
	public void periodic() {
		LEDPattern.solid(Color.kGold).applyTo(buffer);
		led.setData(buffer);
	}
}
