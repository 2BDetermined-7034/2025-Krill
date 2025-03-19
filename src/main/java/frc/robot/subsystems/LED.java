package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LED extends SubsystemBase {

	private final AddressableLED led1 = new AddressableLED(0);
	private final AddressableLED led2 = new AddressableLED(1);

	private final AddressableLEDBuffer ledBuffer1 = new AddressableLEDBuffer(18);
	private final AddressableLEDBuffer ledBuffer2 = new AddressableLEDBuffer(18);



	public LED() {
		led1.setData(ledBuffer1);
		led2.setData(ledBuffer1);
		led1.setLength(ledBuffer1.getLength());
		led2.setLength(ledBuffer1.getLength());

		led1.start();
		led2.start();


		setDefaultCommand(runPattern(LEDPattern.solid(Color.kGold),LEDPattern.solid(Color.kDarkGreen)));
	}

	@Override
	public void periodic() {
		led1.setData(ledBuffer1);
		led2.setData(ledBuffer1);
	}

	public Command runPattern(LEDPattern pattern1, LEDPattern pattern2) {
		return Commands.parallel(run( () -> pattern1.applyTo(ledBuffer1)), run( () -> pattern2.applyTo(ledBuffer2)));
	}
}
