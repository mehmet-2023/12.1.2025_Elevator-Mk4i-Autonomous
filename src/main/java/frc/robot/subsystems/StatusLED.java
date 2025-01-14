
package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LedSubsystem;
import frc.robot.Constants.OI;


public class StatusLED extends SubsystemBase {
	public AddressableLED led;
	public AddressableLEDBuffer buffer;
	public LEDPattern pattern;

	public StatusLED() {
		led = new AddressableLED(0);//PWM Port
		buffer = new AddressableLEDBuffer(0);//Led Length
	}

	public void setDefault(){
		pattern = LedSubsystem.BREATHE_COLOR.breathe(Seconds.of(2));
		pattern.applyTo(buffer);
		led.setData(buffer);
	}

	public void SetProcess(){
		LedSubsystem.ELEVATOR_PROCESS_COLOR.applyTo(buffer);
		led.setData(buffer);
	}
	
	public void setFocus(){
		LedSubsystem.TARGET_FOCUS_COLOR.applyTo(buffer);
		led.setData(buffer);
	}

	public void CheckForProcess(){
		if(OI.IS_PROCESSING){
			SetProcess();
		}else if(OI.IS_SWERVE_FOCUSED){
			setFocus();
		}else{
			setDefault();
		}
	}
	@Override
	public void periodic() {
		CheckForProcess();
	}
}