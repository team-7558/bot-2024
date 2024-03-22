package frc.robot.subsystems.LED;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.motorcontrol.Spark;

public class LEDIOReal implements LEDIO {
  private AddressableLED m_led = new AddressableLED(7);
  private AddressableLEDBuffer m_ledBuffer = new AddressableLEDBuffer(LED.NUM_LEDS);

  private Spark blinkin = new Spark(9);

  public LEDIOReal() {
    blinkin.setSafetyEnabled(false);
    m_led.setLength(m_ledBuffer.getLength());
    m_led.setData(m_ledBuffer);
    m_led.start();
  }

  @Override
  public void updateInputs(LEDIOInputs inputs) {}

  @Override
  public void setColours(boolean isRGB, int[] leds) {
    for (int i = 0; i < m_ledBuffer.getLength() * 3; i += 3) {
      if (isRGB) m_ledBuffer.setRGB(i / 3, leds[i + 0], leds[i + 1], leds[i + 2]);
      else m_ledBuffer.setHSV(i / 3, leds[i + 0], leds[i + 1], leds[i + 2]);
    }
    m_led.setData(m_ledBuffer);
  }

  @Override
  public void setBlinkin(double pwmOut) {
    blinkin.set(pwmOut);
  }
}
