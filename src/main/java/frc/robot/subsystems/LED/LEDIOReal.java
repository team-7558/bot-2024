package frc.robot.subsystems.LED;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;

public class LEDIOReal implements LEDIO {
  public AddressableLED m_led = new AddressableLED(4);
  public AddressableLEDBuffer m_ledBuffer = new AddressableLEDBuffer(LED.NUM_LEDS);

  public LEDIOReal() {
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
}
