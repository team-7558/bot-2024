package frc.robot.subsystems.LED;

import edu.wpi.first.wpilibj.simulation.AddressableLEDSim;

public class LEDIOSim implements LEDIO {
  public AddressableLEDSim m_led = new AddressableLEDSim();
  public byte[] m_ledBuffer = new byte[36 * 3];

  public LEDIOSim() {
    AddressableLEDSim.createForChannel(14);
    m_led.setLength(36 * 3);
    m_led.setData(m_ledBuffer);
    // m_led.start();
  }

  @Override
  public void updateInputs(LEDIOInputs inputs) {}

  @Override
  public void setColours(boolean isRGB, int[] leds) {
    for (int i = 0; i < leds.length; i++) {
        m_ledBuffer[i] = (byte) leds[i];
    }
    m_led.setData(m_ledBuffer);
  }
}
