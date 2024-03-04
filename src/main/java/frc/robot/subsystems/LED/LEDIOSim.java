package frc.robot.subsystems.LED;

import edu.wpi.first.wpilibj.simulation.AddressableLEDSim;

public class LEDIOSim implements LEDIO {
  public AddressableLEDSim m_led = new AddressableLEDSim();
  public byte[] m_ledBuffer = new byte[LED.NUM_LEDS * 3];

  public LEDIOSim() {
    // AddressableLEDSim.createForChannel(14);
    m_led.setLength(LED.NUM_LEDS * 3);
    m_led.setData(m_ledBuffer);
    // m_led.start();
  }

  @Override
  public void updateInputs(LEDIOInputs inputs) {}

  @Override
  public void setColours(boolean isRGB, int[] leds) {
    for (int i = 0; i < LED.NUM_LEDS * 3; i += 3) {
      m_ledBuffer[i] = (byte) leds[i];
      m_ledBuffer[i + 1] = (byte) leds[i + 1];
      m_ledBuffer[i + 2] = (byte) leds[i + 2];

      System.out.print(m_ledBuffer[i] + " " + m_ledBuffer[i + 1] + " " + m_ledBuffer[i + 2] + "|");
    }
    System.out.println();
    m_led.setData(m_ledBuffer);
  }
}
