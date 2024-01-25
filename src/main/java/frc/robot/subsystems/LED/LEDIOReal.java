package frc.robot.subsystems.LED;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;

public class LEDIOReal implements LEDIO{
    private AddressableLED m_led = new AddressableLED(14);
    private AddressableLEDBuffer m_ledBuffer = new AddressableLEDBuffer(36);

    public LEDIOReal() {
        m_led.setLength(m_ledBuffer.getLength());
        m_led.setData(m_ledBuffer);
        m_led.start();
    }

    @Override
    public void updateInputs(LEDIOInputs inputs) {}

    @Override
    public void setColours(boolean isRGB, int[] leds) {
        for (int i=0; i<m_ledBuffer.getLength(); i++) {
            if (isRGB) m_ledBuffer.setHSV(i, leds[i*3 + 0], leds[i*3 + 1], leds[i*3 + 2]);
            if (!isRGB) m_ledBuffer.setRGB(i, leds[i*3 + 0], leds[i*3 + 1], leds[i*3 + 2]);
        }
    m_led.setData(m_ledBuffer);
    }
}
