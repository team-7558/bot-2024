package frc.robot.subsystems.LED;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;

public class LEDIOReal implements LEDIO{
    private AddressableLED m_led = new AddressableLED(14);
    private AddressableLEDBuffer m_ledBuffer = new AddressableLEDBuffer(36);

    @Override
    public void updateLED(LEDIOInputs) {
        m_led.setLength(m_ledBuffer.getLength());
        m_led.setData(m_ledBuffer);
        m_led.start();
    }

    @Override
    public void setColours(int[] leds) {
        for (int i=0; i<m_ledBuffer.getLength(); i++) {
            m_ledBuffer.setHSV(i, 100, 255, 128);
        }
    m_led.setData(m_ledBuffer);
    }
}
