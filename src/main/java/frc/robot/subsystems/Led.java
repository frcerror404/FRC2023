package frc.robot.subsystems;

import java.lang.reflect.Array;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Led extends SubsystemBase {
    private final AddressableLED m_led = new AddressableLED(9);
    private final AddressableLEDBuffer m_ledBuffer = new AddressableLEDBuffer(7);
    private int m_rainbowFirstPixelHue = 0;
    private int[] yellowRGB = {245, 252, 63};
    private int[] purpleRGB = {255, 0, 255};

    public Led() {
        m_led.setLength(m_ledBuffer.getLength());

        for (var i = 0; i < m_ledBuffer.getLength(); i++) {
            m_ledBuffer.setRGB(i, 255, 0,0);
        }
        m_led.setData(m_ledBuffer);
        m_led.start();
    }

    @Override
  public void periodic() {
    yellowPurpleLed();
    m_led.setData(m_ledBuffer);
    SmartDashboard.putNumber("LedBuffer", m_ledBuffer.getLength());
  }

    public void rainbowLed() {
        for (var i = 0; i < m_ledBuffer.getLength(); i++) {
            final var hue = (m_rainbowFirstPixelHue + (i * 180 / m_ledBuffer.getLength())) % 180;
            m_ledBuffer.setHSV(i, hue, 255, 128);
        }

        m_rainbowFirstPixelHue += 3;
    }

    public void yellowPurpleLed() {
        for (var i = 0; i < m_ledBuffer.getLength(); i++) {
            m_ledBuffer.setRGB(i, yellowRGB[0], yellowRGB[1], yellowRGB[2]);
        }
    }

    public AddressableLED getLedController() {
        return m_led;
    }
}
