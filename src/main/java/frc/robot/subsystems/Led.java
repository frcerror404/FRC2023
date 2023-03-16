package frc.robot.subsystems;

import java.util.concurrent.DelayQueue;
import java.util.concurrent.TimeUnit;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Led extends SubsystemBase {
    private final AddressableLED m_led = new AddressableLED(9);
    private final AddressableLEDBuffer m_ledBuffer = new AddressableLEDBuffer(7);
    private int m_rainbowFirstPixelHue = 0;
    public int[] yellowRGB = { 255, 255, 0 };
    public int[] purpleRGB = { 255, 0, 255 };
    public int[] offRGB = {0, 0, 0};
    private int setColorCondition = 0;
    public String color;

    public Led() {
        synchronized (m_ledBuffer){
            m_led.setLength(m_ledBuffer.getLength());
            m_led.setData(m_ledBuffer);
            m_led.start();
        }
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
            // m_ledBuffer.setRGB(i, yellowRGB[0], yellowRGB[1], yellowRGB[2]);
            if (setColorCondition % 2 == 0) {
                m_ledBuffer.setRGBArray(i, purpleRGB);

            } else {

                m_ledBuffer.setRGBArray(i, yellowRGB);
            }
            m_led.setData(m_ledBuffer);
        }

        setColorCondition += 1;
    }

    public void staticPurple(){
        for (var i = 0; i < m_ledBuffer.getLength(); i++) {
            m_ledBuffer.setRGBArray(i, purpleRGB);
        }
        m_led.setData(m_ledBuffer);
    }

    public void staticYellow(){
        for (var i = 0; i < m_ledBuffer.getLength(); i++) {
            m_ledBuffer.setRGBArray(i, yellowRGB);
        }
        m_led.setData(m_ledBuffer);
    }

    public void turnOff() {
        for (var i = 0; i < m_ledBuffer.getLength(); i++) {
            m_ledBuffer.setRGBArray(i, offRGB);
        }
        m_led.setData(m_ledBuffer);
    }
    
    public AddressableLED getLedController() {
        return m_led;
    }
}
