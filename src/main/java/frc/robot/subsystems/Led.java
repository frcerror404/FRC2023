package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;



public class Led extends SubsystemBase {
    private final AddressableLED m_led = new AddressableLED(9);
    private int m_rainbowFirstPixelHue = 0;
    public int[] yellowRGB = { 255, 255, 0 };
    public int[] purpleRGB = { 255, 0, 255 };
    public int[] offRGB = { 0, 0, 0 };
    public int setColorCondition = Constants.setColorCondition;
    public String color;
    private final AddressableLEDBuffer m_ledBuffer = new AddressableLEDBuffer(7);

    private double tempTime;
    private boolean decorationColor = false;
    private int timingStep = 0;

    
    public enum WantedColorState {
        OFF,
        DECORATION,
        YELLOW,
        PURPLE
    }

    private WantedColorState m_ColorState = WantedColorState.DECORATION;

    public Led() {
        synchronized (this.m_led) {
            m_led.setLength(m_ledBuffer.getLength());
            m_led.setData(m_ledBuffer);
            m_led.start();
            this.tempTime = Timer.getFPGATimestamp();
        }
    }

    
    public void LedPeriodic() {
        switch(m_ColorState) {
            case OFF: {
                staticColor(Color.kBlack);
                break;
            }
            case PURPLE: {
                staticColor(Color.kPurple);
                break;
            }
            case YELLOW: {
                staticColor(Color.kYellow);
                break;
            }
            case DECORATION: {
                yellowPurpleLed();
                break;
            }
            default: {
                yellowPurpleLed();
                break;
            }
        }
    }

    public void setLEDColor(WantedColorState state) {
        if(m_ColorState != state) {
            m_ColorState = state;
            timingStep = 0;
            
        }
    }

    public void setLedData(AddressableLEDBuffer buffer) {
        m_led.setData(buffer);
    }

    public void rainbowLed() {
        for (var i = 0; i < m_ledBuffer.getLength(); i++) {
            final var hue = (m_rainbowFirstPixelHue + (i * 180 / m_ledBuffer.getLength())) % 180;
            m_ledBuffer.setHSV(i, hue, 255, 128);
        }

        m_rainbowFirstPixelHue += 3;
    }

    public void yellowPurpleLed() {
        if(Timer.getFPGATimestamp() - tempTime > 2) {
            decorationColor = !decorationColor;
            tempTime = Timer.getFPGATimestamp();
            timingStep = 0;
        }

        if(decorationColor) {
            staticColor(Color.kPurple);
        } else {
            staticColor(Color.kYellow);
        }
    }

    public void staticColor(Color color) {
        if(timingStep < m_ledBuffer.getLength()) {
            m_ledBuffer.setLED(timingStep, color);
        } else {
            this.timingStep = -1;
        }

        timingStep++;

        m_led.setData(m_ledBuffer);
    }

    public AddressableLED getLedController() {
        return m_led;
    }
}
