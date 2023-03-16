package frc.robot.subsystems;

import java.io.Console;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDManager extends SubsystemBase {
    private final Led m_led;

    public enum WantedColorState {
        OFF,
        SWITCHING,
        YELLOW,
        PURPLE
    }

    public LEDManager(Led led) {
        this.m_led = led;
    }

    public WantedColorState m_ColorState = WantedColorState.SWITCHING;

    public void setWantedState(WantedColorState colorstate) {
        m_ColorState = colorstate;
    }

    public void ledSwitch() {
        synchronized (this.m_led) {
            System.out.println(m_ColorState);
            switch (m_ColorState) {
                case OFF:
                    m_led.turnOff();
                case SWITCHING:
                    m_led.yellowPurpleLed();
                case YELLOW:
                    m_led.staticYellow();
                case PURPLE:
                    m_led.staticPurple();
            }
        }
    }
}
