package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Piston extends SubsystemBase {
    Solenoid piston = new Solenoid(PneumaticsModuleType.REVPH, 0);

    public Piston() {

    }

    public void togglePiston(boolean on) {
        piston.set(on);
    }
}




