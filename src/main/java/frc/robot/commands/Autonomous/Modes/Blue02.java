package frc.robot.commands.Autonomous.Modes;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Autonomous.Commands.followPath;
import frc.robot.subsystems.Drivebase;

public class Blue02 extends SequentialCommandGroup {
    public Blue02(Drivebase m_drivebase) {
        super(new followPath(m_drivebase));
    }
}
