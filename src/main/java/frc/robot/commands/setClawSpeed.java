/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Claw;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class setClawSpeed extends InstantCommand {
    private Double m_speed;
    private Claw m_claw;

  public setClawSpeed(Claw claw, Double speed) {
    m_speed = speed;
    m_claw = claw;

    addRequirements(claw);
  }      

  @Override
  public void execute() {
    m_claw.setClawSpeed(m_speed);
  }
}
