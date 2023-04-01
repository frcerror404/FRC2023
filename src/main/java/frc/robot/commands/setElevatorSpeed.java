/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Elevator;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class setElevatorSpeed extends CommandBase {
    private Double m_speed;
    private Elevator m_elevator;

  public setElevatorSpeed(Elevator elevator, double speed) {
    m_speed = speed;
    m_elevator = elevator;

    addRequirements(elevator);
  }      

  @Override
  public void execute() {
    m_elevator.setElevatorSpeed(m_speed);
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  public boolean isFinished() {
    return true; // Runs until interrupted
  }
}
