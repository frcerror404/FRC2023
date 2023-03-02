/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Elevator;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class setElevatorPosition extends CommandBase {
    private Double m_units;
    private Elevator m_elevator;

  public setElevatorPosition(Elevator elevator, double units) {
    m_units = units;
    m_elevator = elevator;

    addRequirements(elevator);
  }      

  @Override
  public void execute() {
    m_elevator.setElevatorPosition(m_units);
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  public boolean isFinished() {
    return true; // Runs until interrupted
  }
}
