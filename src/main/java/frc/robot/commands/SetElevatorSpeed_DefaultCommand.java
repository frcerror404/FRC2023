/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Elevator;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class SetElevatorSpeed_DefaultCommand extends CommandBase {
    private DoubleSupplier m_rightTrigger, m_leftTrigger;
    private Elevator m_elevator;

  public SetElevatorSpeed_DefaultCommand(Elevator elevator, DoubleSupplier rightTrigger, DoubleSupplier leftTrigger) {
    m_rightTrigger = rightTrigger;
    m_leftTrigger = leftTrigger;
    m_elevator = elevator;

    addRequirements(elevator);
  }      

  @Override
  public void execute() {

    double right = m_rightTrigger.getAsDouble();
    double left = m_leftTrigger.getAsDouble();
    SmartDashboard.putNumber("Left Trigger", m_leftTrigger.getAsDouble());
    SmartDashboard.putNumber("Right Trigger", m_rightTrigger.getAsDouble());

    double speed = 0;

    if(right > 0.05) {
      speed = right;
    }

    if(left > 0.05) {
      speed = -left;
    }

    if(right > 0.05 && left > 0.05) {
      speed = 0;
    }

    m_elevator.setElevatorSpeed(speed);
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  public boolean isFinished() {
    return true; // Runs until interrupted
  }
}
