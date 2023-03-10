/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Drivebase;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class SetDrivetrainSpeedCommand extends CommandBase {
  private final Drivebase m_Drivebase;
  private final DoubleSupplier m_leftAxis;
  private final DoubleSupplier m_rightAxis;
  private final BooleanSupplier m_turbo;
  private final BooleanSupplier m_slowMode;

  public SetDrivetrainSpeedCommand(DoubleSupplier leftAxis, DoubleSupplier rightAxis, BooleanSupplier turbo, BooleanSupplier slowMode, Drivebase drivebase) {
    m_Drivebase = drivebase;
    m_leftAxis = leftAxis; 
    m_rightAxis = rightAxis;
    m_turbo = turbo;
    m_slowMode = slowMode;
    addRequirements(drivebase);
  }


@Override
  public void execute() {
    //Default speed multiplier = 70% = .7
    double speedMultiplier = Constants.kDrivetrainSpeedMultiplier;
    double LowSpeedMultiplier = Constants.kDrivetrainLowSpeedMultiplier;
    double leftAxis = Constants.DTleftAxis;
    double rightAxis = Constants.DTrightAxis;

    if(m_turbo.getAsBoolean()) {
      leftAxis = m_leftAxis.getAsDouble();
      rightAxis = m_rightAxis.getAsDouble();
    } else if (m_slowMode.getAsBoolean()) {
      leftAxis = m_leftAxis.getAsDouble() * LowSpeedMultiplier;
      rightAxis = m_rightAxis.getAsDouble() * LowSpeedMultiplier;
    } else {
      leftAxis = m_leftAxis.getAsDouble() * speedMultiplier;
      rightAxis = m_rightAxis.getAsDouble() * speedMultiplier;
    }
    
      
    


    m_Drivebase.manualControl(leftAxis, rightAxis, m_turbo.getAsBoolean());
  }

   // Make this return true when this Command no longer needs to run execute()
   @Override
   public boolean isFinished() {
     return false; // Runs until interrupted
   }
}
