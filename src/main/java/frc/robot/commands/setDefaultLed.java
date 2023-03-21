/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Led;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class setDefaultLed extends CommandBase {
    private Led m_led;

  public setDefaultLed(Led led) {
    m_led = led;

    addRequirements(led);
  }      

  @Override
  public void execute() {
    m_led.color = "PurpleYellow";
        

  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  public boolean isFinished() {
    return true; // Runs until interrupted
  }
}
