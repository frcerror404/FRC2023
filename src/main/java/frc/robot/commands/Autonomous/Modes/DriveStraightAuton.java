/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.Autonomous.Modes;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Autonomous.Commands.SetDrivetrainSpeedForTime;
import frc.robot.subsystems.Drivebase;
// import frc.robot.Limelight;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class DriveStraightAuton extends SequentialCommandGroup {

  /**
   * Drive straight for X amount of seconds
   */

  public DriveStraightAuton(Drivebase drivebase) {
    super(new SetDrivetrainSpeedForTime(-.4, -.4, 4, drivebase));

    addRequirements(drivebase);
  }
}
