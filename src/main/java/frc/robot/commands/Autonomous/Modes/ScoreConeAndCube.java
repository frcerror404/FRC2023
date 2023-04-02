// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autonomous.Modes;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.SetWristSpeed;
import frc.robot.commands.ToggleElevatorExtension;
import frc.robot.commands.setClawSpeed;
import frc.robot.commands.setElevatorSpeed;
import frc.robot.commands.Autonomous.Commands.DelayCommand;
import frc.robot.commands.Autonomous.Commands.DriveStraightXMeters;
import frc.robot.commands.Autonomous.Commands.GyroBalance;
import frc.robot.commands.Autonomous.Commands.QuickTurnXDegrees;
import frc.robot.commands.Autonomous.Commands.SetElevatorHeight;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Drivebase;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Gyro;
import frc.robot.subsystems.Wrist;

public class ScoreConeAndCube extends SequentialCommandGroup {
  public ScoreConeAndCube(Drivebase drivebase, Elevator elevator, Claw claw, Gyro gyro, Wrist wrist) {
    // 1. Put elevator all the way up
    // 2. shoot out
    // 3. lower wrist
    // 4. eject cone and raise wrist
    // 5. retract, put elevator down, keep raising wrist
    // 6. drive backwards until past the charge station
    // 7. quick 180
    // 8. drive forward quickly 
    // 9. drive slow lower wrist, suck in cube
    // 10. quick 180
    // 11. gyro balance and shoot cube
    addCommands(
      // step 1
      new SetElevatorHeight(elevator, Constants.ele_UpperLimit, 1.55, false),

      // step 2
      new ParallelCommandGroup(
        new ToggleElevatorExtension(claw),
        new SetWristSpeed(wrist, .35),
        new SetElevatorHeight(elevator, Constants.ele_UpperLimit, .5, true) // holds position for .5 seconds
      ),

      new ToggleElevatorExtension(claw),
      new setClawSpeed(claw, 1.0),
      new DelayCommand(.45),

      // step 3 and 4 and some of 5
      new ParallelCommandGroup(
        new SetWristSpeed(wrist, -1.0),
        new DelayCommand(.5)
      ),

      new ParallelCommandGroup(
        new setClawSpeed(claw, 0.0),
        new SetElevatorHeight(elevator, Constants.ele_LowerLimit / 2, 1, false)
      ),
      
      new ParallelCommandGroup(
        new SetElevatorHeight(elevator, Constants.ele_LowerLimit, 1, false),
        new DriveStraightXMeters(drivebase, -8.0, .7, 3)
      ),

      new DriveStraightXMeters(drivebase, -9.0, .5, 2),
      new DelayCommand(.15),
      new QuickTurnXDegrees(drivebase, gyro, 180, true, .75),
      new DriveStraightXMeters(drivebase, 2, .7, 1),
      
      new ParallelCommandGroup(
        new DriveStraightXMeters(drivebase, 3, .5, 3),
        new SetWristSpeed(wrist, .4),
        new setClawSpeed(claw, .5)
      ),

      new setClawSpeed(claw, .15),
      new SetWristSpeed(wrist, -1),
      new QuickTurnXDegrees(drivebase, gyro, 180, true, .75),

      new ParallelCommandGroup(
        new GyroBalance(drivebase, gyro),
        new SequentialCommandGroup(
          new SetElevatorHeight(elevator, -200000, 2, true),
          new ToggleElevatorExtension(claw),
          new ParallelCommandGroup(
            new setClawSpeed(claw, -1.0),
            new SetElevatorHeight(elevator, -200000, .6, true)
          ),
          
          new ToggleElevatorExtension(claw),
          new setClawSpeed(claw, 0.0)
        )
      )

    );
  }
}
