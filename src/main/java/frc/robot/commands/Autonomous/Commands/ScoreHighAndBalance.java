// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autonomous.Commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.SetElevatorSpeed_DefaultCommand;
import frc.robot.commands.ToggleElevatorExtension;
import frc.robot.commands.setClawSpeed;
import frc.robot.commands.setElevatorSpeed;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Drivebase;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Gyro;
import frc.robot.subsystems.Wrist;

public class ScoreHighAndBalance extends ParallelCommandGroup {
  /** Creates a new ScoreHighAndBalance. */
  public ScoreHighAndBalance(Wrist wrist, Claw claw, Elevator elevator, Drivebase drivebase, Gyro gyro) {
    // Use addRequirements() here to declare subsystem dependencies.

    addCommands(
      new SequentialCommandGroup(
        new setElevatorSpeed(elevator, 1.0),
        new DelayCommand(1.5),
        //new ToggleElevatorExtension(claw),
        new DelayCommand(.25),
        new setClawSpeed(claw, -.7),
        new DelayCommand(.25),
        new setElevatorSpeed(elevator, -1.0),
        //new ToggleElevatorExtension(claw),
        new setClawSpeed(claw, 0.0),
        new SetWristSpeedForTime(wrist, -1.0, 2),
        new setElevatorSpeed(elevator, 0)

      ),
      new SequentialCommandGroup(
        new DelayCommand(3),
        new FasterBackwardBalance(drivebase, gyro)
      )
    );
  }
}
