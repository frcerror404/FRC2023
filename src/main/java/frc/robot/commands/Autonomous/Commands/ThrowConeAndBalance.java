// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autonomous.Commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.setClawSpeed;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Drivebase;
import frc.robot.subsystems.Gyro;
import frc.robot.subsystems.Wrist;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ThrowConeAndBalance extends ParallelCommandGroup {
  /** Creates a new ThrowConeAndBalance. */
  public ThrowConeAndBalance(Wrist wrist, Claw claw, Drivebase drivebase, Gyro gyro) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        new SequentialCommandGroup(
            new SetWristSpeedForTime(wrist, .5, 0.35),
            new setClawSpeed(claw, -1.0),
            new DelayCommand(.3),
            new setClawSpeed(claw, 0.0),
            new SetWristSpeedForTime(wrist, -.75, 3)),
        new BackwardsChargingStation(drivebase, gyro));
  }
}
