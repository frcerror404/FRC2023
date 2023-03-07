// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autonomous.Commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Wrist;



public class ShootConeOut extends CommandBase {
  private final Wrist m_wrist;
  private final Claw m_claw;


  /* Creates a new ShootConeOut. */
  public ShootConeOut(Claw claw, Wrist wrist) {
    m_wrist = wrist;
    m_claw = claw;
    addRequirements(claw, wrist);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_wrist.setWristSpeed(-.4);
    m_claw.setClawSpeed(1.0);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
