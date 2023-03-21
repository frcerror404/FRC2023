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
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants;
import frc.robot.subsystems.Led;
import frc.robot.subsystems.Led.WantedColorState;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class SetLEDColor extends InstantCommand {
  private WantedColorState m_state;
  private Led m_Led;

  public SetLEDColor(WantedColorState state, Led led) {
    m_state = state;
    m_Led = led;

    addRequirements(led);
  }      

  @Override
  public void execute() {
      m_Led.setLEDColor(m_state);
  }
}