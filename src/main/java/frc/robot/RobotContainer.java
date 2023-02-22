// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import javax.print.attribute.standard.SheetCollate;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Drivebase;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Gyro;
import frc.robot.commands.SetDrivetrainSpeedCommand;
import frc.robot.commands.Autonomous.Commands.ChargingStation1;
import frc.robot.commands.Autonomous.Commands.TurnInPlaceXDegrees;
import frc.robot.commands.Autonomous.Modes.Blue01;
import frc.robot.commands.Autonomous.Modes.DriveStraightAuton;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Piston;
import frc.robot.subsystems.Wrist;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  public final Drivebase drivebase;
  // public final FrontClimber Fclimber = new FrontClimber();
  // public final BackClimber Bclimber = new BackClimber();
  public final Limelight limelight = new Limelight();
  public final Gyro gyro = new Gyro();
  public final Elevator elevator = new Elevator();
  public final Wrist wrist = new Wrist();
  public final Claw claw = new Claw(Constants.SparkMotorRP, Constants.SparkMotorLP, Constants.gearRatio, Constants.motorInverted);
  public final Piston m_piston = new Piston();

  private final XboxController joy0 = new XboxController(0);
  private final XboxController joy1 = new XboxController(1);

  @SuppressWarnings("unused")
  private WPI_TalonFX RightLead;
  @SuppressWarnings("unused")
  private WPI_TalonFX LeftLead;

  private SendableChooser<Command> m_chooser = new SendableChooser<Command>();

  public RobotContainer(Drivebase drivebase) {
    this.drivebase = drivebase;
    RightLead = drivebase.getRightLead();
    LeftLead = drivebase.getLeftLead();
    // Configure the button bindings
    configureButtonBindings();

    // m_chooser.addOption("Rotate 90", new FourBallAuton(drivebase, imu, 90, .6));
    // m_chooser.addOption("Rotate -30", new FourBallAuton(drivebase, imu, -30,
    // .6));
    // m_chooser.addOption("Rotate 180", new FourBallAuton(drivebase, imu, 180,
    // .6));
    m_chooser.addOption("ChargingStation 1", new ChargingStation1(drivebase, gyro, RightLead, LeftLead));
    m_chooser.addOption("ChadIsCool", new Blue01(drivebase, gyro, RightLead, LeftLead));

    SmartDashboard.putData(m_chooser);
  }

  public void getLimelightValues() {
    limelight.GetLimelightValues();
  }

  public void getGyroValues() {
    gyro.getGyroValues();
  }

  public void getElevatorSensors() {
    elevator.getSensorPositios();
  }

  public void putDrivebaseSensors() {
    drivebase.smartDashboardDrivetrainEncoders();
  }

  public Command getAutonomousCommand() {
    return m_chooser.getSelected();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing
   * it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  @SuppressWarnings("unused")
  private void configureButtonBindings() {

    // Driver button mapping
    JoystickButton P0_RBumper = new JoystickButton(joy0, XboxController.Button.kRightBumper.value);
    JoystickButton P0_LBumper = new JoystickButton(joy0, XboxController.Button.kLeftBumper.value);
    JoystickButton P0_BButton = new JoystickButton(joy0, XboxController.Button.kB.value);
    JoystickButton P0_AButton = new JoystickButton(joy0, XboxController.Button.kA.value);
    JoystickButton P0_XButton = new JoystickButton(joy0, XboxController.Button.kX.value);
    JoystickButton P0_YButton = new JoystickButton(joy0, XboxController.Button.kY.value);
    JoystickButton P0_LStick = new JoystickButton(joy0, XboxController.Button.kLeftStick.value);
    JoystickButton P0_RStick = new JoystickButton(joy0, XboxController.Button.kRightStick.value);

    // Operator botton mapping
    JoystickButton P1_leftBumper = new JoystickButton(joy1, XboxController.Button.kLeftBumper.value);
    JoystickButton P1_rightBumper = new JoystickButton(joy1, XboxController.Button.kRightBumper.value);
    JoystickButton P1_BButton = new JoystickButton(joy1, XboxController.Button.kB.value);
    JoystickButton P1_AButton = new JoystickButton(joy1, XboxController.Button.kA.value);
    JoystickButton P1_XButton = new JoystickButton(joy1, XboxController.Button.kX.value);
    JoystickButton P1_YButton = new JoystickButton(joy1, XboxController.Button.kY.value);
    JoystickButton P1_RStick = new JoystickButton(joy1, XboxController.Button.kRightStick.value);
    JoystickButton P1_LStick = new JoystickButton(joy1, XboxController.Button.kLeftStick.value);
    JoystickButton P1_leftTrigger = new JoystickButton(joy1, XboxController.Axis.kLeftTrigger.value);
    JoystickButton P1_rightTrigger = new JoystickButton(joy1, XboxController.Axis.kRightTrigger.value);
    JoystickButton P1_startButton = new JoystickButton(joy1, XboxController.Button.kStart.value);
    // P0_LStick
    // .whenPressed(new SetRelease(martianClimbers, ReleaseType.ShortArmRelease))
    // .whenReleased(new SetRelease(martianClimbers, ReleaseType.None));
    
    // Driver commands

    //Drive, Turbo, Slow
    if (Constants.isCurvatureDrive) {
      drivebase.setDefaultCommand(
          new SetDrivetrainSpeedCommand(
              () -> joy0.getLeftY(),
              () -> joy0.getRightY(),
              () -> joy0.getLeftBumper(),
              () -> joy0.getRightBumper(),
              drivebase));
    } else {
      drivebase.setDefaultCommand(
          new SetDrivetrainSpeedCommand(
              () -> joy0.getLeftY(),
              () -> joy0.getRightY(),
              () -> joy0.getLeftBumper(),
              () -> joy0.getRightBumper(),
              drivebase));
    }

    //Operator commands

    // Manuel Claw
    P1_rightBumper.whileTrue(claw.runClaw(Constants.ClawOuttakeRPM));
    P1_leftBumper.whileTrue(claw.runClaw(Constants.ClawIntakeRPM));
    P1_rightBumper.and(P1_leftBumper).onFalse(claw.runClaw(0));

    // Manuel Elevator
    P1_rightTrigger.getAsBoolean();
    P1_leftTrigger.getAsBoolean();
    P1_rightTrigger.whileTrue(elevator.runElevator(Constants.UPelevatorRPM));
    P1_rightTrigger.whileTrue(elevator.runElevator(Constants.DOWNelevatorRPM));
    P1_leftTrigger.and(P1_rightTrigger).onFalse(claw.runClaw(0));

    // Manuel Piston
    if (P1_startButton.getAsBoolean()) {
      m_piston.togglePiston(true);
    } else {
      m_piston.togglePiston(false);
    }

    // Manuel Wrist
    if (joy1.getLeftY() > 0) {
      wrist.moveWrist(Constants.WristUPRPM);
    } else if (joy1.getLeftY() < 0) {
      wrist.moveWrist(Constants.WristDOWNRPM);
    } else {
      wrist.moveWrist(0);
    }

  }
}