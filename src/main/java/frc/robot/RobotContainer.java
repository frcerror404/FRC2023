// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Drivebase;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Gyro;
import frc.robot.commands.SetDrivetrainSpeedCommand;
import frc.robot.commands.runClaw;
import frc.robot.commands.setClawSpeed;
import frc.robot.commands.SetElevatorSpeed_DefaultCommand;
import frc.robot.commands.SetWristSpeed;
import frc.robot.commands.ToggleElevatorExtension;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Piston;
import frc.robot.subsystems.Wrist;
import frc.robot.subsystems.old_Drivetrain;
import frc.robot.commands.setElevatorPosition;

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
  public final Claw claw = new Claw(Constants.SparkMotorRP, Constants.SparkMotorLP, Constants.gearRatio,
      Constants.motorInverted);
  public final Piston m_piston = new Piston();
  public final Compressor compressor = new Compressor(Constants.kCompressor, PneumaticsModuleType.CTREPCM);

  private final XboxController joy0 = new XboxController(0);
  private final XboxController joy1 = new XboxController(1);
  

  @SuppressWarnings("unused")
  private WPI_TalonFX RightLead;
  @SuppressWarnings("unused")
  private WPI_TalonFX LeftLead;

  private SendableChooser<Command> m_chooser = new SendableChooser<Command>();

  public RobotContainer(Drivebase m_drivebase) {
    drivebase = m_drivebase;
    RightLead = drivebase.getRightLead();
    LeftLead = drivebase.getLeftLead();
    // Configure the button bindings
    configureButtonBindings();

    

    // m_chooser.addOption("Elevator 0", new setElevatorPosition(0.0, elevator));
    // m_chooser.addOption("Elevator Middle", new setElevatorPosition(200000.0,
    // elevator));
    // m_chooser.addOption("Elevator Top", new setElevatorPosition(490000.0,
    // elevator));

    SmartDashboard.putData(m_chooser);
  }

  public void getLimelightValues() {
    limelight.GetLimelightValues();
  }

  public void getGyroValues() {
    gyro.getGyroValues();
  }

  public void updatePigeon() {
    gyro.updatePigeonInfo();
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
    JoystickButton P0_RBumper = new JoystickButton(joy0,
    XboxController.Button.kRightBumper.value);
    JoystickButton P0_LBumper = new JoystickButton(joy0,
    XboxController.Button.kLeftBumper.value);
    JoystickButton P0_BButton = new JoystickButton(joy0,
    XboxController.Button.kB.value);
    JoystickButton P0_AButton = new JoystickButton(joy0,
    XboxController.Button.kA.value);
    JoystickButton P0_XButton = new JoystickButton(joy0,
    XboxController.Button.kX.value);
    JoystickButton P0_YButton = new JoystickButton(joy0,
    XboxController.Button.kY.value);
    JoystickButton P0_LStick = new JoystickButton(joy0,
    XboxController.Button.kLeftStick.value);
    JoystickButton P0_RStick = new JoystickButton(joy0,
    XboxController.Button.kRightStick.value);

    // Operator botton mapping
    JoystickButton P1_leftBumper = new JoystickButton(joy1,
    XboxController.Button.kLeftBumper.value);
    JoystickButton P1_rightBumper = new JoystickButton(joy1,
    XboxController.Button.kRightBumper.value);
    JoystickButton P1_BButton = new JoystickButton(joy1,
    XboxController.Button.kB.value);
    JoystickButton P1_AButton = new JoystickButton(joy1,
    XboxController.Button.kA.value);
    JoystickButton P1_XButton = new JoystickButton(joy1,
    XboxController.Button.kX.value);
    JoystickButton P1_YButton = new JoystickButton(joy1,
    XboxController.Button.kY.value);
    JoystickButton P1_startButton = new JoystickButton(joy1,
    XboxController.Button.kStart.value);

    
    // P0_LStick
    // .whenPressed(new SetRelease(martianClimbers, ReleaseType.ShortArmRelease))
    // .whenReleased(new SetRelease(martianClimbers, ReleaseType.None));

    // Manuel Claw
    P1_rightBumper.whileTrue(new setClawSpeed(claw, 0.5)).whileFalse(new setClawSpeed(claw, 0.0));
    P1_leftBumper.whileTrue(new setClawSpeed(claw, -0.15)).whileFalse(new setClawSpeed(claw, 0.0));

    // Manuel Elevator
    // P1_AButton
    // .whileTrue(new setElevatorPosition(elevator, Constants.ele_MidPosition))
    // .whileFalse(new setElevatorPosition(elevator, Constants.ele_FloorPosition));

    elevator.setDefaultCommand(
        new SetElevatorSpeed_DefaultCommand(
            elevator,
            () -> joy1.getRightTriggerAxis(),
            () -> joy1.getLeftTriggerAxis()));

    P1_startButton.whileTrue(new ToggleElevatorExtension(claw));

    P1_BButton.whileTrue(new SetWristSpeed(wrist, .25)).whileFalse(new SetWristSpeed(wrist, 0));
    P1_XButton.whileTrue(new SetWristSpeed(wrist, -.35)).whileFalse(new SetWristSpeed(wrist, 0));

    if (Constants.isCurvatureDrive) {
      drivebase.setDefaultCommand(
      new SetDrivetrainSpeedCommand(
      () -> joy0.getRightY(),
      () -> joy0.getLeftY(),
      () -> joy0.getLeftBumper(),
      () -> joy0.getRightBumper(),
      drivebase));
      System.out.println(joy0.getRightY());

      } else {
      drivebase.setDefaultCommand(
      new SetDrivetrainSpeedCommand(
      () -> joy0.getRightY(),
      () -> joy0.getLeftY(),
      () -> joy0.getLeftBumper(),
      () -> joy0.getRightBumper(),
      drivebase));
      }


      System.out.print("Container Speed: ");
      System.out.println(joy0.getRightY());
    }
  }
