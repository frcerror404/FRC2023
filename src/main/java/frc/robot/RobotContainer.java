// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Drivebase;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Gyro;
import frc.robot.subsystems.Led;
import frc.robot.commands.SetDrivetrainSpeedCommand;
import frc.robot.commands.setClawSpeed;
import frc.robot.commands.SetElevatorSpeed_DefaultCommand;
import frc.robot.commands.SetWristSpeed;
import frc.robot.commands.ToggleElevatorExtension;
import frc.robot.commands.SetLEDColor;
import frc.robot.commands.SetWristPosition;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Piston;
import frc.robot.subsystems.Wrist;
import frc.robot.subsystems.Led.WantedColorState;
import frc.robot.commands.Autonomous.Commands.DriveStraightOnly;
import frc.robot.commands.Autonomous.Commands.ScoreHighAndBalance;
import frc.robot.commands.Autonomous.Commands.FasterBackwardBalance;
import frc.robot.commands.Autonomous.Commands.GyroBalance;
import frc.robot.commands.Autonomous.Commands.QuickTurnXDegrees;
import frc.robot.commands.Autonomous.Commands.ChargingStation;
import frc.robot.commands.Autonomous.Commands.ThrowConeAndBalance;
import frc.robot.commands.Autonomous.Modes.ScoreConeAndCube;

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
  public final Led m_led = new Led();

  private final XboxController joy0 = new XboxController(0);
  private final XboxController joy1 = new XboxController(1);
  

  // @SuppressWarnings("unused")
  // private WPI_TalonFX RightLead;
  // @SuppressWarnings("unused")
  // private WPI_TalonFX LeftLead;

  private SendableChooser<Command> m_chooser = new SendableChooser<Command>();

  public RobotContainer(Drivebase m_drivebase) {
    drivebase = m_drivebase;
    // RightLead = drivebase.getRightLead();
    // LeftLead = drivebase.getLeftLead();
    // Configure the button bindings
    configureButtonBindings();

    m_chooser.addOption("Regular Balance Only", new ChargingStation(drivebase, gyro));
    m_chooser.addOption("Throw Cone and Backwards Balance", new ThrowConeAndBalance(wrist, claw, drivebase, gyro));
    m_chooser.addOption("Drive Forward Only", new DriveStraightOnly(drivebase, gyro));
    m_chooser.addOption("Backwards Balance Only", new FasterBackwardBalance(drivebase, gyro));
    m_chooser.addOption("Score High And Balance", new ScoreHighAndBalance(wrist, claw, elevator, m_drivebase, gyro));
    m_chooser.addOption("Faster Balance", new FasterBackwardBalance(m_drivebase, gyro));
    m_chooser.addOption("Gyro Balance", new GyroBalance(m_drivebase, gyro));
    m_chooser.setDefaultOption("Score cone and cube", new ScoreConeAndCube(m_drivebase, elevator, claw, gyro, wrist));

    AddAutonomousSelectorToDashboard();
  }

  public void AddAutonomousSelectorToDashboard() {
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
    //return drivebase.getPathCommand("Forward3").andThen(new InstantCommand(() -> drivebase.SetBrakeMode(true)));
  }

  public void runLedManager() {
    this.m_led.LedPeriodic();
  }

  public void runLed() {
    this.m_led.yellowPurpleLed();
  }

  public void setDTBrakes(boolean on) {
    drivebase.SetBrakeMode(on);
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

    P0_AButton.onTrue(new QuickTurnXDegrees(drivebase, gyro, 180, true, 2.0));

    // Manual Claw
    P1_rightBumper.whileTrue(new setClawSpeed(claw, 0.5)).whileFalse(new setClawSpeed(claw, 0.15));
    P1_leftBumper.whileTrue(new setClawSpeed(claw, -0.7)).whileFalse(new setClawSpeed(claw, 0.0));

    elevator.setDefaultCommand(
        new SetElevatorSpeed_DefaultCommand(
            elevator,
            () -> joy1.getRightTriggerAxis(),
            () -> joy1.getLeftTriggerAxis()));

    P1_startButton.whileTrue(new ToggleElevatorExtension(claw));

    P1_BButton.whileTrue(new SetWristSpeed(wrist, .25)).whileFalse(new SetWristSpeed(wrist, -1.0));
    P1_XButton.whileTrue(new SetWristSpeed(wrist, -1.0)).whileFalse(new SetWristSpeed(wrist, 0));

    P1_YButton.whileTrue(new SetWristPosition(wrist)).whileFalse(new SetWristSpeed(wrist, 0));

    P1_AButton.onTrue(new SetLEDColor(WantedColorState.PURPLE, m_led));
    P1_YButton.onTrue(new SetLEDColor(WantedColorState.YELLOW, m_led));

    
    if (Constants.isCurvatureDrive) {
      drivebase.setDefaultCommand(
      new SetDrivetrainSpeedCommand(
      () -> joy0.getRightY(),
      () -> joy0.getLeftY(),
      () -> joy0.getLeftBumper(),
      () -> joy0.getRightBumper(),
      drivebase));

      } else {
      drivebase.setDefaultCommand(
      new SetDrivetrainSpeedCommand(
      () -> joy0.getRightY(),
      () -> joy0.getLeftY(),
      () -> joy0.getLeftBumper(),
      () -> joy0.getRightBumper(),
      drivebase));
      }
    }
  }
