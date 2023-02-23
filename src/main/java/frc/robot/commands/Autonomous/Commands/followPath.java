package frc.robot.commands.Autonomous.Commands;

import java.io.IOException;
import java.nio.file.Path;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.Constants;
import frc.robot.subsystems.Drivebase;

public class followPath extends CommandBase {
  public String trajectoryJSON = "output/ChargingStation.wpilib.json";
  Trajectory trajector = new Trajectory();
  private Drivebase m_drivebase;
  private RamseteCommand m_RamseteCommand;

  public followPath(Drivebase drivebase) {
    m_drivebase = drivebase;
    try {
      Path trajectorPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON);
      trajector = TrajectoryUtil.fromPathweaverJson(trajectorPath);
    } catch (IOException ex) {
      DriverStation.reportError("Error", ex.getStackTrace());
    }

    addRequirements(drivebase);
  }

  @Override
  public void initialize() {
    m_drivebase.resetLeadEncoders();
    m_RamseteCommand = new RamseteCommand(
        trajector,
        m_drivebase::getPose,
        new RamseteController(
            Constants.kRamseteB,
            Constants.kRamseteZeta),
        new SimpleMotorFeedforward(
            Constants.ksVolts,
            Constants.kvVoltsSecondsPerMeter,
            Constants.kaVoltSecondsSquarePErMeter),
        Constants.kDriveKinematics,
        m_drivebase::getWheelSpeeds,
        new PIDController(Constants.kPDriveVel, 0, 0),
        new PIDController(Constants.kPDriveVel, 0, 0),
        m_drivebase::tankDriveVolts,
        m_drivebase);

    m_drivebase.resetOdometry(trajector.getInitialPose());
  }

  @Override
  public void execute() {
    m_RamseteCommand.execute();
  }

  @Override
  public void end(boolean interrupted) {

  }

  @Override
  public boolean isFinished() {
    return false;
  }

}
