package frc.robot.commands.Autonomous.Commands;

import java.io.IOException;
import java.nio.file.Path;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;

public class followPath {
    private String trajectoryJSON = "output/PickFirstCone.wpilib.json";
    Trajectory trajector = new Trajectory();

    public followPath() {
        try {
            Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON);
            trajector = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
          } catch (IOException ex) {
            DriverStation.reportError("Unable to open Trajector Path" + trajectoryJSON, ex.getStackTrace());
          }
    }
}
