package frc.robot.commands.Autonomous.Modes;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Autonomous.Commands.ChargingStation;
import frc.robot.subsystems.Drivebase;
import frc.robot.subsystems.Gyro;

public class Blue01 extends SequentialCommandGroup {
    public Blue01(Drivebase drivebase, Gyro gyro, WPI_TalonFX RightLead, WPI_TalonFX LeftLead) {
        super(new ChargingStation(drivebase, gyro, RightLead, LeftLead));

        addRequirements(drivebase);
    }
}
