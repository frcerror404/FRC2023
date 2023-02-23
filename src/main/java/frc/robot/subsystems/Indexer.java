package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Indexer extends SubsystemBase {

    private final WPI_TalonSRX Indexer = new WPI_TalonSRX(Constants.kIndexer);

    public Indexer() {

    }

    public void setIndexerSpeed(double indexerSpeed) {
        Indexer.set(indexerSpeed);
    }
}
