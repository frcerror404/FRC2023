package frc.robot.subsystems;

import com.ctre.phoenix.sensors.Pigeon2;
import com.ctre.phoenix.sensors.WPI_Pigeon2;
import com.ctre.phoenix.sensors.WPI_PigeonIMU;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Gyro {
    // TalonSRX is on CAN Bus with device ID 0
    public WPI_Pigeon2 gyro = new WPI_Pigeon2(3); // Pigeon uses the talon created above
    // The gain for a simple P loop
    // double kP = 0.05;
    private double yawOffset = gyro.getYaw();
    private double rollOffset = gyro.getRoll();
    private double pitchOffset = gyro.getPitch();
    private double[] quaternion = { 0, 0, 0, 0 };
    private double[] ypr = { 0, 0, 0 };

    private double roll = 0, yaw = 0, pitch = 0, angle = 0, heading = 0, turnRate = 0;
    private Rotation2d rotation = Rotation2d.fromDegrees(0.0);

    public void getGyroValues() {
        // Rotation on Ground: Left (Value Increase) - Right (Value Decrease)
        SmartDashboard.putNumber("Gyro TurnDegree", getYaw());
        // Turn Back and forth: Back (Value Increase) - Forth (Value Decrease)
        SmartDashboard.putNumber("Gyro Roll", getRoll());
        // Turn on side: Left (Value Decrease) - Right (Value Increase)
        SmartDashboard.putNumber("Gyro Pitch", getPitch());
        SmartDashboard.putNumberArray("YPR", ypr);
        SmartDashboard.putNumberArray("Quaternion", quaternion);
    }

    public void updatePigeonInfo() {
        // System.out.println("Beginning of Gyro Update");
        roll = gyro.getRoll() - rollOffset;
        yaw = gyro.getYaw() - yawOffset;
        pitch = gyro.getPitch() - pitchOffset;
        gyro.getYawPitchRoll(ypr);
        rotation = gyro.getRotation2d();
        heading = gyro.getRotation2d().getDegrees();
        turnRate = gyro.getRate();
        // turnRate = -gyro.get;
        gyro.get6dQuaternion(quaternion);
        // System.out.println("End of Gyro Update");
    }

    public double getRoll() {
        return roll;
    }

    public double getYaw() {
        return yaw;
    }

    public double getPitch() {
        return pitch;
    }

    public double getAngle() {
        return angle;
    }

    public Rotation2d get2dRotation() {
        return rotation;
    }

    public double getHeading() {
        return heading;
    }

    public double[] getQuaternion() {
        return quaternion;
    }

    // public double getTurnRate() {
    // return turnRate;
    // }

    // public void zeroHeading() {
    // gyro.();
    // }
}
