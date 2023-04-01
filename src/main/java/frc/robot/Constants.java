// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;

public final class Constants {
    // Drivetrain CAN ID's
    public static int klDT1 = 3;
    public static int klDT2 = 4;
    public static int krDT1 = 1;
    public static int krDT2 = 2;
    //DriveTrain Speed
    public static double driveFastRPM = 1.0;
    public static double driveDefault = .7;
    public static double driveSlowRPM = .4;
    public static double DTleftAxis = 0;
    public static double DTrightAxis = 0;
    //Converter
    public static double EncoderUnits = 2048;
    public static double GearboxUnits = 10.71;
    public static double feetToMeter = 0.479;
    //Paths
    public static final double ksVolts = 0.66186;
    public static final double kvVoltsSecondsPerMeter = 3.5535;
    public static final double kaVoltSecondsSquarePErMeter = 2.9051;
    public static final double kPDriveVel = 5.7234;
    public static final double kMaxSpeedMetersPerSecond = 6;
    public static final double kMaxAccelerationMetersPerSecondSquare = 2;
    public static final double kRamseteB = 2;
    public static final double kRamseteZeta = 0.7;
    public static final DifferentialDriveKinematics kDriveKinematics = new DifferentialDriveKinematics(0.479);
    public static final int kExtendedSolenoid = 0;
    public static final int kRetractedSolenoid = 1;
    public static final double WristPlayerStation = 15000;
   

    //Limelight
    public static double limelightTx;
    public static int limelightPipeline;

    //Led
    public static int setColorCondition = 0;

    // Claw
    public static int clawR = 9;
    public static int clawL = 10;

    public static int kCompressor = 11;

    public static int SparkMotorRP = 61;
    public static int SparkMotorLP = 62;
    public static double gearRatio = 0;
    public static boolean motorInverted = true;
    public static double ClawIntakeRPM = -1;
    public static double ClawOuttakeRPM = 1;
    //Wrist
    public static int wrist = 8;

    public static double WristUPRPM = -1;
    public static double WristDOWNRPM = 1;
    //Elevator
    public static int elevatorR = 6;
    public static int elevatorL = 5;
    public static double currentElevatorLimit = 20;
    public static double currentElevatorThreshold = 25;
    public static double currentThresholdTime = 1.0;

    public static double UPelevatorRPM = 1;
    public static double DOWNelevatorRPM = -1;
    //Gyro
    public static int gyro = 7;

    public static int kIndexer = 0;

    public static double Falcon500SensorUnitsConstant = 2048;
    
    public static double kDrivetrainSpeedMultiplier = 0.75;
    public static double kDrivetrainLowSpeedMultiplier = 0.45;

    // true = arcade drive, false = tank drive
    public static boolean isCurvatureDrive = false;



    /**
     * Elevator Positions
     */

    public static double ele_FloorPosition = 0;
    public static double ele_TopPosition = 300000;
    public static double ele_MidPosition = 200000;
    public static double ele_LowerLimit = -2500;
    public static double ele_UpperLimit = -430000;


    /**
     * Wrist Soft Stops
     */
    public static int wrist_In = 0;
    public static int wrist_Out = 24000;
}
