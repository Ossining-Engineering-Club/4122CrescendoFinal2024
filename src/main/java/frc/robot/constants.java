// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.trajectory.TrapezoidProfile;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class constants {


    public static final double k_PI = 3.141592653589793238462643383279502884197169399;

    public static boolean k_isRed;
    public static final double kRedSpeakerX = 16.579342;
    public static final double kRedSpeakerY = 5.547868;
    public static final double kBlueSpeakerX = -0.0381;
    public static final double kBlueSpeakerY = 5.547868;
    //SwerveModule constants
    public static final double DrivingMotorGearRatio = (1.0/6.75);
    public static final double WheelDiameterInMeters = 0.0967;
    public static final double TurningMotorGearRatio = (7.0/150.0);
    public static final double k_DriveEncoderPosFactor = (DrivingMotorGearRatio * k_PI * WheelDiameterInMeters);
    public static final double k_DriveEncoderVelocityFactor = (k_DriveEncoderPosFactor / 60.0);
    public static final double k_TurnEncoderPosFactor = (TurningMotorGearRatio * k_PI * 2.0);
    public static final double k_TurnEncoderVelocityFactor = (k_TurnEncoderPosFactor / 60.0);
    //Auto drivebase radius for holonomic controller
    public static final double k_DriveBaseRadius = 0.356901;
    //PID constants for motor rotation
    public static final double k_KRP = .45;
    public static final double k_KRI = 0.0;
    public static final double k_KRD = 0.00025;
    
    //Auto Translational PID
    public static final double k_AutoXYP = 5.0;
    public static final double k_AutoXYI = 0.0;
    public static final double k_AutoXYD = 0.0;
    //Auto Rotational PID
    public static final double k_AutoRotP = 5.0;
    public static final double k_AutoRotI = 0.0;
    public static final double k_AutoRotD = 0.0;
    //Odometry at setpoint condition meters and radians
    public static final double k_OdometryToleranceX = 0.1;
    public static final double k_OdometryToleranceY = 0.1;
    public static final double k_OdometryToleranceRot = 0.1;
    //breakbeam threshold and code in volts
    public static final double k_BreakbeamVoltageThreshold = 2.5;
    public static final int k_BreakbeamSamplingWindow = 2;

    public static final double kMaxSpeed = 5.0292; // also remember to change in pathplanner
    public static final double kMaxAcceleration = 5.0292; // 4.441
    
    public static final double k_offsetadj = 1.0;

    
    public static final double[] kTranslationalPIDGains = {2, 0, 0.002}; // {kP, kI, kD}
    public static final double[] kRotationalPIDGains = {3.25, 0, 0.008}; // {kP, kI, kD}
    
    public static final double k_VisionXTolerance = .10;
    public static final double k_VisionYTolerance = .10;
    public static final double k_VisionRotTolerance = .05;
    
    public static final double ShelfAlpha = 3.380949;
    public static final double ShelfBeta = -94.741562;
    public static final double ShelfGamma = -193.466797;


    public static final double kControllerDeadband = 0.12;
    // public static final double ShelfAlpha 8.500027
    // public static final double ShelfBeta -98.335808
    // public static final double ShelfGamma -178.277390
    
    public static final double AutoPlaceAlpha =2.0;
    public static final double AutoPlaceBeta =-232.842621;
    public static final double AutoPlaceGamma= -83.979454;
    // public static final double k_RBZERO = (3.204486-1.57079632) - k_PI + 0.1;
    // public static final double k_RFZERO = (1.761010-1.57079632)- k_PI;
    // public static final double k_LBZERO = (5.787710-1.57079632)- k_PI;
    // public static final double k_LFZERO = (5.514661-1.57079632)- k_PI;
    // public static final double k_RFZERO =(1.761010-1.57079632)-3.1415926535 + 0.009;
    // public static final double k_RBZERO =(3.204486-1.57079632)-3.1415926535 + 0.1 +0.023010;
    // public static final double k_LFZERO =(5.514661-1.57079632)-3.1415926535 -0.06;
    // public static final double k_LBZERO =(5.787710-1.57079632)-3.1415926535 + 0.006;
    //
    //Offset values for motor604388
    public static final double k_RFZERO =2.141437-3.1415926535;//1.055379;
    public static final double k_RBZERO =-1.6858448+3.1415926535;//0.777728-3.1415926535;
    public static final double k_LFZERO =0.716369;//1.787;
    public static final double k_LBZERO =-1.29467;//0.199418-3.1415926535;

    public static final double k_gyroOffset = -k_PI;

    public static final double kMaxAngularSpeed = kMaxSpeed/k_DriveBaseRadius;
    public static final double kMaxAngularAcceleration = kMaxAcceleration/k_DriveBaseRadius; // change to 9.89 after testing

    //Auto things
    //anglepid
    public static final double k_AAP = 9.3;
    public static final double k_AAI = 0.0;
    public static final double k_AAD = 0.0;
    //Constraint for Angled PID
        public static final TrapezoidProfile.Constraints k_angleControllerConstriants =
        new TrapezoidProfile.Constraints(
            kMaxAngularSpeed, kMaxAngularAcceleration);
    //xpid
    public static final double k_AXP = 3.14;
    public static final double k_AXI = 0.0;
    public static final double k_AXD = 0.0;
    //ypid
    public static final double k_AYP = 3.14;
    public static final double k_AYI = 0.0;
    public static final double k_AYD = 0.0;
   // Vision
    public static final double kVisionXToleranceMeters = 0.05;
    public static final double kVisionYToleranceMeters = 0.05;
    public static final double kVisionRotToleranceMeters = 0.05;
    public static final double kVisionXToleranceRadians = 0.05;
    // the rotation pid setpoint is "0.5 meters" ahead of what it is "supposed" to be
    // this is to prevent a delay between when the translation pid finishes and the rotation pid finishes
    public static final double kVisionRotConstant = 0.5;
    // These gains are for vision using robot relative drive, april tags can move
    public static final double[] kVisionTransTrackingPIDGains = {2, 0, 0.002}; // {kP, kI, kD}
    public static final double[] kVisionRotTrackingPIDGains = {3.25, 0, 0.008}; // {kP, kI, kD}
    // These gains are for vision using field relative drive, april tags MUST be still
    public static final double[] kVisionTransPIDGains = {0.5, 0.0, 0.0}; // {kP, kI, kD}
    public static final double[] kVisionRotPIDGains = {1.0, 0.0, 0.0}; // {kP, kI, kD}
    public static final double[] kVisTurretPID = {1.0, 0.0, 0.0}; // {kP, kI, kD}
    public static final double kVisTurretToleranceRadians = 0.262;
    // Limelight
    public static final double kNoteLimelightForwardOffset = 0.0;
    public static final double kNoteLimelightRightOffset = 0.0;

    // Intake
    public static final double kIntakePower = 1.0;
    // public static final int kIntakeMotorTopID = 20;
    // public static final int kIntakeMotorBottomID = 21;
    public static final int kIntakeMotorID = 20;
    public static final int kIntakeBreakbeamPin = 1;

    // Intermediate
    public static final int kIntermediateMotorPower = 1;

    //Shooter 
    public static final double[] kShooter1PIDGains = {0.000003, 0.0, 0.0};
    public static final double[] kShooter2PIDGains = {0.000003, 0.0, 0.0};
    public static final double[] kAnglePIDGains = {.8, 0.0, 0.0};
    public static final double kShooterGearRatio = 1.0/1.0;
    public static final double kRPMTolerance = 150.0;
    public static final double kangleTolerance = 1.0;
    public static final double kAngleRatio = 18.0/48.0*90.0;
    public static final double kShooterAngleMaxVelocity = 1.0;
    public static final double kShooterAngleMaxAcceleration = 1.0;
    public static final double kShooterDefaultRPM = 4500.0;//3000.0;//5100.0;
    public static final double kShooterManualAngleControlSpeedMultiplier = 0.75;
    public static final double kShooterFeederSpeed = 0.4;
    public static final double kStartAngle = 58.0;//in degrees
    public static final double kShooterMaxAngle = 62.0;
    public static final double kShooterMinAngle = 21.9;
    public static final double kShooterSpeakerVoltage = 5.0;
    public static final double kShooterSubwooferAngle = 52.8;
    public static final double kShooterPodiumAngle = 52.8;
    public static final double kShooterAmpAngle = 52.8;
    // Pins/CAN ids
    public static final int kShooterFlywheel1ID = 30;
    public static final int kShooterFlywheel2ID = 31;
    public static final int kShooterPivotID = 32;
    public static final int kShooterFeederID = 33;
    public static final int kShooterBreakbeamPin = 0;
    public static final int kShooterAngleEncoderChannelA = 0;
    public static final int kShooterAngleEncoderChannelB = 1;

    //Elevator
    public static final double kElevatorGearing = 1.0;
    public static final double knElevatorGearing = 1.0;
    public static final double[] kElevatorPIDGains = {1.0, 0.0, 0.0};
    public static final double kElevatorMaxSpeed = 1.0;
    public static final double kElevatorMaxAcceleration = 1.0;
    public static final double kElevatorHighDefault = 1.0;
    public static final double kElevatorManualAngleControlSpeedMultiplier = 0.1;

    // Led
    public static final int kPWMLedPin = 9;

    // Enums
    public static enum State {
        CLEAR,
        INTAKE,
        ELEVATOR,
        SHOOTER,
        SYSTEM
    }
        
    public static enum Direction {
        FORWARD,
        REVERSE,
        STOPPED
    }

    //Climber
    public static final double[] kClimberPIDGains = {1.0, 0.0, 0.0};
    public static final double kClimberMaxSpeed = 1.0; // m/s
    public static final double kClimberMaxAcceleration = 1.0; // m/s^2
    public static final double kClimberGearing = 1.0;
    public static final double kClimberHighDefault = 1.0;

    // Note Searching
    public static final double kNoteSearchingSpeed = 1.0;

    // Secondary Controller
    public static final int kForwardsOrReverseButton = 2;
    public static final int kShooterOrElevatorButton = 0;
    public static final int kAutomaticOrManualButton = 5;
    public static final int kShooterButton = 4;
    public static final int kLightShooterButton = 1;
    public static final int kElevatorButton = 4;
    public static final int kClimberButton = 5;
    public static final int kEjectButton = 9;
    public static final int kIntakeToShooterButton = 3;
    public static final int kShooterElevatorJoystickAxis = 0;
    public static final int kClimberJoystickAxis = 2;

    // Pose estimator
    public static final double[] kPoseEstimatorStateStdDevs = {0.1, 0.1, 0.1}; // {x meters, y meters, theta radians}
    public static final double[] kPoseEstimatorVisionStdDevs = {5.0, 5.0, 5.0}; // {x meters, y meters, theta radians}

    // Go To Note
    public static final double kGoToNoteTimeout = 2.0; // seconds
    public static final double kGoToNoteSpeed = 1.00;

    // Auto Switch Box
    public static final int kAutoSwitch0Pin = 3;
    public static final int kAutoSwitch1Pin = 4;
    public static final int kAutoSwitch2Pin = 5;
    public static final int kAutoSwitch3Pin = 6;
}
