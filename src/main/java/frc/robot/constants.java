// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
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
    public static final double WheelDiameterInMeters = 0.0967;//0.09868;//0.12287;//0.0967;
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
    public static final double k_AutoXYP = 7.00;
    public static final double k_AutoXYI = 0.0;//0.00015;
    public static final double k_AutoXYD = 0.0;
    //Auto Rotational PID
    public static final double k_AutoRotP = 8.00;
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
    public static final double k_RFZERO =-.9955535133; //2.141437-3.1415926535;//1.055379;
    public static final double k_RBZERO =1.44807786; //-1.6858448+3.1415926535;//0.777728-3.1415926535;
    public static final double k_LFZERO =.7209709703; //0.716369;//1.787;
    public static final double k_LBZERO =-1.31157364; //-1.29467;//0.199418-3.1415926535;

    public static final double k_gyroOffset = -k_PI;

    public static final double kMaxAngularSpeed = kMaxSpeed/k_DriveBaseRadius;
    public static final double kMaxAngularAcceleration = kMaxAcceleration/k_DriveBaseRadius; // change to 9.89 after testing

    //Averaging Values
    public static final double kXLatencyBias = 100;
    public static final double kYLatencyBias = 100;
    public static final double kThetaLatencyBias = 100;

    public static final double kXDriftBias = 500;
    public static final double kYDriftBias = 500;
    public static final double kThetaDriftBias = 500;

    //correction threshold is in senconds

    public static final double kCorrectionThreshold = 5.0;

    // Field constants
    public static final double kFieldLengthMeters = 16.54; // x
    public static final double kFieldWidthMeters = 8.21; // y
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
    public static final double kVisTurretToleranceRadians = 0.05;
    // Limelight
    public static final double kNoteLimelightForwardOffset = 0.41;
    public static final double kNoteLimelightRightOffset = 0.23;
    public static final double kNoteLimelightYawOffset = 18.0; // deg
    //photonvision
    public static final double kAprilTagAmbiguityThreshold = 0.2;
    public static final String kShooterCamName = "Arducam_OV9281_USB_Camera";
    public static final Transform3d kRobotToShooterCam = new Transform3d(
            new Translation3d(-0.09, -0.004, .4587), // x, y, z meters
            new Rotation3d(0.0, Units.degreesToRadians(-7.0), Units.degreesToRadians(180.0))); // roll, pitch, yaw radians

    // Intake
    public static final double kIntakePower = 0.5;
    // public static final int kIntakeMotorTopID = 20;
    // public static final int kIntakeMotorBottomID = 21;
    public static final int kIntakeMotorID = 21;
    public static final int kIntakeBreakbeamPin = 1;

    // Intermediate
    public static final int kIntermediateMotorPower = 1;

    //Shooter 
    public static final double absAngleOffset = 20.51;//63.4;//47.3;
    public static final double[] kShooter1PIDGains = {0.002, 0.0, 0.0};//{0.000003, 0.0, 0.0}; // bottom
    public static final double[] kShooter2PIDGains = {0.002, 0.0, 0.0};//{0.000003, 0.0, 0.0}; // top // original .0005 p
    public static final double kShooter1StartingVoltage = 6.0;
    public static final double kShooter2StartingVoltage = 6.0;
    public static final double[] kAnglePIDGains = {.85/*.8*/, 0.0, 0.0/*0.00004*/};
    public static final double[] kAutoAnglePIDGains = {.85/*.8*/, 0.0, 0.0/*0.00004*/};
    public static final double kTurretAlignTimeout = 1.0;
    public static final double kShooterGearRatio = 1.0/1.0;               
    public static final double kRPMTolerance = 250.0;
    public static final double kangleTolerance = 0.5;
    public static final double kAngleRatio = 15.0 / 46.0 * 360.0; //1.0 / 100.0 * 15.0 / 46.0 * 360.0;//18.0/48.0*90.0;
    public static final double kShooterAngleMaxVelocity = 1.0;
    public static final double kShooterAngleMaxAcceleration = 1.0;
    public static final double kShooterDefaultRPM = 3000.0;//3000.0;//5100.0; //originally 4500
    public static final double kShooterManualAngleControlSpeedMultiplier = 0.25;
    public static final double kShooterFeederSpeed = 0.4;
    public static final double kShooterFeederAmpSpeed = 1.0;
    public static final double kStartAngle = 45.0;//in degrees
    public static final double kShooterMaxAngle = 45.0;//62.0;
    public static final double kShooterMinAngle = -60.0;//0.0;//-60.0;//21.9;
    public static final double kShooterSpeakerVoltage = 8.0;
    public static final double kShooterSubwooferAngle = 45.0;
    public static final double kShooterPodiumAngle = 23.2;
    public static final double kShooterAmpAngle = -42.0;///-40.0;
    public static final double kShooterFlywheelSpinUpTime = 1.5; // seconds
    public static final double kPos1Or3ShooterAngle = 45.0;
    public static final double kShooterClearAmpPivotAngle = 0.0;
    // Pins/CAN ids
    public static final int kShooterFlywheel1ID = 30;
    public static final int kShooterFlywheel2ID = 31;
    public static final int kShooterPivotID = 32;
    public static final int kShooterFeederID = 33;
    public static final int kShooterBreakbeamPin = 0;
    public static final int kShooterAngleEncoderChannelA = 0;
    public static final int kShooterAngleEncoderChannelB = 1;
    public static final int kShooterLimitSwitchPin = 0;

    //Elevator
    public static final double kElevatorGearing = 1.0;
    public static final double knElevatorGearing = 1.0;
    public static final double[] kElevatorPIDGains = {1.0, 0.0, 0.0};
    public static final double kElevatorMaxSpeed = 1.0;
    public static final double kElevatorMaxAcceleration = 1.0;
    public static final double kElevatorHighDefault = 1.0;
    public static final double kElevatorManualAngleControlSpeedMultiplier = 0.1;

    // Amp Pivot
    public static final int kAmpPivotID = 35;
    public static final double[] kAmpPivotPIDGains = {0.0055, 0.0, 0.0};
    public static final double kAmpPivotTolerance = 3.0;
    public static final double kAmpPivotAmpAngle = -141.7;

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
    
    public static final double kSingleTagMaxDistance = 5.0;
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

    
        /** Minimum target ambiguity. Targets with higher ambiguity will be discarded */
        public static final double APRILTAG_AMBIGUITY_THRESHOLD = 0.2;
        public static final double POSE_AMBIGUITY_SHIFTER = 0.2;
        public static final double POSE_AMBIGUITY_MULTIPLIER = 4;
        public static final double NOISY_DISTANCE_METERS = 2.5;
        public static final double DISTANCE_WEIGHT = 7;
        public static final int TAG_PRESENCE_WEIGHT = 10;

        /**
         * Standard deviations of model states. Increase these numbers to trust your
         * model's state estimates less. This
         * matrix is in the form [x, y, theta]ᵀ, with units in meters and radians, then
         * meters.
         */
        public static final Matrix<N3, N1> VISION_MEASUREMENT_STANDARD_DEVIATIONS = VecBuilder.fill(
            // if these numbers are less than one, multiplying will do bad things
            1, // x
            1, // y
            1 * Math.PI); // theta

        /**
         * Standard deviations of the vision measurements. Increase these numbers to
         * trust global measurements from vision
         * less. This matrix is in the form [x, y, theta]ᵀ, with units in meters and
         * radians.
         */
        public static final Matrix<N3, N1> STATE_STANDARD_DEVIATIONS = VecBuilder.fill(
            // if these numbers are less than one, multiplying will do bad things
            .1, // x
            .1, // y
            .1);

        public static final Matrix<N3, N1> MULTI_TAG_STANDARD_DEVIATIONS = VecBuilder.fill(0.3, 0.3, 999999999.0/*3.14*/);
        public static final Matrix<N3, N1> SINGLE_TAG_STANDARD_DEVIATIONS = VecBuilder.fill(1.5, 1.5, 999999999.0/*6.24*/);

        public static final Matrix<N3, N1> IGNORE = VecBuilder.fill(999999999, 999999999, 999999999);
}
