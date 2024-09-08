

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import org.littletonrobotics.junction.Logger;
import org.photonvision.EstimatedRobotPose;

import com.ctre.phoenix.sensors.PigeonIMU;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.vision.Vision;
import frc.robot.constants;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.OECNavX;
import frc.robot.subsystems.Photon;
import frc.robot.subsystems.vision.Vision.PoseEstimate;

public class Drivetrain extends SubsystemBase {
  private final Field2d m_field = new Field2d();
  private SwerveMod RFMod = new SwerveMod(3,4,10,false, constants.k_RFZERO,false,true);
  private SwerveMod RBMod = new SwerveMod(7,8,12,false,constants.k_RBZERO,false,true);
  private SwerveMod LBMod = new SwerveMod(5,6,11,false, constants.k_LBZERO,false,true);
  private SwerveMod LFMod = new SwerveMod(1,2,9,false, constants.k_LFZERO,false,true);
  //Robot Dimensions 27,305 by 29,845 (in inches)
  //Positions defined from a top down view
  public Pose2d tempSetpoint;
  private Photon cam =  new Photon(constants.kShooterCamName, constants.kRobotToShooterCam);
  private Translation2d frontLeftLocation = new Translation2d(0.2275, 0.275);
  private Translation2d frontRightLocation = new Translation2d(0.2275, -0.275);
  private Translation2d backLeftLocation = new Translation2d(-0.2275, 0.275);
  private Translation2d backRightLocation = new Translation2d(-0.2275, -0.275);

  public final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(frontLeftLocation, frontRightLocation, backLeftLocation, backRightLocation); 
  public final PigeonIMU gyro;
  SwerveDrivePoseEstimator odometry;
  Trajectory trajectory;
  //String file = "D:/Temp Robotics/JavaSwerveDriveCommand-Imported/src/main/paths/output/Unnamed.wpilib.json";

  private final Limelight m_shooterLimelight;
  private final Vision vision;

  private double v_prevShooterLLTimestamp = -1;
  public static boolean is_red;
  private Pose2d v_prevPose;
  private double v_xSpeed = 0; // m/s
  private double v_ySpeed = 0; // m/s
  private double v_rotSpeed = 0; // rad/s

  public Drivetrain(int gyroport, Vision vision, Limelight shooterLimelight){
      //Motor process:
      /*Decide front back, left and right and assign locations/module names accordingly
        Find the encoder offsets by reading the absolute encoders when all the wheels are lined up in a direction parallel to front
        Log each wheels distance traveled, move forward on the sticks and note any wheels that are moving in a negative distance
        Subtract PI from the absolute encoder offset of the wheels reported in a negative distance going forward on the input
        Utilize the DriveReverse parameter in the module constructors to ensure that each module is spinning in the correct direction
       * 
       */
      //Gyro intialization process
      gyro = new PigeonIMU(gyroport);
     
      gyro.setYaw(0);

      // pass in vision subsystem
      this.vision = vision;

      //initialize limelight variables
      this.m_shooterLimelight = shooterLimelight;

      //Odometry Initialization
      this.odometry =  new SwerveDrivePoseEstimator(
            kinematics,
            Rotation2d.fromDegrees(gyro.getYaw()),
            new SwerveModulePosition[] {
              LFMod.GetPosition(),
              RFMod.GetPosition(),
              LBMod.GetPosition(),
              RBMod.GetPosition()
            },
            new Pose2d(0, 0, new Rotation2d(0)),
             constants.STATE_STANDARD_DEVIATIONS,
            constants.VISION_MEASUREMENT_STANDARD_DEVIATIONS);
      
      // this.odometry = new SwerveDriveOdometry(
      //   kinematics,
      //   Rotation2d.fromDegrees(gyro.getYaw()),
      //       new SwerveModulePosition[] {
      //         LFMod.GetPosition(),
      //         RFMod.GetPosition(),
      //         LBMod.GetPosition(),
      //         RBMod.GetPosition()
      //       }
      //   );

      v_prevPose = SwerveOdometryGetPose();

      //Auto Holonomic controller configuration sets
      AutoBuilder.configureHolonomic(
        this::SwerveOdometryGetPose, // Robot pose supplier
        this::resetPose, // Method to reset odometry (will be called if your auto has a starting pose)
        this::getSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
        this::driveRobotRelative, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
        new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
                new PIDConstants(constants.k_AutoXYP, constants.k_AutoXYI, constants.k_AutoXYD), // Translation PID constants
                new PIDConstants(constants.k_AutoRotP, constants.k_AutoRotI, constants.k_AutoRotD), // Rotation PID constants
                constants.kMaxSpeed, // Max module speed, in m/s
                constants.k_DriveBaseRadius, // Drive base radius in meters. Distance from robot center to furthest module.
                new ReplanningConfig() // Default path replanning config. See the API for the options here
        ),
        () -> {
            var alliance = DriverStation.getAlliance();
            if (alliance.isPresent()) {
                return alliance.get() == DriverStation.Alliance.Red;
            }
            return false;
          },this);
  }
  public void Drive(double xSpeed,double ySpeed,double rot, boolean fieldRelative, boolean isJoystick){
    if(isJoystick && constants.k_isRed){
      xSpeed = -xSpeed;
      ySpeed = -ySpeed;
    }
      SwerveModuleState[] states = kinematics.toSwerveModuleStates(
          fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(
                              xSpeed, ySpeed, rot, getAngle()/*Rotation2d.fromDegrees(gyro.getYaw())*/)
                        : new ChassisSpeeds(xSpeed, ySpeed, rot));
    
      SwerveDriveKinematics.desaturateWheelSpeeds(states, constants.kMaxSpeed);
        //Wheel states as order declared in kinematics constructor
        RFMod.SetDesiredState(states[1]);
        RBMod.SetDesiredState(states[3]);
        LBMod.SetDesiredState(states[2]);
        LFMod.SetDesiredState(states[0]);
  }
  public ChassisSpeeds getSpeeds() {
    return kinematics.toChassisSpeeds(getModuleStates());
  }
  public void driveRobotRelative(ChassisSpeeds robotRelativeSpeeds) {
    ChassisSpeeds targetSpeeds = ChassisSpeeds.discretize(robotRelativeSpeeds, 0.02);
    SwerveModuleState[] targetStates = kinematics.toSwerveModuleStates(targetSpeeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(targetStates, constants.kMaxSpeed);
    //Wheel states as order declared in kinematics constructor
    RFMod.SetDesiredState(targetStates[1]);
    RBMod.SetDesiredState(targetStates[3]);
    LBMod.SetDesiredState(targetStates[2]);
    LFMod.SetDesiredState(targetStates[0]);
  }
  //Method to check if odometry setpoint is reached uses temp setpoint
  public boolean OdometryAtSetpoint(){
    if(Math.abs(this.tempSetpoint.getX()-this.SwerveOdometryGetPose().getX())<constants.k_OdometryToleranceX &&
       Math.abs(this.tempSetpoint.getY()-this.SwerveOdometryGetPose().getY())<constants.k_OdometryToleranceY &&
       Math.abs(this.tempSetpoint.getRotation().getRadians()-this.SwerveOdometryGetPose().getRotation().getRadians())<constants.k_OdometryToleranceRot){
      return true;
    }else{
      return false;
    }
    
  }
  public void setModuleStates(SwerveModuleState[] states){
    SwerveDriveKinematics.desaturateWheelSpeeds(states, constants.kMaxSpeed);
    //Wheel states as order declared in kinematics constructor
        RFMod.SetDesiredState(states[1]);
        RBMod.SetDesiredState(states[3]);
        LBMod.SetDesiredState(states[2]);
        LFMod.SetDesiredState(states[0]);
  }
  public void UpdateOdometry() {
      //Wheel states as order declared in kinematics constructor
      odometry.update(Rotation2d.fromDegrees(gyro.getYaw()),
                        new SwerveModulePosition[]{
                        LFMod.GetPosition(), 
                        RFMod.GetPosition(),
                        LBMod.GetPosition(), 
                        RBMod.GetPosition()});
      // Pose2d current = this.SwerveOdometryGetPose();
      // Pose2d updated = cam.estimatedAveragedPose(current,m_timer.get());
      // this.tempSetpoint = updated;
      // if(Math.abs(updated.getX()-current.getX())<constants.k_OdometryToleranceX &&
      //  Math.abs(updated.getY()-this.SwerveOdometryGetPose().getY())<constants.k_OdometryToleranceY &&
      //  Math.abs(updated.getRotation().getRadians()-current.getRotation().getRadians())<constants.k_OdometryToleranceRot){
      //   m_timer.restart();
      // }                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                            
      // odometry.resetPosition(Rotation2d.fromDegrees(gyro.getYaw()),
      //             new SwerveModulePosition[]{
      //             LFMod.GetPosition(), 
      //             RFMod.GetPosition(),
      //             LBMod.GetPosition(), 
      //             RBMod.GetPosition()},updated);
  }  
  public Pose2d SwerveOdometryGetPose(){
      return odometry.getEstimatedPosition();
  }
  public Rotation2d getAngle(){
      //return Rotation2d.fromRadians(((gyro.GetYaw()*((constants.k_PI)/(180.0)))));
      return SwerveOdometryGetPose().getRotation();
  }
  // public Rotation2d getPitchRad(){
  //     return new Rotation2d((gyro.GetPitch()*((constants.k_PI)/(180.0))));
  // }
  // public Rotation2d getRolRad(){
  //     return new Rotation2d((gyro.GetRoll()*((constants.k_PI)/(180.0))));
  // }
  public void ResetDrive(){
      LFMod.ResetEncoder();
      LBMod.ResetEncoder();
      RFMod.ResetEncoder();
      RBMod.ResetEncoder();
  }
  public SwerveModulePosition[] getModulePositions(){
    return new SwerveModulePosition[] {
                                    LFMod.GetPosition(),
                                    RFMod.GetPosition(),
                                    LBMod.GetPosition(),
                                    RBMod.GetPosition()};
  }
  public SwerveModuleState[] getModuleStates(){
    return new SwerveModuleState[] {
                                    LFMod.GetState(),
                                    RFMod.GetState(),
                                    LBMod.GetState(),
                                    RBMod.GetState()};
  }
  public void resetPose(Pose2d pose) {
    odometry.resetPosition(Rotation2d.fromDegrees(gyro.getYaw()),this.getModulePositions(), pose);
  }
  public double getXSpeed() {
    return v_xSpeed;
  }
  public double getYSpeed() {
    return v_ySpeed;
  }
  public Rotation2d getRotSpeed() {
    return Rotation2d.fromRadians(v_rotSpeed);
  }
  public double getTransSpeed() {
    return Math.sqrt(Math.pow(getXSpeed(), 2) + Math.pow(getYSpeed(), 2));
  }
  public Pose2d replaceRotWithGyro(Pose2d pose) {
    return new Pose2d(pose.getX(), pose.getY(), getAngle());
  }
  private Matrix<N3, N1> confidenceCalculator(EstimatedRobotPose estimation) {
    if (estimation.targetsUsed.size() >= 2) {
      return constants.MULTI_TAG_STANDARD_DEVIATIONS;
    }
    else {
      return constants.SINGLE_TAG_STANDARD_DEVIATIONS;
    }
  }

  public void updateEstimates(PoseEstimate... poses) {
    for (int i = 0; i < poses.length; i++) {
      odometry.addVisionMeasurement(
          poses[i].estimatedPose().estimatedPose.toPose2d(),
          poses[i].estimatedPose().timestampSeconds,
          poses[i].standardDev());
    }
  }

  public void updatePoseWithVision(Photon photon) {
    photon.update();
    EstimatedRobotPose estimatedRobotPose = photon.getLatestEstimatedPose();
    if (estimatedRobotPose != null) {
      var pose2d = estimatedRobotPose.estimatedPose.toPose2d();
      odometry.addVisionMeasurement(pose2d, estimatedRobotPose.timestampSeconds,
            confidenceCalculator(estimatedRobotPose));
    }
  }
/* 
  public void updatePoseEstimatorWithVisionBotPose(Limelight limelight) {
    Pose2d visionBotPose = limelight.getBotPose();
    // invalid LL data
    if (doublesAreEqual(visionBotPose.getX(), 0.0)) {
      SmartDashboard.putBoolean("vision measurement added?", false);
      return;
    }

    // distance from current pose to vision estimated pose
    double poseDifference = odometry.getEstimatedPosition().getTranslation()
        .getDistance(visionBotPose.getTranslation());

    if (limelight.hasTarget()) {
      double xyStds;
      double degStds;
      // multiple targets detected
      if (limelight.getTagCount() >= 2) {
        xyStds = 0.5;
        degStds = 6;
      }
      // 1 target with large area and close to estimated pose
      else if (limelight.getTA() > 0.8 && poseDifference < 0.5) {
        xyStds = 1.0;
        degStds = 12;
      }
      // 1 target farther away and estimated pose is close
      else if (limelight.getTA() > 0.1 && poseDifference < 0.3) {
        xyStds = 2.0;
        degStds = 30;
      }
      // conditions don't match to add a vision measurement
      else {
        SmartDashboard.putBoolean("vision measurement added?", false);
        return;
      }

      SmartDashboard.putBoolean("vision measurement added?", true);
      SmartDashboard.putNumber("xyStds", xyStds);
      //SmartDashboard.putNumber("degStds", degStds);

      odometry.setVisionMeasurementStdDevs(VecBuilder.fill(xyStds, xyStds, 9999999));
      odometry.addVisionMeasurement(visionBotPose, Timer.getFPGATimestamp() - limelight.getLatencyMilliseconds()/1000.0);
    }
    else {
      SmartDashboard.putBoolean("vision measurement added?", false);
    }
  }
*/
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    this.UpdateOdometry();
    m_field.setRobotPose(this.SwerveOdometryGetPose());

    // adding vision measurements if the limelight has a target and it is a new measurement
    // if (m_shooterLimelight.hasTarget() &&
    //   !doublesAreEqual(Timer.getFPGATimestamp()-m_shooterLimelight.getLatencyMilliseconds()/1000.0, v_prevShooterLLTimestamp)) {
    //     odometry.addVisionMeasurement(replaceRotWithGyro(m_shooterLimelight.getBotPose()), Timer.getFPGATimestamp()-m_shooterLimelight.getLatencyMilliseconds()/1000.0);
    //     v_prevShooterLLTimestamp = Timer.getFPGATimestamp()-m_shooterLimelight.getLatencyMilliseconds()/1000.0;      
    // }
    //updatePoseEstimatorWithVisionBotPose(m_shooterLimelight);
    
    // updating pose estimator with vision estimates
    updateEstimates(vision.getEstimatedGlobalPoses());

    // updating robot speeds
    v_xSpeed = (SwerveOdometryGetPose().getX()-v_prevPose.getX())/0.02;
    v_ySpeed = (SwerveOdometryGetPose().getY()-v_prevPose.getY())/0.02;
    v_rotSpeed = (SwerveOdometryGetPose().getRotation().getRadians()-v_prevPose.getRotation().getRadians())/0.02;
    v_prevPose = SwerveOdometryGetPose();

    SmartDashboard.putNumber("Gyro Angle", gyro.getYaw());

    SmartDashboard.putNumber("front left ", LFMod.GetCurrentAngle());
    SmartDashboard.putNumber("front right ", RFMod.GetCurrentAngle());
    SmartDashboard.putNumber("back left ", LBMod.GetCurrentAngle());
    SmartDashboard.putNumber("back right ", RBMod.GetCurrentAngle());
    // SmartDashboard.putNumber("motor positionofamotoer", RBMod.m_Drive.getPosition().getValueAsDouble());
    SmartDashboard.putNumber("x", SwerveOdometryGetPose().getX());
    SmartDashboard.putNumber("displacement", Math.sqrt(Math.pow(SwerveOdometryGetPose().getX(), 2)+Math.pow(SwerveOdometryGetPose().getY(), 2)));
    SmartDashboard.putNumber("y", SwerveOdometryGetPose().getY());
    // SmartDashboard.putNumber("rot deg", SwerveOdometryGetPose().getRotation().getDegrees());

    // SmartDashboard.putNumber("gyro angle", this.getAngle().getRadians());

    /*SmartDashboard.putNumber("vision x", m_shooterLimelight.getBotX());
    SmartDashboard.putNumber("vision y", m_shooterLimelight.getBotY());
    SmartDashboard.putNumber("vision rot deg", m_shooterLimelight.getBotYaw());

    SmartDashboard.putNumber("shooter ll num targets detected", m_shooterLimelight.getNumTargets());
      */
    // AdvantageKit logging
    Logger.recordOutput("SwerveModuleStates", getModuleStates());
    Logger.recordOutput("RobotPose", SwerveOdometryGetPose());
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  boolean doublesAreEqual(double d1, double d2) {
    return (Math.abs(d1-d2) < 0.00001);
  }
}
