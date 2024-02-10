// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.constants;
import frc.robot.subsystems.Limelight;

public class Drivetrain extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  private final Field2d m_field = new Field2d();
  private SwerveMod RFMod = new SwerveMod(5,6,11,false, constants.k_RFZERO,false,true);
  private SwerveMod RBMod = new SwerveMod(8,7,12,false,constants.k_RBZERO,true,true);
  private SwerveMod LBMod = new SwerveMod(1,2,9,false, constants.k_LBZERO,false,true);
  private SwerveMod LFMod = new SwerveMod(3,4,10,false, constants.k_LFZERO,true,true);
  //Robot Dimensions 27,305 by 29,845 (in inches)
  //Positions defined from a top down view
  public Pose2d tempSetpoint;
  private Translation2d frontLeftLocation = new Translation2d(0.29845, 0.27305);
  private Translation2d frontRightLocation = new Translation2d(0.29845, -0.27305);
  private Translation2d backLeftLocation = new Translation2d(-0.29845, 0.27305);
  private Translation2d backRightLocation = new Translation2d(-0.29845, -0.27304);

  public final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(frontLeftLocation, frontRightLocation, backLeftLocation, backRightLocation); 
  OECPigeionIMU gyro;
  SwerveDrivePoseEstimator odometry;
  Trajectory trajectory;
  //String file = "D:/Temp Robotics/JavaSwerveDriveCommand-Imported/src/main/paths/output/Unnamed.wpilib.json";

  private final Limelight m_shooterLimelight;
  private final Limelight m_elevatorLimelight;

  private double v_prevShooterLLTimestamp = -1;
  private double v_prevElevatorLLTimestamp = -1;

  public Drivetrain(int gyroport, Limelight shooterLimelight, Limelight elevatorLimelight){
      //Motor process:
      /*Decide front back, left and right and assign locations/module names accordingly
        Find the encoder offsets by reading the absolute encoders when all the wheels are lined up in a direction parallel to front
        Log each wheels distance traveled, move forward on the sticks and note any wheels that are moving in a negative distance
        Subtract PI from the absolute encoder offset of the wheels reported in a negative distance going forward on the input
        Utilize the DriveReverse parameter in the module constructors to ensure that each module is spinning in the correct direction
       * 
       */
      gyro = new OECPigeionIMU(gyroport);  
      //gyro.BootCalibrate();
      gyro.ResetYaw();

      m_shooterLimelight = shooterLimelight;
      m_elevatorLimelight = elevatorLimelight;

      //Odometry Initialization
      odometry =  new SwerveDrivePoseEstimator(
            kinematics,
            this.getAngle(),
            new SwerveModulePosition[] {
              LFMod.GetPosition(),
              RFMod.GetPosition(),
              LBMod.GetPosition(),
              RBMod.GetPosition()
            },
            new Pose2d(0, 0, new Rotation2d(0)));

          AutoBuilder.configureHolonomic(
            this::SwerveOdometryGetPose, // Robot pose supplier
            this::resetPose, // Method to reset odometry (will be called if your auto has a starting pose)
            this::getSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
            this::driveRobotRelative, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
            new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
                    new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
                    new PIDConstants(5.0, 0.0, 0.0), // Rotation PID constants
                    constants.kMaxSpeed, // Max module speed, in m/s
                    0.4, // Drive base radius in meters. Distance from robot center to furthest module.
                    new ReplanningConfig() // Default path replanning config. See the API for the options here
            ),
            () -> {
                var alliance = DriverStation.getAlliance();
                if (alliance.isPresent()) {
                    return alliance.get() == DriverStation.Alliance.Red;
                }
                return false;
              },this);
      //Add reset Code
  }
  public void Drive(double xSpeed,double ySpeed,double rot, boolean fieldRelative){


      SwerveModuleState[] states = kinematics.toSwerveModuleStates(
          fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(
                              xSpeed, ySpeed, rot, this.getAngle())
                        : new ChassisSpeeds(xSpeed, ySpeed, rot));
    
      SwerveDriveKinematics.desaturateWheelSpeeds(states, constants.kMaxSpeed);
        //Wheel states as order declared in kinematics constructor
        RFMod.SetDesiredState(states[1]);//3]);
        RBMod.SetDesiredState(states[3]);//1]);
        LBMod.SetDesiredState(states[2]);//0]);
        LFMod.SetDesiredState(states[0]);//2]);
  }
  public ChassisSpeeds getSpeeds() {
    return kinematics.toChassisSpeeds(getModuleStates());
  }
  public void driveRobotRelative(ChassisSpeeds robotRelativeSpeeds) {
    ChassisSpeeds targetSpeeds = ChassisSpeeds.discretize(robotRelativeSpeeds, 0.02);

    SwerveModuleState[] targetStates = kinematics.toSwerveModuleStates(targetSpeeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(targetStates, constants.kMaxSpeed);
    //Wheel states as order declared in kinematics constructor
    RFMod.SetDesiredState(targetStates[1]);//3]);
    RBMod.SetDesiredState(targetStates[3]);//1]);
    LBMod.SetDesiredState(targetStates[2]);//0]);
    LFMod.SetDesiredState(targetStates[0]);//2]);
  }
  public boolean OdometryAtSetpoint(){
    if(Math.abs(this.tempSetpoint.getX()-this.SwerveOdometryGetPose().getX())<0.1&&
       Math.abs(this.tempSetpoint.getY()-this.SwerveOdometryGetPose().getY())<0.1 &&
       Math.abs(this.tempSetpoint.getRotation().getRadians()-this.SwerveOdometryGetPose().getRotation().getRadians())<0.1){
      return true;
    }else{
      return false;
    }
    
  }
  public void setModuleStates(SwerveModuleState[] states){
    SwerveDriveKinematics.desaturateWheelSpeeds(states, constants.kMaxSpeed);
    //Wheel states as order declared in kinematics constructor
        RFMod.SetDesiredState(states[1]);//3]);
        RBMod.SetDesiredState(states[3]);//1]);
        LBMod.SetDesiredState(states[2]);//0]);
        LFMod.SetDesiredState(states[0]);//2]);
  }

  public void UpdateOdometry() {
      //Wheel states as order declared in kinematics constructor
      odometry.updateWithTime(Timer.getFPGATimestamp(),
                        this.getAngle(),
                        new SwerveModulePosition[]{
                        LFMod.GetPosition(), 
                        RFMod.GetPosition(),
                        LBMod.GetPosition(), 
                        RBMod.GetPosition()});
  }  
  public Pose2d SwerveOdometryGetPose(){
      return odometry.getEstimatedPosition();
  }
  public Rotation2d getAngle(){
      return Rotation2d.fromRadians(((gyro.GetYaw()*((constants.k_PI)/(180.0)))));
  }
  public Rotation2d getPitchRad(){
      return new Rotation2d((gyro.GetPitch()*((constants.k_PI)/(180.0))));
  }
  public Rotation2d getRolRad(){
      return new Rotation2d((gyro.GetRoll()*((constants.k_PI)/(180.0))));
  }
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
  odometry.resetPosition(this.getAngle(),this.getModulePositions(), pose);
}
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    this.UpdateOdometry();
    m_field.setRobotPose(this.SwerveOdometryGetPose());

    // adding vision measurements if the limelight has a target and it is a new measurement
    if (m_shooterLimelight.hasTarget() &&
      !doublesAreEqual(Timer.getFPGATimestamp()-m_shooterLimelight.getLatencyMilliseconds()/1000.0, v_prevShooterLLTimestamp)) {
        //System.out.println("adding vision measurement: " + m_shooterLimelight.getWPILibBluePose());
        //System.out.println("previous pose: " + SwerveOdometryGetPose());
        odometry.addVisionMeasurement(m_shooterLimelight.getWPILibBluePose(), Timer.getFPGATimestamp()-m_shooterLimelight.getLatencyMilliseconds()/1000.0);
        v_prevShooterLLTimestamp = Timer.getFPGATimestamp()-m_shooterLimelight.getLatencyMilliseconds()/1000.0;
        //System.out.println("new pose: " + SwerveOdometryGetPose());
        
    }/*
    if (m_elevatorLimelight.hasTarget() &&
      doublesAreEqual(Timer.getFPGATimestamp()-m_elevatorLimelight.getLatencyMilliseconds()/1000.0, v_prevElevatorLLTimestamp)) {
        odometry.addVisionMeasurement(m_elevatorLimelight.getWPILibBluePose(), Timer.getFPGATimestamp()-m_elevatorLimelight.getLatencyMilliseconds()/1000.0);
        v_prevElevatorLLTimestamp = Timer.getFPGATimestamp()-m_elevatorLimelight.getLatencyMilliseconds()/1000.0;
    }*/

    SmartDashboard.putNumber("x", this.SwerveOdometryGetPose().getX());
    //SmartDashboard.putNumber("LFwheeltravel_dist", LFMod.GetPosition().distanceMeters);
    //SmartDashboard.putNumber("LBwheeltravel_dist", LBMod.GetPosition().distanceMeters);
    //SmartDashboard.putNumber("RFwheeltravel_dist", RFMod.GetPosition().distanceMeters);
    //SmartDashboard.putNumber("RBwheeltravel_dist", RBMod.GetPosition().distanceMeters);
    SmartDashboard.putNumber("y", this.SwerveOdometryGetPose().getY());
    SmartDashboard.putNumber("Swerve Angle", this.SwerveOdometryGetPose().getRotation().getDegrees());
    SmartDashboard.putNumber("GyroAngle", this.getAngle().getDegrees());

    // SmartDashboard.putNumber("LF", LFMod.GetAbsEncoderAngle());
    // SmartDashboard.putNumber("RF", RFMod.GetAbsEncoderAngle());
    // SmartDashboard.putNumber("LB", LBMod.GetAbsEncoderAngle());
    // SmartDashboard.putNumber("RB", RBMod.GetAbsEncoderAngle());
    
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
