// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.List;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;
import com.pathplanner.lib.commands.FollowPathWithEvents;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Superstructure;
import frc.robot.Constants.DriveTrainConstants;
import frc.robot.commands.ArmToPreset;

public class DriveTrain extends SubsystemBase {
  /* Swerve modules */
  private final SwerveModule frontLeft;
  private final SwerveModule frontRight;
  private final SwerveModule backLeft;
  private final SwerveModule backRight;

  private final Pigeon gyro;
  private final SwerveDriveKinematics kinematics;

  private final SlewRateLimiter translationRateLimiter = new SlewRateLimiter(DriveTrainConstants.TRANSLATION_SLEWRATE);
  private final SlewRateLimiter steerRateLimiter = new SlewRateLimiter(DriveTrainConstants.STEER_SLEWRATE);

  private ChassisSpeeds lastChassisSpeeds = new ChassisSpeeds(0, 0, 0);

  /* Current rotation2d to keep maintain heading to */
  private Rotation2d keepHeading = new Rotation2d();
  private double lastKeepHeadingTime = Timer.getFPGATimestamp();

  public DriveTrain(Pigeon pigeon) {
    frontLeft = SwerveModule.buildFrontLeft();
    frontRight = SwerveModule.buildFrontRight();
    backLeft = SwerveModule.buildBackLeft();
    backRight = SwerveModule.buildBackRight();

    gyro = pigeon;

    kinematics = new SwerveDriveKinematics(DriveTrainConstants.MODULE_POSITIONS);
    SmartDashboard.putData(this);
  }

  public SwerveDriveKinematics getKinematics () {
    return kinematics;
  }

  public ChassisSpeeds getLastChassisSpeeds () {
    return lastChassisSpeeds;
  }

  /**
   * @return current module positions
   */
  public SwerveModulePosition[] getModulePositions () {
    return new SwerveModulePosition[] {
      frontLeft.getModulePosition(),
      frontRight.getModulePosition(),
      backLeft.getModulePosition(),
      backRight.getModulePosition()
    };
  }

  /**
   * Set desired state for each module
   * @param desiredStates
   */
  public void setDesiredModuleStates (SwerveModuleState[] desiredStates) {
    //ChassisSpeeds speeds = kinematics.toChassisSpeeds(desiredStates);
    //ChassisSpeeds convertedSpeeds = new ChassisSpeeds(speeds.vyMetersPerSecond, speeds.vxMetersPerSecond, speeds.omegaRadiansPerSecond);
    //desiredStates = kinematics.toSwerveModuleStates(convertedSpeeds);
    frontLeft.setDesiredState(desiredStates[0]);
    frontRight.setDesiredState(desiredStates[1]);
    backLeft.setDesiredState(desiredStates[2]);
    backRight.setDesiredState(desiredStates[3]);
  }

  /**
   * 
   * Set desired openloop state for each module
   * @param desiredStates
   */
  public void setDesiredModuleStatesOpenLoop (SwerveModuleState[] desiredStates) {
    frontLeft.setDesiredStateOpenLoop(desiredStates[0]);
    frontRight.setDesiredStateOpenLoop(desiredStates[1]);
    backLeft.setDesiredStateOpenLoop(desiredStates[2]);
    backRight.setDesiredStateOpenLoop(desiredStates[3]);
  }

  public void lockModules () {
    frontLeft.setDesiredAngle(Units.degreesToRadians(-45));
    frontRight.setDesiredAngle(Units.degreesToRadians(45));
    backLeft.setDesiredAngle(Units.degreesToRadians(45));
    backRight.setDesiredAngle(Units.degreesToRadians(-45));
  }

  public void ppsetDesiredModuleStates (SwerveModuleState[] desiredStates) {
    ChassisSpeeds speeds = kinematics.toChassisSpeeds(desiredStates);
    ChassisSpeeds convertedSpeeds = new ChassisSpeeds(speeds.vyMetersPerSecond, -speeds.vxMetersPerSecond, -speeds.omegaRadiansPerSecond);
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, DriveTrainConstants.MAX_MODULE_SPEED);
    //System.out.println(convertedSpeeds);
    this.setDesiredModuleStates(kinematics.toSwerveModuleStates(convertedSpeeds));
    //this.setDesiredModuleStates(desiredStates);
  }

  /**
   * Drive closed loop
   * @param vx - x velocity
   * @param vy - y velocity
   * @param omega - angular velocity
   */
  public void drive (
    double vx,
    double vy,
    double omega
  ) {
    vx = -vx;
    // Ratelimiter, do not ratelimit if vx or vy are too low as it makes the angle volatile
    if (Math.abs(vx) > DriveTrainConstants.MODULE_ACTIVATION_THRESHOLD || Math.abs(vy) > DriveTrainConstants.MODULE_ACTIVATION_THRESHOLD) {
      double angle = Math.atan2(vy, vx);
      double mag = translationRateLimiter.calculate(Math.hypot(vx, vy));
      vx = mag * Math.cos(angle);
      vy = mag * Math.sin(angle);
    }
    omega = steerRateLimiter.calculate(omega);

    ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(vx, vy, omega, gyro.getYawAsRotation2d());
    //ChassisSpeeds speeds = new ChassisSpeeds(vx, vy, omega);
    speeds = keepHeading(speeds);

    SwerveModuleState[] desiredStates = kinematics.toSwerveModuleStates(speeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(
      desiredStates,
      lastChassisSpeeds,
      DriveTrainConstants.MAX_MODULE_SPEED,
      DriveTrainConstants.MAX_TRANSLATION_SPEED,
      DriveTrainConstants.MAX_STEER_SPEED
    );

    this.setDesiredModuleStates(desiredStates);
    lastChassisSpeeds = speeds;
  }

  public void driveOpenLoop (
    double vx,
    double vy,
    double omega
  ) {
    vx = -vx;
    // Ratelimiter, do not ratelimit if vx or vy are too low as it makes the angle volatile
    if (Math.abs(vx) > DriveTrainConstants.MODULE_ACTIVATION_THRESHOLD || Math.abs(vy) > DriveTrainConstants.MODULE_ACTIVATION_THRESHOLD) {
      double angle = Math.atan2(vy, vx);
      double mag = translationRateLimiter.calculate(Math.hypot(vx, vy));
      vx = mag * Math.cos(angle);
      vy = mag * Math.sin(angle);
    }
    omega = steerRateLimiter.calculate(omega);

    ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(vx, vy, omega, gyro.getYawAsRotation2d());
    //ChassisSpeeds speeds = new ChassisSpeeds(vx, vy, omega);
    speeds = keepHeading(speeds);

    SwerveModuleState[] desiredStates = kinematics.toSwerveModuleStates(speeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(
      desiredStates,
      lastChassisSpeeds,
      DriveTrainConstants.MAX_MODULE_SPEED,
      DriveTrainConstants.MAX_TRANSLATION_SPEED,
      DriveTrainConstants.MAX_STEER_SPEED
    );

    this.setDesiredModuleStatesOpenLoop(desiredStates);
    lastChassisSpeeds = speeds;
  }

  /** Stop the drivetrain */
  public void stop () {
    frontLeft.stop();
    frontRight.stop();
    backLeft.stop();
    backRight.stop();
  }

  @Override
  public void periodic() {
    if (Constants.tuningMode) {
      /* Real time PID tuning -- same PID gains for each module */
      //if (DriveTrainConstants.DRIVE_PID_GAINS.hasChanged()) {
        //DriveTrainConstants.DRIVE_PID_GAINS.bindToController(frontLeft.getDriveController());
        //DriveTrainConstants.DRIVE_PID_GAINS.bindToController(frontRight.getDriveController());
        //DriveTrainConstants.DRIVE_PID_GAINS.bindToController(backLeft.getDriveController());
        //DriveTrainConstants.DRIVE_PID_GAINS.bindToController(backRight.getDriveController());
      //}
      //if (DriveTrainConstants.STEER_PID_GAINS.hasChanged()) {
        //DriveTrainConstants.STEER_PID_GAINS.bindToController(frontLeft.getSteerController());
        //DriveTrainConstants.STEER_PID_GAINS.bindToController(frontRight.getSteerController());
        //DriveTrainConstants.STEER_PID_GAINS.bindToController(backLeft.getSteerController());
        //DriveTrainConstants.STEER_PID_GAINS.bindToController(backRight.getSteerController());
      //}
    }
  }

  /**
   * Modifies chassis speeds to maintain the last desired robot direction
   * Counteracts swerve drift
   * @param desiredSpeeds - desired robot ChassisSpeeds
   * @return converted chassis speeds
   */
  private ChassisSpeeds keepHeading (ChassisSpeeds desiredSpeeds) {
    double currentTime = Timer.getFPGATimestamp();
    if (
      Math.abs(desiredSpeeds.omegaRadiansPerSecond) > DriveTrainConstants.KEEP_HEADING_OMEGA_THRESHOLD
    ) {
      /** Numerically integrate new desired heading */
      keepHeading = gyro.getYawAsRotation2d().plus(Rotation2d.fromRadians(desiredSpeeds.omegaRadiansPerSecond).times(0.02));
      lastKeepHeadingTime = currentTime;
      return desiredSpeeds;
    }

    //if (currentTime - lastKeepHeadingTime < DriveTrainConstants.KEEP_HEADING_TIME_THRESHOLD) {
    //  return desiredSpeeds;
    //}
    
    if (Math.abs(gyro.getYawAsRotation2d().minus(keepHeading).getRadians()) < 0.04) return desiredSpeeds;

    ChassisSpeeds convertedChassisSpeeds = new ChassisSpeeds(
      desiredSpeeds.vxMetersPerSecond,
      desiredSpeeds.vyMetersPerSecond,
      Math.min(Math.max(gyro.getYawAsRotation2d().minus(keepHeading).getRadians(), -0.08), 0.08)
    );

    return convertedChassisSpeeds;
  }

  /**
   * Returns command that follows a pathplanner trajectory
   * @param trajectory - trajectory to follow
   * @param poseEstimator - pose estimator instance
   * @param resetOdometry - whether or not to reset odometry position
   * @return the follow trajectory command
   */
  public Command followTrajectoryCommand (
    PathPlannerTrajectory trajectory,
    PoseEstimator poseEstimator,
    boolean resetOdometry
  ) {
    return new SequentialCommandGroup(
      new InstantCommand(() -> {
        if (resetOdometry) {
          poseEstimator.resetOdometry(trajectory.getInitialHolonomicPose());
        }
      }),
      new PPSwerveControllerCommand(
        trajectory,
        poseEstimator::getEstimatedPose,
        kinematics,
        DriveTrainConstants.AUTON_DRIVE_PID_GAINS.derivePIDController(),
        DriveTrainConstants.AUTON_DRIVE_PID_GAINS.derivePIDController(),
        DriveTrainConstants.AUTON_STEER_PID_GAINS.derivePIDController(),
        this::ppsetDesiredModuleStates,
        this
      )
    );
  }

  public Command followEventTrajectoryCommand (
    List<PathPlannerTrajectory> trajectory,
    PoseEstimator poseEstimator,
    boolean resetOdometry
  ) {

    SwerveAutoBuilder autoBuilder = new SwerveAutoBuilder(
      poseEstimator::getEstimatedPose,
      poseEstimator::resetOdometry,
      kinematics,
      new PIDConstants(DriveTrainConstants.AUTON_DRIVE_PID_GAINS.getP(), 0, 0),
      new PIDConstants(DriveTrainConstants.AUTON_STEER_PID_GAINS.getP(), 0, 0),
      this::ppsetDesiredModuleStates,
      Superstructure.eventMap,
      true,
      this
    );

    return autoBuilder.fullAuto(trajectory);
  }
}
