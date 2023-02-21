// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.pathplanner.lib.PathPlannerTrajectory;
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
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.DriveTrainConstants;

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

  private ChassisSpeeds lastChassisSpeeds;

  private final ProfiledPIDController keepHeadingController = new ProfiledPIDController(0.0, 0.0, 0.0, DriveTrainConstants.AUTON_CONSTRAINTS);
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

    DriveTrainConstants.KEEP_HEADING_PID_GAINS.bindToController(keepHeadingController);
    keepHeadingController.enableContinuousInput(0, 2 * Math.PI);
    keepHeadingController.setTolerance(DriveTrainConstants.KEEP_HEADING_SETPOINT_TOLERANCE);
  }

  public SwerveDriveKinematics getKinematics () {
    return kinematics;
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

  /**
   * Drive open loop
   * @param vx - x velocity
   * @param vy - y velocity
   * @param omega - angular velocity
   */
  public void driveOpenLoop (
    double vx,
    double vy,
    double omega
  ) {
    double angle = Math.atan2(vy, vx);
    double mag = translationRateLimiter.calculate(Math.hypot(vx, vy));
    vx = mag * Math.cos(angle);
    vy = mag * Math.sin(angle);
    omega = steerRateLimiter.calculate(omega);

    ChassisSpeeds speeds = new ChassisSpeeds(vx, vy, omega);

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
      if (DriveTrainConstants.DRIVE_PID_GAINS.hasChanged()) {
        DriveTrainConstants.DRIVE_PID_GAINS.bindToController(frontLeft.getDriveController());
        DriveTrainConstants.DRIVE_PID_GAINS.bindToController(frontRight.getDriveController());
        DriveTrainConstants.DRIVE_PID_GAINS.bindToController(backLeft.getDriveController());
        DriveTrainConstants.DRIVE_PID_GAINS.bindToController(backRight.getDriveController());
      }
      if (DriveTrainConstants.STEER_PID_GAINS.hasChanged()) {
        DriveTrainConstants.STEER_PID_GAINS.bindToController(frontLeft.getSteerController());
        DriveTrainConstants.STEER_PID_GAINS.bindToController(frontRight.getSteerController());
        DriveTrainConstants.STEER_PID_GAINS.bindToController(backLeft.getSteerController());
        DriveTrainConstants.STEER_PID_GAINS.bindToController(backRight.getSteerController());
      }
    }
  }

  /**
   * Modifies chassis speeds to maintain the last desired robot direction
   * Counteracts swerve drift
   * @param desiredSpeeds - desired robot ChassisSpeeds
   * @return converted chassis speeds
   */
  private ChassisSpeeds keepHeading (ChassisSpeeds desiredSpeeds) {
    double currentTime = Timer.getMatchTime();
    if (
      desiredSpeeds.omegaRadiansPerSecond > DriveTrainConstants.KEEP_HEADING_OMEGA_THRESHOLD ||
      currentTime - lastKeepHeadingTime < DriveTrainConstants.KEEP_HEADING_TIME_THRESHOLD
    ) {
      /** Numerically integrate new desired heading */
      keepHeading = gyro.getYawAsRotation2d().plus(Rotation2d.fromRadians(desiredSpeeds.omegaRadiansPerSecond).times(0.02));
      keepHeadingController.setGoal(keepHeading.getDegrees());
      lastKeepHeadingTime = currentTime;
      return desiredSpeeds;
    }

    ChassisSpeeds convertedChassisSpeeds = new ChassisSpeeds(
      desiredSpeeds.vxMetersPerSecond,
      desiredSpeeds.vyMetersPerSecond,
      keepHeadingController.calculate(Units.degreesToRadians(gyro.getYaw()))
    );
    
    if (keepHeadingController.atGoal()) {
      return desiredSpeeds;
    }

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
        this::setDesiredModuleStates,
        this
      )
    );
  }
}
