// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.DriveTrainConstants;

public class SwerveModule extends SubsystemBase {
  private final String name;
  
  private final CANSparkMax driveMotor;
  private final CANSparkMax steerMotor;

  private final RelativeEncoder driveEncoder;
  private final AbsoluteEncoder steerEncoder;

  private final SparkMaxPIDController driveController;
  private final SparkMaxPIDController steerController;

  private final SimpleMotorFeedforward driveFeedforward;

  private double velocitySetpoint = 0.0;
  private double angleSetpoint = 0.0;

  //private final double encoderOffset;

  /** Builder methods */
  public static final SwerveModule buildFrontLeft () {
    return new SwerveModule(0);
  }

  public static final SwerveModule buildFrontRight () {
    return new SwerveModule(1);
  }

  public static final SwerveModule buildBackLeft () {
    return new SwerveModule(2);
  }

  public static final SwerveModule buildBackRight () {
    return new SwerveModule(3);
  }

  /**
   * @param moduleID - index of module in order of [FL, FR, BL, BR]
   */
  private SwerveModule(int moduleID) {
    name = DriveTrainConstants.MODULE_NAMES[moduleID];

    driveMotor = new CANSparkMax(DriveTrainConstants.CANIDS[moduleID][1], CANSparkMaxLowLevel.MotorType.kBrushless);
    steerMotor = new CANSparkMax(DriveTrainConstants.CANIDS[moduleID][0], CANSparkMaxLowLevel.MotorType.kBrushless);

    driveEncoder = driveMotor.getEncoder();
    steerEncoder = steerMotor.getAbsoluteEncoder(Type.kDutyCycle);

    //encoderOffset = DriveTrainConstants.ENCODER_OFFSETS[moduleID];

    driveMotor.restoreFactoryDefaults();
    steerMotor.restoreFactoryDefaults();

    driveController = driveMotor.getPIDController();
    steerController = steerMotor.getPIDController();

    driveFeedforward = DriveTrainConstants.DRIVE_FF;

    driveMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
    steerMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
    driveMotor.enableVoltageCompensation(12.0);
    steerMotor.enableVoltageCompensation(12.0);
    driveMotor.setSmartCurrentLimit(DriveTrainConstants.DRIVE_CURRENT_LIMIT);
    steerMotor.setSmartCurrentLimit(DriveTrainConstants.STEER_CURRENT_LIMIT);

    // Duty cycle encoder position packet (default 200ms - bad!)
    driveMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 10);
    steerMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 10);
    steerMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus4, 65535);

    // RPM -> m/s
    driveEncoder.setVelocityConversionFactor(Math.PI * DriveTrainConstants.WHEEL_DIAMETER / (60 * DriveTrainConstants.DRIVE_GEAR_RATIO));
    // Revs -> meters
    driveEncoder.setPositionConversionFactor(Math.PI * DriveTrainConstants.WHEEL_DIAMETER / DriveTrainConstants.DRIVE_GEAR_RATIO);
    // Revs -> radians
    steerEncoder.setPositionConversionFactor(2 * Math.PI / DriveTrainConstants.STEER_GEAR_RATIO);

    steerEncoder.setInverted(true); // MAXSwerve module steer gearing is reversed
    driveEncoder.setPosition(0);

    DriveTrainConstants.DRIVE_PID_GAINS.bindToController(driveController);
    DriveTrainConstants.STEER_PID_GAINS.bindToController(steerController);

    /* Make inputs circular (e.g 361 degrees is equivalent to 1 degree) */
    steerController.setPositionPIDWrappingEnabled(true);
    steerController.setPositionPIDWrappingMinInput(0);
    steerController.setPositionPIDWrappingMaxInput(2 * Math.PI);

    steerEncoder.setZeroOffset(DriveTrainConstants.ENCODER_OFFSETS[moduleID]);

    driveController.setFeedbackDevice(driveEncoder);
    steerController.setFeedbackDevice(steerEncoder);

    /* Burn flash so configurations are saved if brownout */
    if (Constants.competitionMode) {
      // System.out.println(steerEncoder.getZeroOffset());
      // System.out.println(driveMotor.burnFlash());
      steerMotor.burnFlash();
    }

    SmartDashboard.putData("MAXSwerve " + this.name, this);
  }

  public SparkMaxPIDController getDriveController () {
    return driveController;
  }

  public SparkMaxPIDController getSteerController () {
    return driveController;
  }

  /**
   * 
   * @return current wheel angle
   */
  public double getAngle () {
    return steerEncoder.getPosition();
  }

  public Rotation2d getAngleAsRotation2d () {
    return Rotation2d.fromRadians(this.getAngle());
  }

  /**
   * 
   * @return current wheel speed
   */
  public double getVelocity () {
    return driveEncoder.getVelocity();
  }

  public SwerveModulePosition getModulePosition () {
    return new SwerveModulePosition(driveEncoder.getPosition(), this.getAngleAsRotation2d().rotateBy(Rotation2d.fromDegrees(90)));
  }

  public void setDesiredAngle (double angle) {
    steerController.setReference(angle, ControlType.kPosition);
    angleSetpoint = angle;
  }

  public void setDesiredSpeed (double speed) {
    driveController.setReference(speed, ControlType.kVelocity, 0, driveFeedforward.calculate(speed));
    velocitySetpoint = speed;
  }

  /**
   * 
   * Set speed open loop; no feedback control
   * @param speed
   */
  public void setSpeed (double speed) {
    driveMotor.set(speed / DriveTrainConstants.MAX_MODULE_SPEED);
  }

  /**
   * 
   * Set desired speed and angle of module
   * @param desiredState
   */
  public void setDesiredState (SwerveModuleState desiredState) {
    //desiredState.angle = desiredState.angle.plus(Rotation2d.fromRadians(encoderOffset));
    desiredState = SwerveModule.optimize(desiredState, this.getAngleAsRotation2d(), Units.degreesToRadians(90.0));

    if (Math.abs(desiredState.speedMetersPerSecond) < DriveTrainConstants.MODULE_ACTIVATION_THRESHOLD) {
      this.stop();
      return;
    }

    this.setDesiredAngle(desiredState.angle.getRadians());
    this.setDesiredSpeed(desiredState.speedMetersPerSecond);
  }

  /**
   * 
   * Set desired open loop speed and angle of module
   * @param desiredState
   */
  public void setDesiredStateOpenLoop (SwerveModuleState desiredState) {
    //desiredState.angle = desiredState.angle.plus(Rotation2d.fromRadians(encoderOffset));
    desiredState = SwerveModule.optimize(desiredState, this.getAngleAsRotation2d(), Units.degreesToRadians(90.0));

    if (Math.abs(desiredState.speedMetersPerSecond) < DriveTrainConstants.MODULE_ACTIVATION_THRESHOLD) {
      this.stop();
      return;
    }

    this.setDesiredAngle(desiredState.angle.getRadians());
    this.setSpeed(desiredState.speedMetersPerSecond);
  }

  public void stop () {
    driveMotor.set(0);
    steerMotor.set(0);
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    if (Constants.telemetryMode == false) return;

    builder.setSmartDashboardType("Swerve");
    builder.addDoubleProperty("Angle", () -> this.getAngleAsRotation2d().getRadians(), null);
    builder.addDoubleProperty("Velocity", this::getVelocity, null);
    builder.addDoubleProperty("Angle Setpoint", () -> angleSetpoint, null);
    builder.addDoubleProperty("Velocity Setpoint", () -> velocitySetpoint, null);
  }

  @Override
  public void periodic() {}

  /**
   * 
   * Optimize swerve module state with customizable threshold
   * @param desiredState
   * @param currentAngle
   * @param threshold
   * @return
   */
  public static SwerveModuleState optimize (
    SwerveModuleState desiredState, Rotation2d currentAngle, double threshold
  ) {
    Rotation2d delta = desiredState.angle.minus(currentAngle);
    if (Math.abs(delta.getRadians()) > threshold) {
      return new SwerveModuleState(
        -desiredState.speedMetersPerSecond,
        desiredState.angle.rotateBy(Rotation2d.fromDegrees(180.0))
      );
    }
    return desiredState;
  }
}
