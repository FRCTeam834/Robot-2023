// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.utility.PIDGains;

public class SwerveModule extends SubsystemBase {
  
  private final String name;

  private final CANSparkMax steerMotor;
  private final CANSparkMax driveMotor;

  private final SparkMaxPIDController steerController;
  private final SparkMaxPIDController driveController;

  private final RelativeEncoder steerEncoder;
  private final RelativeEncoder driveEncoder;

  private final PIDGains steerPIDGains;
  private final PIDGains drivePIDGains;

  public final static SwerveModule buildModule (
    String name,
    int STEERID,
    int DRIVEID,
    PIDGains steerPID,
    PIDGains drivePID
  ) {
    /* Clone pid gains so each module has unique instance (important if the gains are tuneable) */
    steerPID = steerPID.clone(name);
    drivePID = drivePID.clone(name);

    SwerveModule module = new SwerveModule(name, STEERID, DRIVEID, steerPID, drivePID);
    CANSparkMax steerMotor = module.getSteerMotor();
    CANSparkMax driveMotor = module.getDriveMotor();
    SparkMaxPIDController steerController = module.getSteerController();
    SparkMaxPIDController driveController = module.getDriveController();
    RelativeEncoder steerEncoder = module.getSteerEncoder();
    RelativeEncoder driveEncoder = module.getDriveEncoder();

    /* Set PID gains to the SparkMax PID controllers */
    steerPID.bindToController(steerController);
    drivePID.bindToController(driveController);

    /* Enables continous input; 360 becomes 0 etc. */
    steerController.setPositionPIDWrappingEnabled(true);
    steerController.setPositionPIDWrappingMaxInput(359);
    steerController.setPositionPIDWrappingMaxInput(0);

    // revolutions => degrees
    steerEncoder.setPositionConversionFactor(360.0 / Constants.DRIVETRAIN.STEER_GEAR_RATIO);
    // revolutions => meters
    driveEncoder.setPositionConversionFactor(Math.PI * Constants.DRIVETRAIN.WHEEL_DIAMETER / Constants.DRIVETRAIN.DRIVE_GEAR_RATIO);
    // rev/min => meters/sec
    driveEncoder.setVelocityConversionFactor(Math.PI * Constants.DRIVETRAIN.WHEEL_DIAMETER / (60 * Constants.DRIVETRAIN.DRIVE_GEAR_RATIO));

    /* Do not touch these unless you know what you are doing */
    steerMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 10);
    steerMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 20);
    steerMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 20);
    steerMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 50);
    steerMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus4, 20);
    steerMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 200);
    steerMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus6, 200);

    driveMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 10);
    driveMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 20);
    driveMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 20);
    driveMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 50);
    driveMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus4, 20);
    driveMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 200);
    driveMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus6, 200);

    if (Constants.competitionMode) {
      /* Protection against brownouts */
      module.getSteerMotor().burnFlash();
      module.getDriveMotor().burnFlash();
    }

    return module;
  }

  /* Module "factory" methods */
  public final static SwerveModule buildFrontLeft () {
    return SwerveModule.buildModule(
      "FL_MODULE",
      Constants.CANIDS.FL_STEER,
      Constants.CANIDS.FL_DRIVE,
      Constants.PIDGAINS.MODULE_STEER,
      Constants.PIDGAINS.MODULE_DRIVE
    );
  }

  public final static SwerveModule buildFrontRight () {
    return SwerveModule.buildModule(
      "FR_MODULE",
      Constants.CANIDS.FR_STEER,
      Constants.CANIDS.FR_DRIVE,
      Constants.PIDGAINS.MODULE_STEER,
      Constants.PIDGAINS.MODULE_DRIVE
    );
  }

  public final static SwerveModule buildBackLeft () {
    return SwerveModule.buildModule(
      "BL_MODULE",
      Constants.CANIDS.BL_STEER,
      Constants.CANIDS.BL_DRIVE,
      Constants.PIDGAINS.MODULE_STEER,
      Constants.PIDGAINS.MODULE_DRIVE
    );
  }

  public final static SwerveModule buildBackRight () {
    return SwerveModule.buildModule(
      "BR_MODULE",
      Constants.CANIDS.BR_STEER,
      Constants.CANIDS.BR_DRIVE,
      Constants.PIDGAINS.MODULE_STEER,
      Constants.PIDGAINS.MODULE_DRIVE
    );
  }

  /** Constructor */
  private SwerveModule(
    String name, // Module name
    int STEERID, // CANID of steer motor
    int DRIVEID, // CANID of drive motor
    PIDGains steerPID, // PID gains for steer controller
    PIDGains drivePID // PID gains for drive controller
  ) {
    this.name = name;

    steerMotor = new CANSparkMax(STEERID, CANSparkMaxLowLevel.MotorType.kBrushless);
    driveMotor = new CANSparkMax(DRIVEID, CANSparkMaxLowLevel.MotorType.kBrushless);

    steerController = steerMotor.getPIDController();
    driveController = driveMotor.getPIDController();

    steerEncoder = steerMotor.getEncoder();
    driveEncoder = driveMotor.getEncoder();

    steerPIDGains = steerPID;
    drivePIDGains = drivePID;
  }

  /* Getters */
  public CANSparkMax getSteerMotor () {
    return this.steerMotor;
  }

  public CANSparkMax getDriveMotor () {
    return this.driveMotor;
  }

  public SparkMaxPIDController getSteerController () {
    return this.steerController;
  }

  public SparkMaxPIDController getDriveController () {
    return this.driveController;
  }

  public RelativeEncoder getSteerEncoder () {
    return this.steerEncoder;
  }

  public RelativeEncoder getDriveEncoder () {
    return this.driveEncoder;
  }

  public double getCurrentAngle () {
    return this.steerEncoder.getPosition();
  }

  public Rotation2d getCurrentAngleAsRotation2d () {
    return Rotation2d.fromDegrees(this.getCurrentAngle());
  }

  public double getCurrentVelocity () {
    return this.driveEncoder.getVelocity();
  }

  public double getCurrentPosition () {
    return this.driveEncoder.getPosition();
  }

  public SwerveModulePosition getModulePosition () {
    return new SwerveModulePosition(this.getCurrentPosition(), this.getCurrentAngleAsRotation2d());
  }

  /**
   * @param angle - desired module angle
   */
  public void setDesiredAngle (Rotation2d angle) {
    this.steerController.setReference(angle.getDegrees(), CANSparkMax.ControlType.kPosition);
  }

  /**
   * @param speed - desired speed in m/s
   */
  public void setDesiredSpeed (double speed) {
    this.driveController.setReference(speed, CANSparkMax.ControlType.kVelocity);
  }

  /**
   * @param speed - speed in m/s
   */
  public void setSpeed (double speed) {
    this.driveMotor.set(speed / Constants.DRIVETRAIN.MAX_MODULE_SPEED);
  }

  /** Stop module */
  public void halt () {
    this.steerMotor.set(0);
    this.driveMotor.set(0);
  }

  /**
   * Set desired module state
   * @param desiredState
   */
  public void setDesiredState (SwerveModuleState desiredState) {
    desiredState = SwerveModule.optimize(desiredState, this.getCurrentAngleAsRotation2d(), Constants.DRIVETRAIN.MODULE_OPTIMIZE_THRESHOLD);

    if (Math.abs(desiredState.speedMetersPerSecond) < Constants.DRIVETRAIN.MODULE_MOVE_THRESHOLD) {
      this.halt();
      return;
    }

    this.setDesiredAngle(desiredState.angle);
    this.setDesiredSpeed(desiredState.speedMetersPerSecond);
  }

  /**
   * Set desired module state open loop
   * @param desiredState
   */
  public void setDesiredStateOpenLoop (SwerveModuleState desiredState) {
    desiredState = SwerveModule.optimize(desiredState, this.getCurrentAngleAsRotation2d(), Constants.DRIVETRAIN.MODULE_OPTIMIZE_THRESHOLD);

    if (Math.abs(desiredState.speedMetersPerSecond) < Constants.DRIVETRAIN.MODULE_MOVE_THRESHOLD) {
      this.halt();
      return;
    }

    this.setDesiredAngle(desiredState.angle);
    this.setSpeed(desiredState.speedMetersPerSecond);
  }

  @Override
  public void initSendable (SendableBuilder builder) {
    if (Constants.telemetryMode == false) return;

    builder.setSmartDashboardType("Swerve Module " + this.name);
    builder.addDoubleProperty("Angle", this::getCurrentAngle, null);
    builder.addDoubleProperty("Velocity", this::getCurrentVelocity, null);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if (Constants.tuningMode) {
      if (this.steerPIDGains.hasChanged()) {
        this.steerController.setP(this.steerPIDGains.getP());
        this.steerController.setI(this.steerPIDGains.getI());
        this.steerController.setD(this.steerPIDGains.getD());
      }
      if (this.drivePIDGains.hasChanged()) {
        this.driveController.setP(this.drivePIDGains.getP());
        this.driveController.setI(this.drivePIDGains.getI());
        this.driveController.setD(this.drivePIDGains.getD());
      }
    }
  }

  /** 
   * SwerveModuleState optimize but with customizable threshold
   * @param threshold - angle before optimization in degrees
   */
  public static SwerveModuleState optimize (
    SwerveModuleState desiredState, Rotation2d currentAngle, double threshold
  ) {
    Rotation2d delta = desiredState.angle.minus(currentAngle);
    if (Math.abs(delta.getDegrees()) > threshold) {
      return new SwerveModuleState(
        -desiredState.speedMetersPerSecond,
        desiredState.angle.rotateBy(Rotation2d.fromDegrees(180.0))
      );
    }
    return desiredState;
  }
}
