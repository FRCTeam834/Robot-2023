// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ArmConstants;

public class Arm extends SubsystemBase {
  private final CANSparkMax motor;
  private final RelativeEncoder encoder;

  private final ArmFeedforward feedforward;
  private final ProfiledPIDController controller = new ProfiledPIDController(0.0, 0.0, 0.0, ArmConstants.PROFILE_CONSTRAINTS);

  private boolean isStopped = true;

  /** Constructor */
  public Arm() {
    motor = new CANSparkMax(ArmConstants.CANID, CANSparkMaxLowLevel.MotorType.kBrushless);
    encoder = motor.getEncoder();

    /* Set PID Gains */
    ArmConstants.PID_GAINS.bindToController(controller);

    feedforward = ArmConstants.FEEDFORWARD;

    /* Configurations */

    motor.restoreFactoryDefaults();
    motor.setIdleMode(CANSparkMax.IdleMode.kBrake);
    motor.setInverted(true);
    motor.enableVoltageCompensation(12.0);
    motor.setSmartCurrentLimit(ArmConstants.CURRENT_LIMIT);

    /* Revs -> radians */
    encoder.setPositionConversionFactor(2 * Math.PI / ArmConstants.GEAR_REDUCTION);
    encoder.setVelocityConversionFactor(2 * Math.PI / (ArmConstants.GEAR_REDUCTION * 60));

    encoder.setPosition(ArmConstants.STARTING_POSITION);

    if (Constants.competitionMode) {
      motor.burnFlash();
    }
  }

  /**
   * @return current arm angle rad
   */
  public double getPosition () {
    return this.encoder.getPosition();
  }

  /**
   * @return current arm velocity rad/s
   */
  public double getVelocity () {
    return this.encoder.getVelocity();
  }

  /**
   * Set desired state of arm
   * @param position - arm position in rad
   * @param velocity - arm velocity in rad/s
   */
  public void setDesiredState (double position, double velocity) {
    /* Enforce soft limits */
    position = Math.max(Math.min(position, ArmConstants.MAX_POSITION), ArmConstants.MIN_POSITION);

    isStopped = false;
    controller.reset(this.getPosition(), this.getVelocity());
    controller.setGoal(new TrapezoidProfile.State(position, velocity));
  }

  /**
   * Numerically integrate to set desired velocity for one periodic
   * Preferred over using a flag
   * @param velocity - arm velocity in deg/s
   */
  public void setPeriodicVelocity (double velocity) {
    this.setDesiredState(this.getPosition() + velocity * 0.02, velocity);
  }

  /**
   * Approximate arm as rod and intake as point mass
   * @return current moment of inertia of the arm
   */
  public double getMomentOfInertia () {
    double theta = Units.degreesToRadians(90) + this.getPosition() - ArmConstants.INTAKE_ANGLE_TO_HORIZONTAL;
    // Law of cosines
    double intakeDistToPivot = Math.sqrt(
      ArmConstants.ARM_LENGTH * ArmConstants.ARM_LENGTH +
      ArmConstants.INTAKE_LENGTH * ArmConstants.INTAKE_LENGTH -
      2 * ArmConstants.ARM_LENGTH * ArmConstants.INTAKE_LENGTH * Math.cos(theta)
    );

    return 
      1.0/3.0 * ArmConstants.ARM_MASS * ArmConstants.ARM_LENGTH * ArmConstants.ARM_LENGTH +
      ArmConstants.INTAKE_MASS * intakeDistToPivot * intakeDistToPivot;
  }

  /**
   * @return angular acceleration from arm counterbalancing
   */
  public double getAccelerationFromCounterbalance () {
    double beta = ArmConstants.ARM_ANGLE_TO_CB_ARM - this.getPosition() - Units.degreesToRadians(90);
    // Location where force is applied
    double Fx = ArmConstants.BASE_LENGTH - (ArmConstants.CB_ARM_LENGTH * Math.cos(beta));
    double Fy = ArmConstants.ARM_HEIGHT - (ArmConstants.CB_ARM_LENGTH * Math.sin(beta));

    double theta = Math.PI - Math.atan(Fy / Fx) - beta;
    
    double torque = ArmConstants.COUNTERBALANCE_FORCE * ArmConstants.CB_ARM_LENGTH * Math.sin(theta);
    
    // torque = Ia; a = torque / I
    return torque / getMomentOfInertia();
  }

  /**
   * Stop arm motor
   */
  public void stop () {
    isStopped = true;
    motor.set(0);
  }

  /** Called every 20ms */
  @Override
  public void periodic() {
    if (Constants.tuningMode) {
      /* Real time PID tuning */
      if (ArmConstants.PID_GAINS.hasChanged()) {
        ArmConstants.PID_GAINS.bindToController(controller);
      }
    }

    if (isStopped) return;

    motor.setVoltage(
      feedforward.calculate(controller.getGoal().position, controller.getGoal().velocity) +//-
      //feedforward.ka * this.getAccelerationFromCounterbalance() +
      controller.calculate(this.getPosition())
    );
  }

  @Override
  public void initSendable (SendableBuilder builder) {
    //if (Constants.telemetryMode == false) return;

    builder.setSmartDashboardType("Arm");
    builder.addDoubleProperty("Position", this::getPosition, null);
  }
}
