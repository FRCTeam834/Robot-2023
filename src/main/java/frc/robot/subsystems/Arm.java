// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Superstructure;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.GamePieceType;
import frc.robot.Constants.ArmConstants.ArmPositionPresets;

public class Arm extends SubsystemBase {
  private final CANSparkMax motor;
  public final RelativeEncoder encoder;

  private final ArmFeedforward feedforward;
  public final ProfiledPIDController controller = new ProfiledPIDController(0.0, 0.0, 0.0, ArmConstants.PROFILE_CONSTRAINTS);

  public boolean isStopped = true;
  private boolean resetToggled = false;

  /** Constructor */
  public Arm() {
    motor = new CANSparkMax(ArmConstants.CANID, CANSparkMaxLowLevel.MotorType.kBrushless);
    encoder = motor.getEncoder();

    /* Set PID Gains */
    ArmConstants.PID_GAINS.bindToController(controller);
    controller.setTolerance(ArmConstants.SETPOINT_TOLERANCE);

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

    //encoder.setPosition(ArmConstants.STARTING_POSITION);

    motor.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 10);
    motor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 10);

    if (Constants.competitionMode) {
      motor.burnFlash();
    }

    SmartDashboard.putData(this);
    SmartDashboard.putData(controller);
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

  //
  //
  //

  //
  //
  //
  // THIS NEEDS TO BE FIXED TO ATGOAL
  public boolean atSetpoint () {
    return controller.atSetpoint();
  }

  /**
   * Set desired state of arm
   * @param position - arm position in rad
   * @param velocity - arm velocity in rad/s
   */
  public void setDesiredState (double position, double velocity) {
    isStopped = false;
    resetToggled = true;
    controller.reset(this.getPosition(), this.getVelocity());
    controller.setGoal(new TrapezoidProfile.State(position, velocity));
  }

  /**
   * Numerically integrate to set desired velocity for one periodic
   * Preferred over using a flag
   * @param velocity - arm velocity in deg/s
   */
  /*public void setPeriodicVelocity (double velocity) {
    this.setDesiredState(this.getPosition() + velocity * 0.02, velocity);
  }*/

  /**
   * Approximate arm as rod and intake as point mass
   * @return current moment of inertia of the arm
   */
  /*public double getMomentOfInertia () {
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
  }*/

  /**
   * @return angular acceleration from arm counterbalancing
   */
  //public double getAccelerationFromCounterbalance () {
    /*double beta = ArmConstants.ARM_ANGLE_TO_CB_ARM - this.getPosition() - Units.degreesToRadians(90);
    // Location where force is applied
    double Fx = ArmConstants.BASE_LENGTH - (ArmConstants.CB_ARM_LENGTH * Math.cos(beta));
    double Fy = ArmConstants.ARM_HEIGHT - (ArmConstants.CB_ARM_LENGTH * Math.sin(beta));

    double theta = Math.PI - Math.atan(Fy / Fx) - beta;
    
    double torque = ArmConstants.COUNTERBALANCE_FORCE * ArmConstants.CB_ARM_LENGTH * Math.sin(theta);
    // torque = Ia; a = torque / I
    return torque / getMomentOfInertia();*/
    //return ArmConstants.CB_LERP_TABLE.get(this.getPosition());
  //}

  public void setVoltage (double voltage) {
    motor.setVoltage(voltage);
  }

  /**
   * Stop arm motor
   */
  public void stop () {
    isStopped = true;
    motor.set(0);
  }

  @Override
  public void periodic () {
    if (Constants.tuningMode) {
      /* Real time PID tuning */
      if (ArmConstants.PID_GAINS.hasChanged()) {
        ArmConstants.PID_GAINS.bindToController(controller);
      }
    }
  }

  /** Called every 10ms */
  public void periodic10() {
    if (isStopped) return;

    if (this.getPosition() > ArmConstants.MAX_POSITION || this.getPosition() < ArmConstants.MIN_POSITION) {
      motor.set(0);
      //if (controller.getGoal().position == ArmPositionPresets.ESCAPE.position) {
      //  setDesiredState(ArmPositionPresets.STOW.position, 0);
      //}
    } else {
      //if (resetToggled == false && Math.abs(this.getPosition() - controller.getGoal().position) < 0.05) {
      //  resetToggled = true;
      //  controller.reset(getPosition(), getVelocity());
      //}
      double voltage = feedforward.calculate(controller.getGoal().position, controller.getGoal().velocity) + controller.calculate(this.getPosition());

      if (
        controller.getGoal().position == ArmPositionPresets.GCUBE.position &&
        Superstructure.desiredGamePiece == GamePieceType.CONE
        ) {
        controller.setGoal(ArmPositionPresets.GCONE.position);
      }
      
      if (
        controller.getGoal().position == ArmPositionPresets.GCONE.position &&
        Superstructure.desiredGamePiece == GamePieceType.CUBE
        ) {
        controller.setGoal(ArmPositionPresets.GCUBE.position);
      }

      if (controller.getGoal().position == ArmPositionPresets.ESCAPE.position) {
        voltage -= 8;
      }

      motor.setVoltage(voltage);

      /*System.out.println(feedforward.calculate(controller.getGoal().position, controller.getGoal().velocity) +//-
      feedforward.ka * this.getAccelerationFromCounterbalance() +
      controller.calculate(this.getPosition()));*/
    }
    //this.getAccelerationFromCounterbalance();
  }

  public double getPositionSetpoint () {
    return controller.getGoal().position;
  }

  @Override
  public void initSendable (SendableBuilder builder) {
    if (Constants.telemetryMode == false) return;

    builder.setSmartDashboardType("Arm");
    builder.addDoubleProperty("Position", () -> Units.radiansToDegrees(this.getPosition()), null);
    builder.addDoubleProperty("Setpoint", () -> Units.radiansToDegrees(this.getPositionSetpoint()), null);
    builder.addDoubleProperty("Error", () -> Units.radiansToDegrees(this.getPosition()) - Units.radiansToDegrees(this.getPositionSetpoint()), null);
    builder.addDoubleProperty("Applied Output", motor::getAppliedOutput, null);
  }
}
