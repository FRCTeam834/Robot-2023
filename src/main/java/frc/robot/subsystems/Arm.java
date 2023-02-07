// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.StateSpaceUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.LinearQuadraticRegulator;
import edu.wpi.first.math.estimator.KalmanFilter;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ArmConstants;

public class Arm extends SubsystemBase {
  private final CANSparkMax motor;
  private final AbsoluteEncoder encoder;

  /* Feedforward control */
  private final ArmFeedforward feedforward;

  /* State space control */
  private final LinearSystem<N2, N1, N1> plant;
  private final LinearQuadraticRegulator<N2, N1, N1> controller;
  private final KalmanFilter<N2, N1, N1> observer;
  private final Matrix<N1, N2> K;
  private double lastTime;

  /* Desired states */
  private TrapezoidProfile.State desiredState;
  private TrapezoidProfile.State lastReferenceState;

  /** Constructor */
  public Arm() {
    motor = new CANSparkMax(ArmConstants.CANID, CANSparkMaxLowLevel.MotorType.kBrushless);
    encoder = motor.getAbsoluteEncoder(Type.kDutyCycle);

    // Duty cycle encoder position packet (default 200ms - bad!)
    motor.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 20);
    motor.setPeriodicFramePeriod(PeriodicFrame.kStatus6, 20);
    motor.setPeriodicFramePeriod(PeriodicFrame.kStatus4, 65535);

    feedforward = ArmConstants.FEEDFORWARD;

    plant = LinearSystemId.identifyPositionSystem(
      ArmConstants.FEEDFORWARD.kv,
      ArmConstants.FEEDFORWARD.ka
    );

    controller = new LinearQuadraticRegulator<>(
      plant,
      VecBuilder.fill(Units.degreesToRadians(1), Units.degreesToRadians(2)), // position and velocity error tolerance
      VecBuilder.fill(12), // control effort
      0.02
    );

    observer = new KalmanFilter<>(
      Nat.N2(),
      Nat.N1(),
      plant,
      VecBuilder.fill(Units.degreesToRadians(1), Units.degreesToRadians(2)), // model accuracy in rad and rad/s
      VecBuilder.fill(0.01), // how much we trust encoder readings
      0.02
    );

    K = controller.getK();

    /* Configurations */

    motor.restoreFactoryDefaults();
    motor.setIdleMode(CANSparkMax.IdleMode.kBrake);
    motor.setInverted(false);
    motor.enableVoltageCompensation(12.0);
    motor.setSmartCurrentLimit(ArmConstants.CURRENT_LIMIT);

    /* Revs -> radians */
    encoder.setPositionConversionFactor(2 * Math.PI / ArmConstants.GEAR_REDUCTION);

    if (Constants.competitionMode) {
      motor.burnFlash();
    }
  }

  /**
   * @return current arm angle
   */
  public double getPosition () {
    return this.encoder.getPosition();
  }

  /**
   * @return current arm angle in radians
   */
  public double getPositionRadians () {
    return Units.degreesToRadians(this.getPosition());
  }

  /**
   * Set desired state of arm
   * @param position - arm position in deg
   * @param velocity - arm velocity in deg/s
   */
  public void setDesiredState (double position, double velocity) {
    desiredState = new TrapezoidProfile.State(position, velocity);
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
   * @return calculated motor voltage
   */
  public double stateSpaceCalculate () {
    if (desiredState == null) return 0.0;
    /* Reference state */
    var r = VecBuilder.fill(desiredState.position, desiredState.velocity);
    
    /* Control law u = K(r-x)*/
    var u = StateSpaceUtil.desaturateInputVector(
      K.times(r.minus(observer.getXhat())),
      12.0
    );

    double currentTime = Timer.getFPGATimestamp();
    observer.predict(u, currentTime - lastTime);
    lastTime = currentTime;

    observer.correct(
      u,
      VecBuilder.fill(this.getPositionRadians())
    );

    return u.get(0, 0);
  }

  /**
   * @return angular acceleration from arm counterbalancing
   */
  public double getAccelerationFromCounterbalance () {
    // torque = I * alpha = F * r * sin phi
    return 0.0; // Placeholder
  }

  /**
   * Keep arm static
   */
  public void hold () {
    motor.setVoltage(
      feedforward.calculate(this.getPositionRadians(), 0) -
      feedforward.ka * this.getAccelerationFromCounterbalance()
    );
  }

  /**
   * Stop arm motor
   */
  public void stop () {
    desiredState = null;
    motor.set(0);
  }

  /** Called every 20ms */
  @Override
  public void periodic() {
    if (desiredState != null) {
      /* New reference state from desired state */
      TrapezoidProfile.State newReferenceState = (new TrapezoidProfile(
        ArmConstants.PROFILE_CONSTRAINTS,
        desiredState,
        lastReferenceState
      )).calculate(0.02);

      double position = newReferenceState.position;
      double velocity = newReferenceState.velocity;
      double acceleration = (newReferenceState.velocity - lastReferenceState.velocity) / 0.02;

      if (
        (position <= ArmConstants.MIN_POSITION && velocity < 0) ||
        (position >= ArmConstants.MAX_POSITION && velocity > 0)
      ) {
        /* Arm is trying to exceed softlimits */
        this.hold();
      } else {
        motor.setVoltage(
          this.stateSpaceCalculate() +
          feedforward.calculate(position, velocity, acceleration) -
          feedforward.ka * this.getAccelerationFromCounterbalance()
        );
      }

      lastReferenceState = newReferenceState;
    }
  }

  @Override
  public void initSendable (SendableBuilder builder) {
    if (!Constants.telemetryMode) return;
    
    builder.setSmartDashboardType("Arm");
    builder.addDoubleProperty("Position", this::getPosition, null);
  }
}
