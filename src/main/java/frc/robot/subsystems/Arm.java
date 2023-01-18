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
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.CANIDS;
import frc.robot.Constants.CHANNELIDS;

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
  private Matrix<N1, N1> lastU;

  /* Desired states */
  private TrapezoidProfile.State desiredState;
  private TrapezoidProfile.State lastReferenceState;
  /* Purely velocity setpoint, use numerical integration (eg driver control) */
  private boolean velocitySetpoint = false;

  /* Limit switch for homing */
  private final DigitalInput homingLimitSwitch;

  /** Constructor */
  public Arm() {
    motor = new CANSparkMax(CANIDS.ARM, CANSparkMaxLowLevel.MotorType.kBrushless);
    encoder = motor.getAbsoluteEncoder(Type.kDutyCycle);

    feedforward = new ArmFeedforward(
      ArmConstants.kS,
      ArmConstants.kG,
      ArmConstants.kV,
      ArmConstants.kA
    );

    plant = LinearSystemId.identifyPositionSystem(
      ArmConstants.kV,
      ArmConstants.kA
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

    homingLimitSwitch = new DigitalInput(CHANNELIDS.ARM_HOMING_LS);

    /* Configurations */

    motor.restoreFactoryDefaults();
    motor.setIdleMode(CANSparkMax.IdleMode.kBrake);
    motor.setInverted(false);
    motor.enableVoltageCompensation(12.0);
    motor.setSmartCurrentLimit(ArmConstants.CURENT_LIMIT);
    /* Do not mess with these unless you know what you are doing */
    motor.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 255);
    motor.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 255);
    motor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 255);
    motor.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 255);
    motor.setPeriodicFramePeriod(PeriodicFrame.kStatus4, 255);
    motor.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 20);
    motor.setPeriodicFramePeriod(PeriodicFrame.kStatus6, 255);

    /* Revolutions -> Degrees */
    encoder.setPositionConversionFactor(360.0 / ArmConstants.GEAR_REDUCTION);
  }

  /**
   * 
   * @return current arm angle (position)
   */
  public double getPosition () {
    return this.encoder.getPosition();
  }

  /**
   * 
   * @return current arm angle in radians
   */
  public double getPositionRadians () {
    return Units.degreesToRadians(this.getPosition());
  }

  /**
   * 
   * Set desired state of arm
   * @param position - arm position in deg
   * @param velocity - arm velocity in deg/s
   */
  public void setDesiredState (double position, double velocity) {
    velocitySetpoint = false;
    desiredState = new TrapezoidProfile.State(position, velocity);
  }

  /**
   * 
   * Purely velocity setpoint for driver control
   * @param velocity - arm velocity in deg/s
   */
  public void setDesiredVelocity (double velocity) {
    velocitySetpoint = true;
    desiredState = new TrapezoidProfile.State(this.getPosition(), velocity);
  }

  /**
   * 
   * @param nextPosition
   * @param nextVelocity
   * @return
   */
  public double stateSpaceCalculate () {
    if (desiredState == null) return 0.0;

    var r = VecBuilder.fill(desiredState.position, desiredState.velocity);
    
    observer.correct(
      lastU,
      VecBuilder.fill(this.getPositionRadians())
    );

    /* Control law u = K(r-x)*/
    var u = StateSpaceUtil.desaturateInputVector(
      K.times(r.minus(observer.getXhat())),
      12.0
    );

    observer.predict(u, 0.02);

    lastU = u;
    return u.get(0, 0);
  }

  /**
   * Hold arm at current angle
   */
  public void hold () {
    // Don't clear desiredState
    motor.setVoltage(feedforward.calculate(this.getPosition(), 0.0));
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
      if (velocitySetpoint) {
        /* Numerically integrate next desired position to achieve velocity setpoint */
        desiredState.position = this.getPosition() + desiredState.velocity * 0.02;
      }
      /* New reference state from desired state */
      TrapezoidProfile.State newReferenceState = (new TrapezoidProfile(
        ArmConstants.PROFILE_CONSTRAINTS,
        desiredState,
        lastReferenceState
      )).calculate(0.02);

      double position = newReferenceState.position;
      double velocity = newReferenceState.velocity;
      double acceleration = (newReferenceState.velocity - lastReferenceState.velocity) / 0.02;

      if (position < ArmConstants.MIN_POSITION || position > ArmConstants.MAX_POSITION) {
        /* Reference position over softlimit */
        this.hold();
      } else {
        motor.setVoltage(
          this.stateSpaceCalculate() +
          feedforward.calculate(position, velocity, acceleration)
        );
      }

      lastReferenceState = newReferenceState;
    }
  }

  /**
   * 
   * @return homing command for arm
   */
  public Command getHomeCommand () {
    return new StartEndCommand(
      /* Called on initializing command */
      () -> {
        /* No need to home if already homed */
        if (!homingLimitSwitch.get()) {
          motor.setSmartCurrentLimit(ArmConstants.HOME_CURRENT_LIMIT);
          motor.set(ArmConstants.HOME_SPEED);
        }
      },
      /* Called when command stops */
      () -> {
        motor.setSmartCurrentLimit(ArmConstants.CURENT_LIMIT);
        motor.set(0);
        encoder.setZeroOffset(ArmConstants.HOME_POSITION - encoder.getPosition());
      },
      /* Command requirements */
      this
    ).until(homingLimitSwitch::get);
  }

  /** Telemetry */
  @Override
  public void initSendable (SendableBuilder builder) {
    
  }
}
