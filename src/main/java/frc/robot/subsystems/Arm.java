// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;

import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.LinearQuadraticRegulator;
import edu.wpi.first.math.estimator.KalmanFilter;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.LinearSystemLoop;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.CANIDS;
import frc.robot.Constants.CHANNELIDS;

public class Arm extends SubsystemBase {
  private final CANSparkMax motor;
  private final AbsoluteEncoder encoder;

  private final ArmFeedforward feedforward;

  private LinearSystem<N2, N1, N1> systemPlant;
  private LinearQuadraticRegulator<N2, N1, N1> systemController;
  private KalmanFilter<N2, N1, N1> systemObserver;
  private LinearSystemLoop<N2, N1, N1> systemLoop;

  private TrapezoidProfile.State referenceState;
  private TrapezoidProfile.State lastReferenceState;

  private final DigitalInput homingLimitSwitch;

  private final DoubleSupplier MOISupplier;
  private double lastMOI;

  public Arm(DoubleSupplier MOISupplier) {
    motor = new CANSparkMax(CANIDS.ARM, CANSparkMaxLowLevel.MotorType.kBrushless);
    encoder = motor.getAbsoluteEncoder(Type.kDutyCycle);

    motor.restoreFactoryDefaults();

    motor.setIdleMode(CANSparkMax.IdleMode.kBrake);
    this.setCurrentLimit();

    encoder.setPositionConversionFactor(360.0 / ArmConstants.GEAR_REDUCTION);
    encoder.setVelocityConversionFactor(360.0 / (60 * ArmConstants.GEAR_REDUCTION));
    encoder.setAverageDepth(8);

    if (Constants.competitionMode) {
      motor.burnFlash();
    }

    // Set kV and kA to 0.0 to avoid double calculating motor dynamics
    feedforward = new ArmFeedforward(ArmConstants.kS, ArmConstants.kG, 0.0, 0.0);

    this.MOISupplier = MOISupplier;
    this.constructSystem();

    homingLimitSwitch = new DigitalInput(CHANNELIDS.ARM_HOMING_LS);
  }

  public void constructSystem () {
    if (lastMOI == this.MOISupplier.getAsDouble()) return;

    systemPlant = LinearSystemId.createSingleJointedArmSystem(
      DCMotor.getNEO(1),
      this.MOISupplier.getAsDouble(),
      ArmConstants.GEAR_REDUCTION
    );

    systemController = new LinearQuadraticRegulator<>(
      systemPlant,
      VecBuilder.fill(Units.degreesToRadians(1), Units.degreesToRadians(10)), // qelms; pos and vel error tolerance
      VecBuilder.fill(12.0), // relms; control effort
      0.02
    );

    systemObserver = new KalmanFilter<>(
      Nat.N2(),
      Nat.N1(),
      systemPlant,
      VecBuilder.fill(Units.degreesToRadians(1), Units.degreesToRadians(2)), // Model accuracy; in rad and rad/s
      VecBuilder.fill(0.01), // Encoder accuracy
      0.02
    );

    systemLoop = new LinearSystemLoop<>(
      systemPlant,
      systemController,
      systemObserver,
      12.0,
      0.02
    );

    lastMOI = this.MOISupplier.getAsDouble();
  }

  public double getCurrentPosition () {
    return this.encoder.getPosition();
  }

  public double getCurrentVelocity () {
    return this.encoder.getVelocity();
  }

  public void setCurrentLimit () {
    motor.setSmartCurrentLimit(ArmConstants.CURRENT_LIMIT);
  }

  public void setHomingCurrentLimit () {
    motor.setSmartCurrentLimit(ArmConstants.HOME_CURRENT_LIMIT);
  }

  /**
   * Set only desired velocity setpoint using first order integration
   * @param velocity - desired velocity of elevator
   */
  public void setDesiredVelocity (double velocity) {
    // Side effect is higher velocities have higher soft limits, which is good
    this.setDesiredState(this.getCurrentPosition() + velocity * 0.02, velocity);
  }

  /**
   * 
   * @param position - desired position of arm
   * @param velocity - desired velocity of arm
   */
  public void setDesiredState (double position, double velocity) {
    referenceState = new TrapezoidProfile.State(position, velocity);

    // Enforce soft limits
    if (
      referenceState.position > ArmConstants.MAX_POSITION ||
      referenceState.position < ArmConstants.MIN_POSITION
    ) {
      referenceState = null;
    }
  }

  /** */
  public void resetSystem () {
    systemLoop.reset(VecBuilder.fill(
      this.getCurrentPosition(),
      this.getCurrentVelocity()
    ));

    lastReferenceState = new TrapezoidProfile.State(this.getCurrentPosition(), this.getCurrentVelocity());
  }

  public void stop () {
    motor.set(0);
    referenceState = null;
  }

  /**
   * 
   * @return
   */
  public Command getHomeCommand () {
    return new StartEndCommand(
      () -> {
        if (!homingLimitSwitch.get()) {
          this.setHomingCurrentLimit();
          motor.set(ArmConstants.HOME_SPEED);
        }
      },
      () -> {
        this.stop();
        this.setCurrentLimit();
        encoder.setZeroOffset(ArmConstants.HOMED_POSITION);
        this.resetSystem();
      },
      this)
      .until(homingLimitSwitch::get);
  }

  @Override
  public void periodic() {
    // reconstruct system to match MOI (temp)
    this.constructSystem();

    if (referenceState != null) {
      TrapezoidProfile.State newReferenceState = (new TrapezoidProfile(
        ArmConstants.PROFILE_CONSTRAINTS,
        referenceState,
        lastReferenceState
      )).calculate(0.02);

      double acceleration = (newReferenceState.velocity - lastReferenceState.velocity) / 0.02;

      systemLoop.setNextR(Units.degreesToRadians(newReferenceState.position), Units.degreesToRadians(newReferenceState.velocity));
      systemLoop.correct(VecBuilder.fill(Units.degreesToRadians(this.getCurrentPosition())));
      systemLoop.predict(0.02);
      motor.setVoltage(systemLoop.getU(0) + feedforward.calculate(newReferenceState.position, newReferenceState.velocity, acceleration));

      lastReferenceState = newReferenceState;
    }
  }
}
