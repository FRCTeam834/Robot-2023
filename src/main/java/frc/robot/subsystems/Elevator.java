// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.LinearQuadraticRegulator;
import edu.wpi.first.math.estimator.KalmanFilter;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.LinearSystemLoop;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.CANIDS;
import frc.robot.Constants.CHANNELIDS;
import frc.robot.Constants.ElevatorConstants;

public class Elevator extends SubsystemBase {
  private final CANSparkMax masterMotor;
  private final CANSparkMax followerMotor;

  private final RelativeEncoder encoder;
  
  private final ElevatorFeedforward feedforward;
  private final LinearSystem<N2, N1, N1> systemPlant;
  private final LinearQuadraticRegulator<N2, N1, N1> systemController;
  private final KalmanFilter<N2, N1, N1> systemObserver;
  private final LinearSystemLoop<N2, N1, N1> systemLoop;

  private TrapezoidProfile.State referenceState;
  private TrapezoidProfile.State lastReferenceState;

  private final DigitalInput homingLimitSwitch;
  
  public Elevator() {
    masterMotor = new CANSparkMax(CANIDS.ELEVATOR_MASTER, CANSparkMaxLowLevel.MotorType.kBrushless);
    followerMotor = new CANSparkMax(CANIDS.ELEVATOR_FOLLOWER, CANSparkMaxLowLevel.MotorType.kBrushless);

    masterMotor.restoreFactoryDefaults();
    followerMotor.restoreFactoryDefaults();

    masterMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
    followerMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);

    this.setCurrentLimit();

    followerMotor.follow(masterMotor, false);

    encoder = masterMotor.getEncoder();
    encoder.setPositionConversionFactor(2 * Math.PI * ElevatorConstants.DRUM_RADIUS / ElevatorConstants.GEAR_REDUCTION);
    encoder.setVelocityConversionFactor(2 * Math.PI * ElevatorConstants.DRUM_RADIUS / (60 * ElevatorConstants.GEAR_REDUCTION));

    if (Constants.competitionMode) {
      masterMotor.burnFlash();
      followerMotor.burnFlash();
    }

    // Set kV and kA to 0.0 to avoid double calculating motor dynamics
    feedforward = new ElevatorFeedforward(ElevatorConstants.kS, ElevatorConstants.kG, 0.0, 0.0);

    systemPlant = LinearSystemId.identifyPositionSystem(ElevatorConstants.kV, ElevatorConstants.kA);

    systemController = new LinearQuadraticRegulator<>(
      systemPlant,
      VecBuilder.fill(0.05, 0.1), // qelms; pos and vel error tolerance
      VecBuilder.fill(12.0), // relms; control effort
      0.02
    );

    systemObserver = new KalmanFilter<>(
      Nat.N2(),
      Nat.N1(),
      systemPlant,
      VecBuilder.fill(0.1, 0.2), // Model accuracy; in meters and meters/s
      VecBuilder.fill(0.001), // Encoder accuracy
      0.02
    );

    systemLoop = new LinearSystemLoop<>(
      systemPlant,
      systemController,
      systemObserver,
      12.0,
      0.02
    );

    homingLimitSwitch = new DigitalInput(CHANNELIDS.ELEVATOR_HOMING_LS);
  }

  public double getCurrentPosition () {
    return this.encoder.getPosition();
  }

  public double getCurrentVelocity () {
    return this.encoder.getVelocity();
  }
  
  /** Get moment of inertia of elevator on pivoting axis */
  public double getMOI () {
    // temp
    return 0.0;
  }

  public void setCurrentLimit () {
    masterMotor.setSmartCurrentLimit(ElevatorConstants.CURRENT_LIMIT);
    followerMotor.setSmartCurrentLimit(ElevatorConstants.CURRENT_LIMIT);
  }

  public void setHomingCurrentLimit () {
    masterMotor.setSmartCurrentLimit(ElevatorConstants.HOME_CURRENT_LIMIT);
    followerMotor.setSmartCurrentLimit(ElevatorConstants.HOME_CURRENT_LIMIT);
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
   * @param position - desired position of elevator
   * @param velocity - desired velocity of elevator
   */
  public void setDesiredState (double position, double velocity) {
    referenceState = new TrapezoidProfile.State(position, velocity);

    // Enforce soft limits
    if (
      referenceState.position > ElevatorConstants.MAX_POSITION ||
      referenceState.position < ElevatorConstants.MIN_POSITION
    ) {
      referenceState = null;
    }
  }

  /** Reset system model to encoder readings */
  public void resetSystem () {
    systemLoop.reset(VecBuilder.fill(
      this.getCurrentPosition(),
      this.getCurrentVelocity()
    ));

    lastReferenceState = new TrapezoidProfile.State(this.getCurrentPosition(), this.getCurrentVelocity());
  }

  /** Stops the elevator */
  public void stop () {
    masterMotor.set(0);
    referenceState = null;
  }

  /**
   * Get command for homing elevator
   * @return home command
   */
  public Command getHomeCommand () {
    return new StartEndCommand(
      () -> {
        if (!homingLimitSwitch.get()) {
          this.setHomingCurrentLimit();
          masterMotor.set(ElevatorConstants.HOME_SPEED);
        }
      },
      () -> {
        this.stop();
        this.setCurrentLimit();
        encoder.setPosition(0);
        this.resetSystem();
      },
      this)
      .until(homingLimitSwitch::get);
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    if (!Constants.telemetryMode) return;

    builder.setSmartDashboardType("Elevator");
    builder.addDoubleProperty("Position", this::getCurrentPosition, null);
  }

  @Override
  public void periodic() {
    if (referenceState != null) {
      TrapezoidProfile.State newReferenceState = (new TrapezoidProfile(
        ElevatorConstants.PROFILE_CONSTRAINTS,
        referenceState,
        lastReferenceState
      )).calculate(0.02);

      double acceleration = (newReferenceState.velocity - lastReferenceState.velocity) / 0.02;

      systemLoop.setNextR(newReferenceState.position, newReferenceState.velocity);
      systemLoop.correct(VecBuilder.fill(this.getCurrentPosition()));
      systemLoop.predict(0.02);
      masterMotor.setVoltage(systemLoop.getU(0) + feedforward.calculate(newReferenceState.velocity, acceleration));

      lastReferenceState = newReferenceState;
    }
  }
}
