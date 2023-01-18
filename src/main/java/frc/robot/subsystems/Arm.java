// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;

import edu.wpi.first.math.InterpolatingMatrixTreeMap;
import edu.wpi.first.math.controller.LinearQuadraticRegulator;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Arm extends SubsystemBase {
  
  private final CANSparkMax motor;
  private final AbsoluteEncoder encoder;

  private ArmFeedforward feedforward;

  private final InterpolatingMatrixTreeMap KTable;

  public Arm() {
    motor = new CANSparkMax(CANIDS.ARM, CANSparkMaxLowLevel.MotorType.kBrushless);
  }

  /** Calculate feedforward kA term from MOI */
  public double solveFeedforwardkA (double MOI) {
    return MOI * 12.0 / DCMotor.getNEO(1).stallTorqueNewtonMeters;
  }

  /** Calculate LQR K matrix from MOI */
  public Matrix<N1, N2> solveKFromMOI (double MOI) {
    LinearSystem<N2, N1, N1> plant = LinearSystemId.createSingleJointedArmSystem(
      DCMotor.getNEO(1),
      MOI,
      ArmConstants.GEAR_REDUCTION
    );

    LinearQuadraticRegulator<N2, N1, N1> controller = new LinearQuadraticRegulator<>(
      plant,
      VecBuilder.fill(),
      VecBuilder.fill(),
      0.02
    );

    return controller.getK();
  }

  /**
   * !Shouldn't be necessary to cache in file
   */
  public InterpolatingMatrixTreeMap<Double, N1, N2> generateKTable () {

  }

  

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
