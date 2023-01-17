// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileWriter;
import java.io.IOException;
import java.util.Arrays;
import java.util.Scanner;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;

import edu.wpi.first.math.InterpolatingMatrixTreeMap;
import edu.wpi.first.math.MatBuilder;
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
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Arm extends SubsystemBase {
  // value placeholder
  private static final double ARBVALUE = 1.0;

  private final CANSparkMax motor;
  private final AbsoluteEncoder encoder;

  private final ArmFeedforward feedforward;

  private LinearSystem<N2, N1, N1> plant;
  private KalmanFilter<N2, N1, N1> observer;
  private final InterpolatingMatrixTreeMap<Double, N1, N2> KLerpTable;

  public Arm() {
    motor = new CANSparkMax(0, CANSparkMaxLowLevel.MotorType.kBrushless);
    encoder = motor.getAbsoluteEncoder(Type.kDutyCycle);
    // ... config motor and encoder

    feedforward = new ArmFeedforward(ARBVALUE, ARBVALUE, ARBVALUE, ARBVALUE);

    if (true) {
      this.writeKLerpTable();
    }

    KLerpTable = this.loadKLerpTable();
  }

  private Matrix<N1, N2> getKFromMOI (double MOI) {
    LinearSystem<N2, N1, N1> plant = LinearSystemId.createSingleJointedArmSystem(
      DCMotor.getNEO(1),
      MOI,
      ARBVALUE
    );

    LinearQuadraticRegulator<N2, N1, N1> controller = new LinearQuadraticRegulator<>(
      plant,
      VecBuilder.fill(ARBVALUE, ARBVALUE),
      VecBuilder.fill(ARBVALUE),
      0.02
    );

    return controller.getK();
  }

  private void writeKLerpTable () {
    String buffer = "";
    for (double i = ARBVALUE; i < ARBVALUE * 1000; i+=ARBVALUE) {
      Matrix<N1, N2> K = this.getKFromMOI(i);
      buffer += String.format("%f,%f,%f\n", i, K.get(0, 0), K.get(0, 1));
    }
    try {
      FileWriter writer = new FileWriter(Filesystem.getDeployDirectory() + "/arm.txt");
      writer.write(buffer);
      writer.close();
    } catch (IOException e) {
      e.printStackTrace();
    }
  }

  private InterpolatingMatrixTreeMap<Double, N1, N2> loadKLerpTable () {
    InterpolatingMatrixTreeMap<Double, N1, N2> ret = new InterpolatingMatrixTreeMap<>();
    File file = new File(Filesystem.getDeployDirectory() + "/arm.txt");
    try {
      Scanner reader = new Scanner(file);
      while (reader.hasNextLine()) {
        String line = reader.nextLine();
        String[] split = line.split(",");
        Double[] values = 
          Arrays.stream(split)
          .map(Double::valueOf)
          .toArray(Double[]::new);
        ret.put(values[0], new MatBuilder<>(Nat.N1(), Nat.N2()).fill(values[1], values[2]));
      }
      reader.close();
    } catch (FileNotFoundException e) {
      e.printStackTrace();
    }

    return ret;
  }

  /** Estimated moment of inertia */
  private double getEstimatedMOI () {
    return ARBVALUE;
  }

  private Matrix<N1, N1> calculate (Matrix<N2, N1> x, Matrix<N2, N1> nextR) {
    Matrix<N1, N1> u = KLerpTable.get(this.getEstimatedMOI()).times(nextR.minus(x));
    return u;
  }

  private void correct (Matrix<N1, N1> y, Matrix<N1, N1> u) {
    observer.correct(u, y);
  }
  
  private void predict (Matrix<N2, N1> nextR) {
    var u = StateSpaceUtil.desaturateInputVector(
      this.calculate(observer.getXhat(), nextR),
      12.0
    );
    observer.predict(u, 0.02);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
