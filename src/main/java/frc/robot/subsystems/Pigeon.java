// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.time.OffsetDateTime;

import com.ctre.phoenix.sensors.Pigeon2;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.PigeonConstants;

public class Pigeon extends SubsystemBase {
  private final Pigeon2 pigeon;
  private double pitchVelocity;
  private LinearFilter pitchVelocityFilter = LinearFilter.backwardFiniteDifference(1, 3, 0.02);
  public double offset = 0.0;

  public Pigeon() {
    pigeon = new Pigeon2(PigeonConstants.CANID);
    pigeon.setYaw(0);

    SmartDashboard.putData(this);
  }

  public double getYaw () {
    return Units.degreesToRadians(pigeon.getYaw());
  }

  public double getPitch () {
    return Units.degreesToRadians(pigeon.getPitch());
  }

  public double getPitchVelocity () {
    return pitchVelocity;
  }

  public Rotation2d getYawAsRotation2d () {
    return Rotation2d.fromRadians(this.getYaw());
  }

  public void resetYaw (double angle) {
    pigeon.setYaw(angle);
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    if (Constants.telemetryMode == false) return;

    builder.setSmartDashboardType("Pigeon");
    builder.addDoubleProperty("Yaw", this::getYaw, null);
    builder.addDoubleProperty("Pitch", this::getPitch, null);
    builder.addDoubleProperty("Pitch Velocity", this::getPitchVelocity, null);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    pitchVelocity = pitchVelocityFilter.calculate(getPitch());
  }
}
