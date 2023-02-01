// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.sensors.Pigeon2;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.PigeonConstants;

public class Pigeon extends SubsystemBase {
  private final Pigeon2 pigeon;

  public Pigeon() {
    pigeon = new Pigeon2(PigeonConstants.CANID);
  }

  public double getYaw () {
    return Units.degreesToRadians(pigeon.getYaw());
  }

  public Rotation2d getYawAsRotation2d () {
    return Rotation2d.fromRadians(this.getYaw());
  }

  public void resetYaw (double angle) {
    pigeon.setYaw(angle);
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    if (!Constants.telemetryMode) return;

    builder.setSmartDashboardType("Pigeon");
    builder.addDoubleProperty("Yaw", this::getYaw, null);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
