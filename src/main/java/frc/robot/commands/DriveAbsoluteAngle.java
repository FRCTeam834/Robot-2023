// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.ProfiledPIDCommand;
import frc.robot.Constants.DriveTrainConstants;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Pigeon;

public class DriveAbsoluteAngle extends CommandBase {

  private final DriveTrain driveTrain;
  private final Pigeon gyro;
  private final DoubleSupplier vxSupplier;
  private final DoubleSupplier vySupplier;
  private final DoubleSupplier angleXSupplier;
  private final DoubleSupplier angleYSupplier;

  private final ProfiledPIDController angleController = new ProfiledPIDController(0, 0, 0, DriveTrainConstants.AUTON_STEER_CONSTRAINTS);

  public DriveAbsoluteAngle(
    DriveTrain driveTrain,
    Pigeon gyro,
    DoubleSupplier vxSupplier,
    DoubleSupplier vySupplier,
    DoubleSupplier angleXSupplier,
    DoubleSupplier angleYSupplier
  ) {
    this.driveTrain = driveTrain;
    this.gyro = gyro;
    this.vxSupplier = vxSupplier;
    this.vySupplier = vySupplier;
    this.angleXSupplier = angleXSupplier;
    this.angleYSupplier = angleYSupplier;

    angleController.enableContinuousInput(0, Math.PI * 2);

    addRequirements(driveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double omega = 0;
    if (Math.hypot(angleXSupplier.getAsDouble(), angleYSupplier.getAsDouble()) > 0.1) {
      omega = Math.atan2(angleYSupplier.getAsDouble(), angleXSupplier.getAsDouble());
      omega = angleController.calculate(gyro.getYaw(), omega);
    }

    driveTrain.drive(
      vxSupplier.getAsDouble(),
      vySupplier.getAsDouble(),
      omega
    );
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveTrain.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
