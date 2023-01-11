// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;

public class DriveWithJoysticks extends CommandBase {
  DriveTrain driveTrain;
  DoubleSupplier xSupplier;
  DoubleSupplier ySupplier;
  DoubleSupplier turnSupplier;
  /** Creates a new DriveWithJoysticks. */
  public DriveWithJoysticks(
    DriveTrain driveTrain,
    DoubleSupplier xSupplier,
    DoubleSupplier ySupplier,
    DoubleSupplier turnSupplier
  ) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveTrain);
    this.driveTrain = driveTrain;
    this.xSupplier = xSupplier;
    this.ySupplier = ySupplier;
    this.turnSupplier = turnSupplier;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double xRaw = this.xSupplier.getAsDouble();
    double yRaw = this.ySupplier.getAsDouble();
    double turnRaw = this.turnSupplier.getAsDouble();
    if (Math.abs(xRaw) < 0.075) xRaw = 0;
    if (Math.abs(yRaw) < 0.075) yRaw = 0;
    if (Math.abs(turnRaw) < 0.075) turnRaw = 0;
    this.driveTrain.drive(
      xRaw, yRaw, turnRaw
    );
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
