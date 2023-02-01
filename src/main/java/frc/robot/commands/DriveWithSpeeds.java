// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;

public class DriveWithSpeeds extends CommandBase {

  private final DriveTrain driveTrain;
  private final DoubleSupplier vxSupplier;
  private final DoubleSupplier vySupplier;
  private final DoubleSupplier omegaSupplier;

  public DriveWithSpeeds(
    DriveTrain driveTrain,
    DoubleSupplier vxSupplier,
    DoubleSupplier vySupplier,
    DoubleSupplier omegaSupplier
  ) {
    this.driveTrain = driveTrain;
    this.vxSupplier = vxSupplier;
    this.vySupplier = vySupplier;
    this.omegaSupplier = omegaSupplier;
    addRequirements(driveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    driveTrain.driveOpenLoop(
      vxSupplier.getAsDouble(),
      vySupplier.getAsDouble(),
      omegaSupplier.getAsDouble()
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
