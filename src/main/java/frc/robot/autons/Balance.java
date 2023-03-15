// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autons;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Pigeon;

public class Balance extends CommandBase {
  /** Creates a new Balance. */
  private final DriveTrain driveTrain;
  private final Pigeon gyro;
  private final PIDController controller = new PIDController(0.3, 0, 0);

  public Balance(DriveTrain driveTrain, Pigeon gyro) {
    this.driveTrain = driveTrain;
    this.gyro = gyro;
    addRequirements(driveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double speed = controller.calculate(gyro.getPitch(), 0);
    if (Math.abs(gyro.getPitch()) > 0.05) speed += Math.copySign(0.5, speed);
    if (Math.abs(gyro.getPitch()) < 0.005) speed = 0;
    driveTrain.drive(0, speed, 0);
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
