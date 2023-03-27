// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriveTrainConstants;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Pigeon;

public class AutoBalance extends CommandBase {
  /** Creates a new AutoBalance. */
  private final DriveTrain driveTrain;
  private final Pigeon gyro;
  private double speed = 0.0;
  private final Timer timer = new Timer();


  public AutoBalance(DriveTrain driveTrain, Pigeon gyro) {
    this.driveTrain = driveTrain;
    this.gyro = gyro;
    addRequirements(driveTrain);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    speed = 0.3;
    timer.stop();
    timer.reset();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (timer.get() != 0 && timer.get() < 0.5) {
      driveTrain.stop();
      driveTrain.lockModules();
      return;
    };
    if (Math.abs(gyro.getPitch()) < Units.degreesToRadians(8)) {
      timer.reset();
      timer.start();
      speed *= 0.4;
      return;
    }
    driveTrain.drive(0, Math.copySign(speed, gyro.getPitch()), 0);
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
