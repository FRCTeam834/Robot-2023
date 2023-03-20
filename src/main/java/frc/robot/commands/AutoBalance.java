// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Pigeon;

public class AutoBalance extends CommandBase {
  /** Creates a new AutoBalance. */
  private final DriveTrain driveTrain;
  private final Pigeon gyro;
  private final PIDController controller = new PIDController(2.5, 0, 0);
  private final PIDController vcontroller = new PIDController(1, 0, 0);
  private boolean flipped = false;
  private double flipDirection = 0;
  private Timer flipTimer = new Timer();
  private Timer bufferTimer = new Timer();


  public AutoBalance(DriveTrain driveTrain, Pigeon gyro) {
    this.driveTrain = driveTrain;
    this.gyro = gyro;
    addRequirements(driveTrain);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    flipped = false;
    flipTimer.stop();
    flipTimer.reset();
    bufferTimer.reset();
    bufferTimer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (flipTimer.get() > 0.3) {
      driveTrain.stop();
      driveTrain.lockModules();
      return;
    }
    if (flipped) {
      driveTrain.drive(0, flipDirection * 0.4, 0);
    } else {
      double speed = 0.0;
      speed = controller.calculate(gyro.getPitch(), 0);
      speed = Math.max(Math.min(speed, 0.5), -0.5);
      driveTrain.drive(0, speed, 0);
  
      if (Math.abs(gyro.getPitchVelocity()) > 3 && bufferTimer.get() > 0.5) {
        flipped = true;
        flipDirection = Math.signum(-speed);
        flipTimer.start();
      }
    }
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
