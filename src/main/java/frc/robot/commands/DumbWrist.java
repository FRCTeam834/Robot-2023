// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Wrist;

public class DumbWrist extends CommandBase {
  private final Wrist wrist;
  private final DoubleSupplier supplier;
  private boolean active = false;

  public DumbWrist(Wrist wrist, DoubleSupplier supplier) {
    this.wrist = wrist;
    this.supplier = supplier;
    addRequirements(wrist);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    active = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (active == false && supplier.getAsDouble() != 0.0) {
      active = true;
      //wrist.isStopped = true;
    }
    //System.out.println(supplier.getAsDouble() * 6);
    if (active == true) {
      this.wrist.setVoltage(supplier.getAsDouble());
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // this.arm.setVoltage(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
