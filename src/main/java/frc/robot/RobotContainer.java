// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.DriveFollowPath;
import frc.robot.commands.DriveWithJoysticks;
import frc.robot.subsystems.DriveTrain;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  DriveTrain driveTrain = new DriveTrain();
  Joystick leftJoystick = new Joystick(0);
  Joystick rightJoystick = new Joystick(1);

  SendableChooser<Command> autonChooser = new SendableChooser<>();
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    autonChooser.setDefaultOption("Linear Test Path", new DriveFollowPath(
      driveTrain,
      PathPlanner.loadPath("Linear Test Path", new PathConstraints(3, 2)),
      true
    ));
    autonChooser.setDefaultOption("Spin In Place", new DriveFollowPath(
      driveTrain,
      PathPlanner.loadPath("Spin In Place", new PathConstraints(3, 2)),
      true
    ));
    autonChooser.setDefaultOption("Complex", new DriveFollowPath(
      driveTrain,
      PathPlanner.loadPath("Complex", new PathConstraints(3, 2)),
      true
    ));

    SmartDashboard.putData(autonChooser);
    // Configure the trigger bindings
    configureBindings();
    driveTrain.setDefaultCommand(new DriveWithJoysticks(driveTrain, leftJoystick::getX, leftJoystick::getY, rightJoystick::getX));
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {}

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autonChooser.getSelected();
  }
}
