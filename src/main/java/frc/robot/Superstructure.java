// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.autons.LinearPath;
import frc.robot.commands.DriveWithSpeeds;
// import frc.robot.subsystems.Arm;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.PidControllerArm;
import frc.robot.subsystems.Pigeon;
import frc.robot.subsystems.PoseEstimator;
import frc.robot.subsystems.Vision;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class Superstructure {
  /* Initialize subsystems */
  OI oi = new OI();
  // Pigeon pigeon = new Pigeon();
  // DriveTrain driveTrain = new DriveTrain(pigeon);
  // Arm arm = new Arm();

  PidControllerArm arm = new PidControllerArm();
  Joystick leftJoystick = new Joystick(0);

  JoystickButton button0 = new JoystickButton(leftJoystick, 1);


  // Intake intake = new Intake();
  // Vision vision = new Vision();



  // PoseEstimator poseEstimator = new PoseEstimator(
  //   driveTrain.getKinematics(),
  //   driveTrain,
  //   pigeon,
  //   vision
  // );

  // SendableChooser<Command> autonChooser = new SendableChooser<>();

  public Superstructure() {
  //   autonChooser.setDefaultOption("Do nothing", new InstantCommand());
  //   autonChooser.addOption("Linear test path", new LinearPath(driveTrain, poseEstimator));

  //   driveTrain.setDefaultCommand(new DriveWithSpeeds(
  //     driveTrain,
  //     OI::getRightJoystickX,
  //     OI::getRightJoystickY,
  //     OI::getLeftJoystickX
  //   ));
    // Configure the trigger bindings
    configureBindings();
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
  private void configureBindings() {
    new JoystickButton(leftJoystick, 1).onTrue(new InstantCommand(() -> {
      arm.setGoal(Units.degreesToRadians(30));
    }));

    new JoystickButton(leftJoystick, 2).onTrue(new InstantCommand(() -> {
      arm.stop();
    }));

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * 
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return null;
    // return autonChooser.getSelected();
  }
}
