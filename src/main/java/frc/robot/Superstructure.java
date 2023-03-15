// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.HashMap;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ArmConstants.ArmPositionPresets;
import frc.robot.Constants.DriveTrainConstants.OnTheFlyConstants;
import frc.robot.autons.Balance;
import frc.robot.autons.LinearPath;
import frc.robot.autons.CableOnePlusOne;
import frc.robot.autons.OnePlusZero;
import frc.robot.commands.ArmToPreset;
import frc.robot.commands.DriveAbsoluteAngle;
import frc.robot.commands.DriveIntoGriddy;
import frc.robot.commands.DriveToWaypoint;
import frc.robot.commands.DriveWithSpeeds;
import frc.robot.commands.DumbArm;
import frc.robot.commands.IntakeCone;
import frc.robot.commands.IntakeCube;
import frc.robot.commands.Outtake;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LEDs;
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
  Pigeon pigeon = new Pigeon();
  DriveTrain driveTrain = new DriveTrain(pigeon);
  Arm arm = new Arm();
  Intake intake = new Intake();
  Vision vision = new Vision();
  PoseEstimator poseEstimator = new PoseEstimator(
    driveTrain.getKinematics(),
    driveTrain,
    pigeon,
    vision
  );

  public static LEDs leds = new LEDs();

  SendableChooser<Command> autonChooser = new SendableChooser<>();

  public static final HashMap<String, Command> eventMap = new HashMap<>();

  public Superstructure() {
    eventMap.put("ESCAPE", new ParallelCommandGroup(
      new ArmToPreset(arm, ArmPositionPresets.ESCAPE),
      new RepeatCommand(new InstantCommand()).withTimeout(0.2)
    ));
    eventMap.put("STOW", new ParallelCommandGroup(
      new ArmToPreset(arm, ArmPositionPresets.STOW),
      new RepeatCommand(new InstantCommand()).until(() -> arm.getPosition() < ArmPositionPresets.L1.position)
    ));
    eventMap.put("L1", new ParallelCommandGroup(
      new ArmToPreset(arm, ArmPositionPresets.L1),
      new RepeatCommand(new InstantCommand()).until(() -> arm.getPosition() < ArmPositionPresets.L2.position)
    ));
    eventMap.put("L3", new ParallelCommandGroup(
      new ArmToPreset(arm, ArmPositionPresets.L3),
      new RepeatCommand(new InstantCommand()).until(() -> arm.getPosition() > ArmPositionPresets.L2.position)
    ));
    eventMap.put("CONE", new IntakeCone(intake));
    eventMap.put("CUBE", new IntakeCube(intake));
    eventMap.put("OUT", new Outtake(intake));
  
    autonChooser.setDefaultOption("Do nothing", new InstantCommand());
    autonChooser.addOption("Balance", new Balance(driveTrain, pigeon));
    autonChooser.addOption("1 + 0", new OnePlusZero(driveTrain, arm, intake, poseEstimator));
    autonChooser.addOption("Cable 1 + 1", new CableOnePlusOne(driveTrain, arm, intake, poseEstimator));
    SmartDashboard.putData(autonChooser);

    driveTrain.setDefaultCommand(new DriveWithSpeeds(
      driveTrain,
      OI::getRightJoystickX,
      OI::getRightJoystickY,
      OI::getLeftJoystickX
    ));

    arm.setDefaultCommand(new DumbArm(
      arm,
      OI::getXboxLeftJoystickY
    ));

    //arm.setDefaultCommand(new DumbArm(arm, OI::getRightJoystickY));
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
    new JoystickButton(new XboxController(2), 4).onTrue(new IntakeCone(intake));
    new JoystickButton(new XboxController(2), 3).onTrue(new IntakeCube(intake));
    new JoystickButton(new XboxController(2), 2).onTrue(new Outtake(intake));
    
    // Zero the arm manually when both triggers are pressed
    new JoystickButton(new XboxController(2), 5)
      .and(new JoystickButton(new XboxController(2), 6))
      .onTrue(new InstantCommand(() -> {
        arm.encoder.setPosition(0);
      }));
    //new JoystickButton(new Joystick(0), 2).onTrue(new IntakeCone(intake));
    //new JoystickButton(new Joystick(0), 3).onTrue(new IntakeCube(intake));
    //new JoystickButton(new Joystick(0), 4).onTrue(new Outtake(intake));
    new JoystickButton(new XboxController(3), 5).onTrue(new ArmToPreset(arm, ArmPositionPresets.L3));
    new JoystickButton(new XboxController(4), 3).onTrue(new ArmToPreset(arm, ArmPositionPresets.DS));
    new JoystickButton(new XboxController(5), 2).onTrue(new ArmToPreset(arm, ArmPositionPresets.L2));
    new JoystickButton(new XboxController(3), 1).onTrue(new ArmToPreset(arm, ArmPositionPresets.L1));
    new JoystickButton(new XboxController(5), 5).onTrue(new ArmToPreset(arm, ArmPositionPresets.ESCAPE));
    //new JoystickButton(new Joystick(1), 4).onTrue(new ArmToPreset(arm, ArmPositionPresets.L1));
    //new JoystickButton(new Joystick(1), 5).onTrue(new ArmToPreset(arm, ArmPositionPresets.L2));
    //new JoystickButton(new Joystick(1), 6).onTrue(new ArmToPreset(arm, ArmPositionPresets.L3));


    //new JoystickButton(new Joystick(1), 9).whileTrue(new DriveToWaypoint(driveTrain, poseEstimator, new Pose2d(2.32, 1.05, Rotation2d.fromDegrees(180))));

    // Temp auto score L3
    new JoystickButton(new Joystick(1), 10).onTrue(new SequentialCommandGroup(
      new DriveIntoGriddy(driveTrain, poseEstimator, "ColumnTwoAlign"),
      new ArmToPreset(arm, ArmPositionPresets.L3).until(() -> arm.getPosition() > ArmPositionPresets.L2.position),
      new ParallelCommandGroup(
        new DriveWithSpeeds(driveTrain, () -> -0.2, () -> 0.0, () -> 0.0).withTimeout(1),
        new WaitUntilCommand(arm::atSetpoint)
      ),
      new Outtake(intake)
    ));
  }

  /** Runs every 10ms */
  public void periodic10 () {
    arm.periodic10();
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autonChooser.getSelected();
  }
}
