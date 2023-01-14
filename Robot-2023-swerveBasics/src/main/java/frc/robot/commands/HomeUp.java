package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class MoveElevatorUp extends CommandBase {

    public HomeUp() {}

    @Override
    public void initialize() {
        RobotContainer.elevator.setElevatorMotor(Constants.ElevatorSpeed);
    }

    @Override
    public void execute() {}

    @Override
    public boolean isFinished() {
        return RobotContainer.elevator.topLimitSwitch.get();
    }

    @Override
    public void end(boolean interrupted) {
        RobotContainer.elevator.stopElevator();
        
    }
}