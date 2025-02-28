// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Elevator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Elevator.stateLevel;


/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ElevatorMovePos extends Command {

  stateLevel state;
  boolean finish;
  /** Creates a new moveElevator. */
  public ElevatorMovePos(stateLevel state) {
    addRequirements(RobotContainer.s_Elevator);
    this.state = state;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    finish = true;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    switch (state) {
      case L1 -> RobotContainer.s_Elevator.movePosition(Constants.ElevatorConstants.LEVEL_1);
      case L2 -> RobotContainer.s_Elevator.movePosition(Constants.ElevatorConstants.LEVEL_2);
      case L3 -> RobotContainer.s_Elevator.movePosition(Constants.ElevatorConstants.LEVEL_3);
      case L4 -> RobotContainer.s_Elevator.movePosition(Constants.ElevatorConstants.LEVEL_4);
      case GROUND -> RobotContainer.s_Elevator.movePosition(Constants.ElevatorConstants.GROUND_POSITION);
      case FEEDER -> RobotContainer.s_Elevator.movePosition(Constants.ElevatorConstants.FEEDER_POSITION);
      case BARGE -> RobotContainer.s_Elevator.movePosition(Constants.ElevatorConstants.BARGE_POSITION);
      case REST -> RobotContainer.s_Elevator.movePosition(Constants.ElevatorConstants.REST_POSITION);
      case LOWALGAE -> RobotContainer.s_Elevator.movePosition(Constants.ElevatorConstants.LOW_ALGAE_POSITION);
      case HIGHALGAE -> RobotContainer.s_Elevator.movePosition(Constants.ElevatorConstants.HIGH_ALGAE_POSITION);
      case PROCESSOR -> RobotContainer.s_Elevator.movePosition(Constants.ElevatorConstants.PROCESSOR_POSITION);
      default -> throw new AssertionError(state.name());
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return finish;
  }
}
