// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Elevator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Elevator.stateLevel;
import frc.robot.subsystems.Grabber.GrabberPlacement;;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ElevatorMovePos extends Command {

  stateLevel state;
  boolean finish;
  /** Creates a new moveElevator. */
  public ElevatorMovePos(stateLevel state) {
    addRequirements(RobotContainer.m_Elevator);
    this.state = state;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if (state == stateLevel.L1){
      RobotContainer.m_Elevator.movePosition(Constants.ElevatorConstants.LEVEL_1);
    } else if (state == stateLevel.L2){
      RobotContainer.m_Elevator.movePosition(Constants.ElevatorConstants.LEVEL_2);
    } else if (state== stateLevel.L3){
      RobotContainer.m_Elevator.movePosition(Constants.ElevatorConstants.LEVEL_3);
    } else if (state == stateLevel.L4){
      RobotContainer.m_Elevator.movePosition(Constants.ElevatorConstants.LEVEL_4);
    } else if (state == stateLevel.GROUND){
      RobotContainer.m_Elevator.movePosition(Constants.ElevatorConstants.GROUND_POSITION);
    } else if (state == stateLevel.FEEDER){
      RobotContainer.m_Elevator.movePosition(Constants.ElevatorConstants.FEEDER_POSITION);
    } else if (state == stateLevel.BARGE){
      RobotContainer.m_Elevator.movePosition(Constants.ElevatorConstants.BARGE_POSITION);
    } else if (state == stateLevel.REST){
      RobotContainer.m_Elevator.movePosition(Constants.ElevatorConstants.REST_POSITION);
    } else if (state == stateLevel.LOWALGAE){
      RobotContainer.m_Elevator.movePosition(Constants.ElevatorConstants.LOW_ALGAE_POSITION);
    } else if (state == stateLevel.HIGHALGAE){
      RobotContainer.m_Elevator.movePosition(Constants.ElevatorConstants.HIGH_ALGAE_POSITION);
    } 
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return finish;
  }
}
