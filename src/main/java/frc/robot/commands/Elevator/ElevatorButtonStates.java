// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Elevator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Elevator.stateLevel;
import frc.robot.subsystems.Grabber.GrabberPlacement;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ElevatorButtonStates extends Command {
  /** Creates a new MovePosElevator. */
  boolean finish = false;
  stateLevel state;
  public ElevatorButtonStates(stateLevel state) {
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
    finish = true;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    switch (state){
      case HIGHALGAE:
        RobotContainer.m_Elevator.setLevel(stateLevel.HIGHALGAE);
        System.out.println("high algae");
        break;
      case LOWALGAE:
        RobotContainer.m_Elevator.setLevel(stateLevel.LOWALGAE);
        System.out.println("low algae");
        break;
      case PROCESSOR:
        RobotContainer.m_Elevator.setLevel(stateLevel.PROCESSOR);
        System.out.println("processor");
        break;
      case BARGE:
        RobotContainer.m_Elevator.setLevel(stateLevel.BARGE);
        System.out.println("barge");
        break;
      case FEEDER:
        RobotContainer.m_Elevator.setLevel(stateLevel.FEEDER);
        System.out.println("feeder");
        break;
      case GROUND:
        RobotContainer.m_Elevator.setLevel(stateLevel.GROUND);
        System.out.println("ground");
        break;
      default:
        break;
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return finish;
  }
}