// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Elevator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Elevator.stateLevel;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ElevatorBumperDown extends Command {
  /** Creates a new PreviousPs. */
  boolean finish = false;
  public ElevatorBumperDown() {

    addRequirements(RobotContainer.m_Elevator);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    finish = true;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    switch (RobotContainer.m_Elevator.getLevel()){
      case L1:
        RobotContainer.m_Elevator.setLevel(stateLevel.L4);
        System.out.println("L4");
        break;
      case L2:
        RobotContainer.m_Elevator.setLevel(stateLevel.L1);
        System.out.println("L1");
        break;
      case L3:
        RobotContainer.m_Elevator.setLevel(stateLevel.L2);
        System.out.println("L2");
        break;
      case L4:
        RobotContainer.m_Elevator.setLevel(stateLevel.L3);
        System.out.println("L3");
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
