// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Elevator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Elevator.stateLevel;
import frc.robot.subsystems.Grabber.GrabberPlacement;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ElevatorBumperUp extends Command {
  /** Creates a new NextPosElevator. */
  boolean finish = false;

  public ElevatorBumperUp() {
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
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    switch (RobotContainer.m_Elevator.getLevel()) {
      case REST:
        RobotContainer.m_Elevator.setLevel(stateLevel.L1);
        System.out.println("L1 elev");
        break;
      case L1:
        RobotContainer.m_Elevator.setLevel(stateLevel.L2);
        System.out.println("L2 elev");
        break;
      case L2:
        RobotContainer.m_Elevator.setLevel(stateLevel.L3);
        System.out.println("L3 elev");
        break;
      case L3:
        RobotContainer.m_Elevator.setLevel(stateLevel.L4);
        System.out.println("L4 elev");
        break;
      case L4:
        RobotContainer.m_Grabber.setPlacement(GrabberPlacement.L1);
        System.out.println("L1 elev");
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
