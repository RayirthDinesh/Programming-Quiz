// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Elevator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Elevator.stateReset;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ElevatorRecalibrate extends Command {
  /** Creates a new ElevatorRecalibrate. */
  boolean finish = false;
  public ElevatorRecalibrate() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.s_Elevator);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    RobotContainer.s_Elevator.slowMode();
    RobotContainer.s_Elevator.changeState(stateReset.INITIALIZING);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (!RobotContainer.s_Elevator.getLimitSwitch()) {
      RobotContainer.s_Elevator.movePosition(RobotContainer.s_Elevator.getPosition()-0.3);
    } else {  
      RobotContainer.s_Elevator.changeState(stateReset.INITIALIZED);
      RobotContainer.s_Elevator.setPosition(0);
      RobotContainer.s_Elevator.movePosition(0);
      finish = true;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.s_Elevator.normalMode();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return finish;
  }
}
