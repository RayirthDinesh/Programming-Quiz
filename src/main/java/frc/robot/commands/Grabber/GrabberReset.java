// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Grabber;


import java.lang.Thread.State;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Grabber.States;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class GrabberReset extends Command {
  boolean finish = false;
  /** Creates a new reset. */
  public GrabberReset() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.s_Grabber);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (!RobotContainer.s_Grabber.getLimitSwitch()){

      // RobotContainer.s_Grabber.disableMotor();
      RobotContainer.s_Grabber.setState(States.NOT_INITIALIZED);

    }

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (!RobotContainer.s_Grabber.getLimitSwitch() && RobotContainer.s_Grabber.getState() == States.INITIALIZED){
      RobotContainer.s_Grabber.setPos(0);
      RobotContainer.s_Grabber.moveTurningMotor(0);
      finish = true;
    } 

    if(RobotContainer.s_Grabber.getLimitSwitch()){
      RobotContainer.s_Grabber.moveTurningMotor(RobotContainer.s_Grabber.getPos() - 0.03);
      RobotContainer.s_Grabber.setState(States.INITIALIZED);

    }
    if(RobotContainer.s_Grabber.getState() == States.NOT_INITIALIZED && !RobotContainer.s_Grabber.getLimitSwitch() ){
      RobotContainer.s_Grabber.moveTurningMotor(RobotContainer.s_Grabber.getPos()+0.03);
      //RobotContainer.s_Grabber.setState(States.INITIALIZING);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.s_Elevator.setPosition(0);
    RobotContainer.s_Elevator.movePosition(0);
    // RobotContainer.s_Grabber.setPos(0);
    // RobotContainer.s_Grabber.moveTurningMotor(0);
    // RobotContainer.s_Grabber.setState(States.INITIALIZED);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return finish;
  }
}
