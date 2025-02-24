// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Grabber;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Grabber.States;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class GrabberRecalibrate extends Command {
  /** Creates a new GrabberRecalibrate. */
  boolean finish = false;
  public GrabberRecalibrate() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.s_Grabber);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    RobotContainer.s_Grabber.slowMode();
    RobotContainer.s_Grabber.setState(States.INITIALIZING);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (!RobotContainer.s_Grabber.getLimitSwitch()){
      RobotContainer.s_Grabber.moveTurningMotor(RobotContainer.s_Grabber.getPos() + 0.03);
    } else {
      RobotContainer.s_Grabber.setPos(0);
      RobotContainer.s_Grabber.moveTurningMotor(0);
      RobotContainer.s_Grabber.setState(States.INITIALIZED);
      finish = true;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.s_Grabber.normalMode();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return finish;
  }
}
