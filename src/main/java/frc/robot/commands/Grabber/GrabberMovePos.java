// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Grabber;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Grabber.GrabberPlacement;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class GrabberMovePos extends Command {
  GrabberPlacement place;
  boolean finish;

  /** Creates a new moveGrabber. */
  public GrabberMovePos(GrabberPlacement place) {
    addRequirements(RobotContainer.s_Grabber);
    this.place = place;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    finish = true;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    switch (place) {
          case L1 -> RobotContainer.s_Grabber.moveTurningMotor(Constants.GrabberConstants.LEVEL_1);
          case L2 -> RobotContainer.s_Grabber.moveTurningMotor(Constants.GrabberConstants.LEVEL_1_TO_3);
          case L3 -> RobotContainer.s_Grabber.moveTurningMotor(Constants.GrabberConstants.LEVEL_1_TO_3);
          case L4 -> RobotContainer.s_Grabber.moveTurningMotor(Constants.GrabberConstants.LEVEL_4);
          case GROUND -> RobotContainer.s_Grabber.moveTurningMotor(Constants.GrabberConstants.GROUND_POSITION);
          case FEEDER -> RobotContainer.s_Grabber.moveTurningMotor(Constants.GrabberConstants.FEEDER_POSITION);
          case BARGE -> RobotContainer.s_Grabber.moveTurningMotor(Constants.GrabberConstants.BARGE_POSITION);
          case REST -> RobotContainer.s_Grabber.moveTurningMotor(Constants.GrabberConstants.REST_POSITION);
          case PROCESSOR -> RobotContainer.s_Grabber.moveTurningMotor(Constants.GrabberConstants.PROCESSOR_POSITION);
          case ALGAE_ON_TOP -> RobotContainer.s_Grabber.moveTurningMotor(Constants.GrabberConstants.ALGAE_ON_TOP);
          default -> {
        }
      }
  }
  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return finish;
  }
}