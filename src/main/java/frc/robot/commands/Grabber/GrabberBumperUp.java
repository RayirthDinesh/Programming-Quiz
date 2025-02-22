// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Grabber;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Grabber.GrabberPlacement;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class GrabberBumperUp extends Command {
  /** Creates a new NextPos. */
  boolean finish;
  public GrabberBumperUp() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.s_Grabber);
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
    switch (RobotContainer.s_Grabber.getPlacement()){
      case REST -> RobotContainer.s_Grabber.setPlacement(GrabberPlacement.L1);
      case L1 -> RobotContainer.s_Grabber.setPlacement(GrabberPlacement.L2);
      case L2 -> RobotContainer.s_Grabber.setPlacement(GrabberPlacement.L3);
      case L3 -> RobotContainer.s_Grabber.setPlacement(GrabberPlacement.L4);
      case L4 -> RobotContainer.s_Grabber.setPlacement(GrabberPlacement.L1);
      default -> RobotContainer.s_Grabber.setPlacement(GrabberPlacement.L1);
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return finish;
  }
}