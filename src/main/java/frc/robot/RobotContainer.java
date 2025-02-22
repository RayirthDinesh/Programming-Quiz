package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.ComplexWidget;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.Climb.ClimbMove;
import frc.robot.commands.Grabber.GrabberIntake;
import frc.robot.commands.Grabber.GrabberOutake;
import frc.robot.commands.ParallelCommands.ElevatorAndGrabberBumperDown;
import frc.robot.commands.ParallelCommands.ElevatorAndGrabberButtonStates;
import frc.robot.commands.ParallelCommands.ElevatorandGrabberBumperUp;
import frc.robot.subsystems.Climb;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Elevator.stateLevel;
import frc.robot.subsystems.Elevator.stateReset;
import frc.robot.subsystems.Grabber;
import frc.robot.subsystems.Grabber.GrabberPlacement;
import frc.robot.subsystems.Grabber.States;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Vision;


public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  public static Climb s_Climb = new Climb();
  public static Elevator s_Elevator = new Elevator();
  public static Grabber s_Grabber = new Grabber();
  public static Swerve s_Swerve = new Swerve();
  public static Vision s_Vision = new Vision();

  public static Joystick driverJoystick = new Joystick(0);
  public static Joystick operatorJoystick = new Joystick(1);

  public static JoystickButton m_IntakeButton = new JoystickButton(driverJoystick, 5);
  public static JoystickButton m_OuttakeButton = new JoystickButton(driverJoystick, 6);
  public static JoystickButton zeroGyro = new JoystickButton(driverJoystick, 10);
 
  public static JoystickButton CenterAim = new JoystickButton(driverJoystick, 4);
  public static JoystickButton LeftAim = new JoystickButton(driverJoystick, 1);
  public static JoystickButton RightAim = new JoystickButton(driverJoystick, 3);


  private final SendableChooser<Command> autoChooser = new SendableChooser<>();
  ComplexWidget ShuffleBoardAutonomousRoutines = Shuffleboard.getTab("Driver")
      .add("Autonomous Routines Selector", autoChooser).withWidget(BuiltInWidgets.kComboBoxChooser).withSize(2, 2)
      .withPosition(0, 2);
  // public static JoystickButton ElevatorUp = new
  // JoystickButton(operatorJoystick, 8);
  // public static JoystickButton ElevatorDown = new
  // JoystickButton(operatorJoystick, 9);
  // public static final JoystickButton wristUp = new
  // JoystickButton(operatorJoystick, 8);
  // public static final JoystickButton wristDown = new
  // JoystickButton(operatorJoystick, 9);
  public static JoystickButton Next = new JoystickButton(operatorJoystick, 6);
  public static JoystickButton Previous = new JoystickButton(operatorJoystick, 8);
  public static JoystickButton Feeder = new JoystickButton(operatorJoystick, 3);
  public static JoystickButton Ground = new JoystickButton(operatorJoystick, 4);
  public static JoystickButton moveClimb = new JoystickButton(operatorJoystick, 10);


  // Replace with CommandPS4Controller or CommandJoystick if needed

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be
   * created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
   * an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link
   * CommandXboxController
   * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or
   * {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // if(s_Grabber.getState() == States.ENCODER && s_Elevator.getState() == stateReset.INITIALIZED){
      m_IntakeButton.onTrue(new GrabberIntake());
      m_OuttakeButton.onTrue(new GrabberOutake());
  
      // wristDown.onTrue(new GrabberManualMoveDown());
      // wristUp.onTrue(new GrabberManualMoveUp());
  
      // ElevatorUp.onTrue(new ElevatorManualMoveUp());
      // ElevatorDown.onTrue(new ElevatorManualMoveDown());
  
      zeroGyro.onTrue(new InstantCommand(() -> s_Swerve.zeroHeading()));
      moveClimb.onTrue(new ClimbMove());
  
      // LowAlgae.onTrue(new ElevatorAndGrabberButtonStates(stateLevel.LOWALGAE, GrabberPlacement.LOWALGAE));
      // HighAlgae.onTrue(new ElevatorAndGrabberButtonStates(stateLevel.HIGHALGAE, GrabberPlacement.HIGHALGAE));
      // Barge.onTrue(new ElevatorAndGrabberButtonStates(stateLevel.BARGE, GrabberPlacement.BARGE));
      // Processor.onTrue(new ElevatorAndGrabberButtonStates(stateLevel.PROCESSOR, GrabberPlacement.PROCESSOR));
      
      Feeder.onTrue(new ElevatorAndGrabberButtonStates(stateLevel.FEEDER, GrabberPlacement.FEEDER));
      Ground.onTrue(new ElevatorAndGrabberButtonStates(stateLevel.GROUND, GrabberPlacement.GROUND));
  
  
      Next.onTrue(new ElevatorandGrabberBumperUp());
      Previous.onTrue(new ElevatorAndGrabberBumperDown());
  
      RightAim.onTrue(new InstantCommand(() -> s_Vision.setAutoAim(Vision.autoAim.RIGHT)));
      LeftAim.onTrue(new InstantCommand(() -> s_Vision.setAutoAim(Vision.autoAim.LEFT)));
      CenterAim.onTrue(new InstantCommand(() -> s_Vision.setAutoAim(Vision.autoAim.MIDDLE)));
  
      RightAim.onFalse(new InstantCommand(() -> s_Vision.setAutoAim(Vision.autoAim.NONE)));
      LeftAim.onFalse(new InstantCommand(() -> s_Vision.setAutoAim(Vision.autoAim.NONE)));
      CenterAim.onFalse(new InstantCommand(() -> s_Vision.setAutoAim(Vision.autoAim.NONE)));
  
  
    // }
    
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return null;
  }
}
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// import frc.robot.Constants.OperatorConstants;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
