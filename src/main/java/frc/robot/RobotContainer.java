package frc.robot;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

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
import frc.robot.commands.Elevator.ElevatorReset;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Vision;

public class RobotContainer {
    // The robot's subsystems and commands are defined here...
    // public static Climb s_Climb = new Climb();

    public static Elevator s_Elevator = new Elevator();
    public static Swerve s_Swerve = new Swerve();
    public static Vision s_Vision = new Vision();

    public static Joystick driverJoystick = new Joystick(0);
    public static Joystick operatorJoystick = new Joystick(1);

    public static JoystickButton m_IntakeButton = new JoystickButton(driverJoystick, 5);
    public static JoystickButton m_OuttakeButton = new JoystickButton(driverJoystick, 6);
    public static JoystickButton zeroGyro = new JoystickButton(driverJoystick, 2);

    public static JoystickButton CenterAim = new JoystickButton(driverJoystick, 4);
    public static JoystickButton LeftAim = new JoystickButton(driverJoystick, 1);
    public static JoystickButton RightAim = new JoystickButton(driverJoystick, 3);

    public static JoystickButton wristUp = new JoystickButton(driverJoystick, 8);
    public static JoystickButton wristDown = new JoystickButton(driverJoystick, 7);
    // public static JoystickButton Recalibrate = new JoystickButton(driverJoystick,
    // 9);

    private final SendableChooser<Command> autoChooser = new SendableChooser<>();
    ComplexWidget ShuffleBoardAutonomousRoutines = Shuffleboard.getTab("Driver")
            .add("Autonomous Routines Selector", autoChooser).withWidget(BuiltInWidgets.kComboBoxChooser).withSize(2, 2)
            .withPosition(0, 2);
    public static JoystickButton ElevatorUp = new JoystickButton(operatorJoystick, 10);//Elevator Up slightly is options
    public static JoystickButton ElevatorDown = new JoystickButton(operatorJoystick, 9); // Elevator Down Slightly is share

    public static JoystickButton Next = new JoystickButton(operatorJoystick, 6);//Previous is Left Bumper
    public static JoystickButton Previous = new JoystickButton(operatorJoystick, 8);//Next is Right Bumper (R1)
    public static JoystickButton Feeder = new JoystickButton(operatorJoystick, 3);//Coral Feeder intake is circle
    public static JoystickButton Ground = new JoystickButton(operatorJoystick, 2);//Ground Algae intake is triangle
    // public static JoystickButton moveClimb = new JoystickButton(operatorJoystick, 10);

    public static JoystickButton Processor = new JoystickButton(operatorJoystick, 1);
    ///Proccessor is square
  public static JoystickButton Scram = new JoystickButton(operatorJoystick, 5); //Scram is Left Bumper (L1)
    public static JoystickButton Barge = new JoystickButton(operatorJoystick, 4); //Barge  is x

    // Replace with CommandPS4Controller or CommandJoystick if needed
    /**
     * The container for the robot. Contains subsystems, OI devices, and
     * commands.
     */
    public RobotContainer() {
        NamedCommands.registerCommand("ResetElev", new ElevatorReset());

        // NamedCommands.registerCommand("High Algae",
        //     new ElevatorAndGrabberMovePos(GrabberPlacement.HIGHALGAE, stateLevel.HIGHALGAE));
        // NamedCommands.registerCommand("Low Algae",
        //     new ElevatorAndGrabberMovePos(GrabberPlacement.LOWALGAE, stateLevel.LOWALGAE));
        //NamedCommands.registerCommand("AutoAimOff", new InstantCommand(() -> s_Swerve.autoaimstate = false).andThen(new InstantCommand(() -> RobotContainer.s_Swerve.drive(new Translation2d(0, 0), 0, false, true))));

        NamedCommands.registerCommand("stop", getAutonomousCommand());
        // Configure the trigger bindings
        configureBindings();
        // autoChooser.setDefaultOption("SPEAKER Routine", new SpeakerRoutine());
        autoChooser.addOption("TR L2 Barge", new PathPlannerAuto("TR L2 Barge"));
        autoChooser.addOption("BR L2 Barge", new PathPlannerAuto("BR L2 Barge"));

        autoChooser.addOption("S1-Forward", new PathPlannerAuto("S1-Forward"));
        autoChooser.addOption("S2-Forward", new PathPlannerAuto("S2-Forward"));
        autoChooser.addOption("S3-Forward", new PathPlannerAuto("S3-Forward"));
        autoChooser.addOption("Testing2", new PathPlannerAuto("Testing2"));
        autoChooser.addOption("Algae T to Processor", new PathPlannerAuto("Algae T to Processor"));
        autoChooser.addOption("Start to Reef TL 3", new PathPlannerAuto("Start to Reef TL 3"));
        autoChooser.addOption("null", new PathPlannerAuto("null"));
    }

    /**
     * Use this method to define your trigger->command mappings. Triggers can be
     * created via the
     * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor
     * with an arbitrary predicate, or via the named factories in {@link
     * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for      {@link
   * CommandXboxController
     * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
     * PS4} controllers or      {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
     */
    private void configureBindings() {

        zeroGyro.onTrue(new InstantCommand(() -> s_Swerve.zeroHeading()));
// 

        // LowAlgae.onTrue(new ElevatorAndGrabberButtonStates(stateLevel.LOWALGAE,
        // GrabberPlacement.LOWALGAE));
        // HighAlgae.onTrue(new ElevatorAndGrabberButtonStates(stateLevel.HIGHALGAE,
        // GrabberPlacement.HIGHALGAE));
        // Barge.onTrue(new ElevatorAndGrabberButtonStates(stateLevel.BARGE,
        // GrabberPlacement.BARGE));
        // Processor.onTrue(new ElevatorAndGrabberButtonStates(stateLevel.PROCESSOR,
        // GrabberPlacement.PROCESSOR));
        // Recalibrate.onTrue(new ElevatorAndGrabberRecalibrate());
        // }
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {

        // An example command will be run in autonomous
        return autoChooser.getSelected();
    }
}
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// import frc.robot.Constants.OperatorConstants;
/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and trigger mappings) should be declared here.
 */
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and trigger mappings) should be declared here.
 */
