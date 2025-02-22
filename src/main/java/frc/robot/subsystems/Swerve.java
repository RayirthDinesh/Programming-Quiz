package frc.robot.subsystems;

import java.lang.reflect.Field;
import java.util.Map;

import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.ComplexWidget;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.SwerveModules;

public class Swerve extends SubsystemBase {
  /** Creates a new Swerve. */
  Field2d field = new Field2d();
  ComplexWidget fieldWidget = Shuffleboard.getTab("Swerve").add("Field", field).withWidget(BuiltInWidgets.kField);

  public SwerveDriveOdometry swerveOdometry;
  public SwerveModules[] mSwerveMods;
  public static Pigeon2 gyro;
  public ChassisSpeeds mSpeeds;
  public SwerveDrivePoseEstimator robotPose;
  public boolean encoderJoymodeState = false; 

  // ShuffleboardTab joystickTab = Shuffleboard.getTab("Joystick");
  // public GenericEntry pEntry = joystickTab.add("P Gain", 0.08).getEntry();
  // public GenericEntry iEntry = joystickTab.add("I Gain", 0.00).getEntry();
  // public GenericEntry dEntry = joystickTab.add("D Gain", 0.005).getEntry();
  // public GenericEntry currentRotValueEntry = joystickTab.add("Current Rotation Value", 0.005).getEntry();
  // public GenericEntry targetrotValueEntry = joystickTab.add("Target Rotation Value", 0.005).getEntry();
  GenericEntry EncoderModeEntry = Shuffleboard.getTab("Swerve").add("Encoder Mode",encoderJoymodeState).getEntry();


  public Swerve() {

    gyro = new Pigeon2(Constants.SwerveConstants.PIGEON_ID);
    gyro.getConfigurator().apply(new Pigeon2Configuration());
    gyro.setYaw(0);

    // Initialize the Swerve modules array
    mSwerveMods = new SwerveModules[] {
        new SwerveModules(0, Constants.SwerveConstants.Mod0.DRIVE_MOTOR_ID, Constants.SwerveConstants.Mod0.ANGLE_MOTOR_ID,
            Constants.SwerveConstants.Mod0.CANCODER_ID, Constants.SwerveConstants.Mod0.ANGLE_OFFSET),
        new SwerveModules(1, Constants.SwerveConstants.Mod1.DRIVE_MOTOR_ID, Constants.SwerveConstants.Mod1.ANGLE_MOTOR_ID,
            Constants.SwerveConstants.Mod1.CANCODER_ID, Constants.SwerveConstants.Mod1.ANGLE_OFFSET),
        new SwerveModules(2, Constants.SwerveConstants.Mod2.DRIVE_MOTOR_ID, Constants.SwerveConstants.Mod2.ANGLE_MOTOR_ID,
            Constants.SwerveConstants.Mod2.CANCODER_ID, Constants.SwerveConstants.Mod2.ANGLE_OFFSET),
        new SwerveModules(3, Constants.SwerveConstants.Mod3.DRIVE_MOTOR_ID, Constants.SwerveConstants.Mod3.ANGLE_MOTOR_ID,
            Constants.SwerveConstants.Mod3.CANCODER_ID, Constants.SwerveConstants.Mod3.ANGLE_OFFSET)
    };
    swerveOdometry = new SwerveDriveOdometry(Constants.SwerveConstants.SWERVE_KINEMATICS, getGyroYaw(), getModulePositions());
    robotPose = new SwerveDrivePoseEstimator(Constants.SwerveConstants.SWERVE_KINEMATICS, getGyroYaw(), getModulePositions(),
        getPose());

    RobotConfig config = null;
    try {
      config = RobotConfig.fromGUISettings();
    } catch (Exception e) {
      // Handle exception as needed
      e.printStackTrace();
    }

 
    // AutoBuilder.configure(
    //     this::getPose, // Robot pose supplier
    //     this::setPose, // Method to reset odometry (will be called if your auto has a starting pose)
    //     this::getChassisSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
    //     (speeds, feedforwards) -> drive(speeds), // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds.
    //                                              // Also optionally outputs individual module feedforwards
    //     new PPHolonomicDriveController( // PPHolonomicController is the built in path following controller for holonomic
    //                                     // drive trains
    //         new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
    //         new PIDConstants(5.0, 0.0, 0.0) // Rotation PID constants
    //     ),
    //     config, // The robot configuration
    //     () -> {
    //       // Boolean supplier that controls when the path will be mirrored for the red
    //       // alliance
    //       // This will flip the path being followed to the red side of the field.
    //       // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

    //       var alliance = DriverStation.getAlliance();
    //       if (alliance.isPresent()) {
    //         return alliance.get() == DriverStation.Alliance.Red;
    //       }
    //       return false;
    //     },
    //     this // Reference to this subsystem to set requirements
    // );
  }

  // return modulestates of all swerve modules in a list
  public SwerveModuleState[] getModuleStates() {
    SwerveModuleState[] states = new SwerveModuleState[4];
    for (SwerveModules mod : mSwerveMods) {
      states[mod.modNumber] = mod.getState();
    }
    return states;
  }

  // return modulestates of all swerve modules in a list
  public SwerveModulePosition[] getModulePositions() {
    SwerveModulePosition[] positions = new SwerveModulePosition[4];
    for (SwerveModules mod : mSwerveMods) {
      positions[mod.modNumber] = mod.getPosition();
    }
    return positions;
  }

  // move to set location
  public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
    SwerveModuleState[] swerveModuleStates = Constants.SwerveConstants.SWERVE_KINEMATICS.toSwerveModuleStates(
        fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(
            translation.getX(),
            translation.getY(),
            rotation,
            Rotation2d.fromDegrees(gyro.getYaw().getValueAsDouble()))
            : new ChassisSpeeds(
                translation.getX(),
                translation.getY(),
                rotation));
    mSpeeds = new ChassisSpeeds(translation.getX(), translation.getY(), rotation);
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.SwerveConstants.MAX_SPEED_METERS_PER_SECOND);

    for (SwerveModules mod : mSwerveMods) {
      mod.setDesiredState(swerveModuleStates[mod.modNumber], isOpenLoop);

    }
  }

  public void autoMove() {
    if (RobotContainer.s_Vision.isApriltag() == true) {
      double strafeVal = RobotContainer.s_Vision.autostrafe() * Constants.SwerveConstants.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND;
      double rotationval = RobotContainer.s_Vision.autoAngle() * Constants.SwerveConstants.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND;
      double translationVal = RobotContainer.s_Vision.autotrans() * Constants.SwerveConstants.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND;
      RobotContainer.s_Swerve.drive(new Translation2d(translationVal, strafeVal).times(Constants.SwerveConstants.MAX_SPEED_METERS_PER_SECOND),
          rotationval, true, true);
    }
  }

  // set speeds of all modules and move to current location
  public void drive(ChassisSpeeds speeds) {
    SwerveModuleState[] swerveModuleStates = Constants.SwerveConstants.SWERVE_KINEMATICS.toSwerveModuleStates(speeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.SwerveConstants.MAX_SPEED_METERS_PER_SECOND);
    mSpeeds = speeds;

    for (SwerveModules mod : mSwerveMods) {
      mod.setDesiredState(swerveModuleStates[mod.modNumber], true);
    }
  }

  public ChassisSpeeds getChassisSpeeds() {
    return mSpeeds;
  }

  /* Used by SwerveControllerCommand in Auto */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    // renormalize wheelspeeds
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.SwerveConstants.MAX_SPEED_METERS_PER_SECOND);

    // move to desired state with optimization
    for (SwerveModules mod : mSwerveMods) {
      mod.setDesiredState(desiredStates[mod.modNumber], true);
    }
  }

  // return current position of swerveodometry (the chassis internal positioning
  // system)
  public Pose2d getPose() {
    return swerveOdometry.getPoseMeters();
  }

  // Change the position of the swerveodometry
  public void setPose(Pose2d pose) {
    swerveOdometry.resetPosition(getGyroYaw(), getModulePositions(), pose);
  }

  // get the direction of robot face
  public Rotation2d getHeading() {
    return getPose().getRotation();
  }

  // change the direct of the robot face (use the current position to get there)
  public void setHeading(Rotation2d heading) {
    swerveOdometry.resetPosition(getGyroYaw(), getModulePositions(), new Pose2d(getPose().getTranslation(), heading));
  }

  // reset heading to 0 degrees
  public void zeroHeading() {
    swerveOdometry.resetPosition(getGyroYaw(), getModulePositions(),
        new Pose2d(getPose().getTranslation(), new Rotation2d()));
    // for (SwerveModules mod : mSwerveMods) {
    // mod.setHeadingZero();
    // }
    gyro.setYaw(90);

  }

  // returns curent rotation
  public Rotation2d getGyroYaw() {
    return Rotation2d.fromDegrees(gyro.getYaw().getValueAsDouble());
  }

  // reset all modules to absolute postion (what was recorded previously, accounts
  // for offsets)
  public void resetModulesToAbsolute() {
    for (SwerveModules mod : mSwerveMods) {
      mod.resetToAbsolute();
    }
  }

  @Override
  public void periodic() {
    swerveOdometry.update(getGyroYaw(), getModulePositions());
    // This method will be called once per scheduler run
    EncoderModeEntry.setBoolean(encoderJoymodeState);
    SmartDashboard.putNumber("Gyro Yaw", getGyroYaw().getDegrees());

    for (SwerveModules mod : mSwerveMods) {
      SmartDashboard.putNumber("Module " + mod.modNumber + " Desired Angle: ", mod.Desired());
      SmartDashboard.putNumber("Mod " + mod.modNumber + " CANcoder", mod.getCANcoder().getDegrees());
      SmartDashboard.putNumber("Mod " + mod.modNumber + " Angle", mod.getPosition().angle.getDegrees());
      SmartDashboard.putNumber("Mod " + mod.modNumber + " Velocity", mod.getState().speedMetersPerSecond);

    }

  }
}