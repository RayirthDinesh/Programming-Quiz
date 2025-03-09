// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.MAXMotionConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;



public class Grabber extends SubsystemBase {
  /** Creates a new Grabber. */

  public enum States {
    INITIALIZED,
    INITIALIZING,
    NOT_INITIALIZED,
    ENCODER
  }

  public enum GrabberPlacement {
    L1,
    L2,
    L3,
    L4,
    GROUND,
    FEEDER,
    REST,
    BARGE,
    PROCESSOR,
    HIGHALGAE,
    LOWALGAE
    
  }

  public enum IntakeOuttake {
    INTAKE,
    OUTTAKE,
    NOTHING
  }

  static double target;

  static Timer timer = new Timer();
  SparkMax max = new SparkMax(Constants.GrabberConstants.FOLLOWER_NEOMOTOR_PORT, MotorType.kBrushless);
  SparkMax maxLeader = new SparkMax(Constants.GrabberConstants.LEADER_NEOMOTOR_PORT, MotorType.kBrushless);
  TalonFX turning = new TalonFX(Constants.GrabberConstants.JOINT_MOTOR_PORT);
  DigitalInput limitswitch = new DigitalInput(Constants.GrabberConstants.LIMITSWITCH_PORT);


  SparkMaxConfig config = new SparkMaxConfig();
  SparkMaxConfig leaderConfig = new SparkMaxConfig();
  MAXMotionConfig maxMotionConfig = new MAXMotionConfig();
  SparkClosedLoopController maxController = max.getClosedLoopController();
  
  TalonFXConfiguration talonConfig = new TalonFXConfiguration();
  MotionMagicVoltage motion = new MotionMagicVoltage(0);
 

  States curStates = States.INITIALIZING;
  GrabberPlacement curPlacement = GrabberPlacement.REST;
  static IntakeOuttake intakeOuttake = IntakeOuttake.NOTHING;

  static double auxFF = 0;
  static double grabberAngle;

  GenericEntry jointLimitswitchEntry = Shuffleboard.getTab("Grabber").add("Joint limitswitch", limitswitch.get()).getEntry();
  GenericEntry jontCurrentPosEntry = Shuffleboard.getTab("Grabber").add("Joint Current Positon", 0).getEntry();
  GenericEntry jointTargetPosEntry = Shuffleboard.getTab("Grabber").add("JointTarget Position", target).getEntry();
  GenericEntry jointInitStateEntry = Shuffleboard.getTab("Grabber").add("Joint State Init", curStates.toString()).getEntry();
  GenericEntry jointLevelStateEntry = Shuffleboard.getTab("Grabber").add("Joint State Level", curPlacement.toString()).getEntry();
  GenericEntry grabberIntakeOuttakeEntry = Shuffleboard.getTab("Grabber").add("Grabber Intake Outtake", intakeOuttake.toString()).getEntry();


  public Grabber() {
    var slot0config = talonConfig.Slot0;
    var magicmotionconfig = talonConfig.MotionMagic;
    turning.setInverted(true);

    slot0config.kP = 23;
    slot0config.kI = 0;
    slot0config.kD = 0;

    magicmotionconfig.MotionMagicAcceleration = 30;
    magicmotionconfig.MotionMagicCruiseVelocity = 30;

    turning.getConfigurator().apply(talonConfig);
    turning.setNeutralMode(NeutralModeValue.Coast);

    config
        .follow(Constants.GrabberConstants.LEADER_NEOMOTOR_PORT, true)
        .inverted(false)
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(40);
    leaderConfig
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(40);

    maxLeader.configure(leaderConfig, null, null);
    max.configure(config, null, null);

    // maxLeader.set(.07);
    SmartDashboard.putNumber("target", 0);
  }

  public void slowMode(){
    talonConfig.MotionMagic.MotionMagicAcceleration = 5;
    talonConfig.MotionMagic.MotionMagicCruiseVelocity = 5;
    turning.getConfigurator().refresh(talonConfig);
  }

  public void normalMode(){
    talonConfig.MotionMagic.MotionMagicAcceleration = 15;
    talonConfig.MotionMagic.MotionMagicCruiseVelocity = 15;
    turning.getConfigurator().refresh(talonConfig);
  }

  public double getOutputCurrent() {
    return maxLeader.getOutputCurrent();
  }

  public IntakeOuttake getIntakeOuttake() {
    return intakeOuttake;
  }

  public void setIntakeOutake(IntakeOuttake state) {
    intakeOuttake = state;
  }

  public States getState() {
    return curStates;
  }

  public void setPos(double pos) {
    turning.setPosition(pos);
  }

  public void moveNeos(double speed) {
    maxLeader.set(speed);
  }

  public static void Ticker(double time, boolean outake) {
    timer.start();
    if (timer.hasElapsed(time)) {
      timer.stop();
      timer.reset();
      if (outake) {
        intakeOuttake = IntakeOuttake.NOTHING;
      }
    }

  }

  public void moveTurningMotor(double pos) {
    target=pos;
    System.out.println(pos);
    if (curStates == States.ENCODER) {

      if (getPos() > Constants.GrabberConstants.BOTTOM_HARD_LIMIT || getPos() < Constants.GrabberConstants.TOP_HARD_LIMIT) {
        turning.disable();
        target = 0;
      }
      if (getPos() > Constants.GrabberConstants.BOTTOM_SOFT_LIMIT) {
        target = Constants.GrabberConstants.REST_POSITION;
      }
      if (getPos() < Constants.GrabberConstants.TOP_SOFT_LIMIT) {
        target = Constants.GrabberConstants.BARGE_POSITION;
      }
    }
    
    turning.setControl(motion.withPosition(target * Constants.GrabberConstants.GEAR_RATIO));
  }

  public boolean getLimitSwitch() {
    return limitswitch.get();
  }

  public void disableMotor() {
    turning.disable();
    maxLeader.disable();
    max.disable();
  }

  public double getPos() {
    return turning.getPosition().getValueAsDouble() / Constants.GrabberConstants.GEAR_RATIO;
  }

  
  public GrabberPlacement getPlacement() {
    return curPlacement;
  }

  public void setState(States state) {
    curStates = state;
  }

  public void setPlacement(GrabberPlacement place) {
    curPlacement = place;
  }



  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    grabberAngle = 0 + ((getPos() * 360 / Constants.GrabberConstants.GEAR_RATIO)); // try removing gear ratios
    auxFF = /* FFWEntry.getDouble(-0.15) */ -0.35 * Math.sin(Math.toRadians(grabberAngle)); // -0.128
    // auxFF = 0.35 * Math.sin(Math.toRadians((getPos()-25)));
    jointLimitswitchEntry.setBoolean(getLimitSwitch());
    jontCurrentPosEntry.setDouble(getPos());
    jointTargetPosEntry.setDouble(target);
    jointInitStateEntry.setString(curStates.toString());
    jointLevelStateEntry.setString(curPlacement.toString());
    grabberIntakeOuttakeEntry.setString(intakeOuttake.toString());
    SmartDashboard.putNumber("Output",getOutputCurrent() );

    switch (curStates) {
      case NOT_INITIALIZED:
        break;
      case INITIALIZING:
        break;
      case INITIALIZED:
        setState(States.ENCODER);
      case ENCODER:
        switch (intakeOuttake) {
          case NOTHING -> {
            maxLeader.set(0);}
          case INTAKE -> {
              maxLeader.set(-0.4);
              if (getOutputCurrent() >= Constants.GrabberConstants.CURRENT_LIMIT) {
                  
                  // System.out.println("here" + getOutputCurrent());
                  Ticker(0.5, false);
                  maxLeader.set(-0.3);
              }
            }

          case OUTTAKE -> {
            if(curPlacement==GrabberPlacement.L1){
              maxLeader.set(0.3);
              Ticker(1, true);
            }
            else{
              maxLeader.set(0.3);
              Ticker(1, true);
            }
              
                // maxLeader.set(0.2);
            }

          default -> maxLeader.set(0);

        }
        break;


    }

  }
}