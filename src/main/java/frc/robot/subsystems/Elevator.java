// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;


public class Elevator extends SubsystemBase {
  TalonFX masterMotor = new TalonFX(Constants.ElevatorConstants.LEADER_MOTOR_PORT);
  TalonFX followerMotor = new TalonFX(Constants.ElevatorConstants.FOLLOWER_MOTOR_PORT);
  DigitalInput limitSwitch = new DigitalInput(Constants.ElevatorConstants.LIMITSWITCH_PORT);

  

  public static enum stateLevel {
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
  public enum stateReset {
    NOT_INITIALIZED,
    INITIALIZING,
    INITIALIZED
  }

  static stateReset currentState = stateReset.INITIALIZING;
  static stateLevel currentLevel = stateLevel.REST;
  static double currentPos;

  GenericEntry elevatorLimitswitchEntry = Shuffleboard.getTab("Elevator").add("Elevatorlimitswitch",limitSwitch.get()).getEntry();
  GenericEntry elevatorCurrentPosEntry = Shuffleboard.getTab("Elevator").add("Elevator Current Positon", 0).getEntry();
  GenericEntry elevatorTargetPosEntry = Shuffleboard.getTab("Elevator").add("Elevator Target Position", 0).getEntry();
  GenericEntry elevatorCurrentStateInitEntry = Shuffleboard.getTab("Elevator").add("Elevator State Init", currentState.toString()).getEntry();
  GenericEntry elevatorCurrentStateLevelEntry = Shuffleboard.getTab("Elevator").add("Elevator State Level", currentLevel.toString()).getEntry();

  TalonFXConfiguration configs = new TalonFXConfiguration();
  public Elevator() {
    
    // This TalonFX should be configured with a kP of 1, a kI of 0, a kD of 10, and a kV of 2 on slot 0
    configs.Slot0.kP = 1;
    configs.Slot0.kI = 0;
    configs.Slot0.kD = 0;
    
    // Set the position to 0 rotations for initial use
    masterMotor.setPosition(0);
    followerMotor.setControl(new Follower(13, true));
    
    // Get Position and Velocity
    var position = masterMotor.getPosition();
    var velocity = masterMotor.getVelocity();
    
    //set motor acceleration for motion magic
    configs.MotionMagic.MotionMagicAcceleration = 5;
    configs.MotionMagic.MotionMagicCruiseVelocity = 5;

    // Write these configs to the TalonFX
    masterMotor.getConfigurator().apply(configs);
  }

  public void slowMode(){
    configs.MotionMagic.MotionMagicAcceleration = 1;
    configs.MotionMagic.MotionMagicCruiseVelocity = 1;
    masterMotor.getConfigurator().refresh(configs);
  }

  public void normalMode(){
    configs.MotionMagic.MotionMagicAcceleration = 5;
    configs.MotionMagic.MotionMagicCruiseVelocity = 5;
    masterMotor.getConfigurator().refresh(configs);
  }

  public void movePosition(double pos) {
    currentPos = pos;
    if (getPosition() <= Constants.ElevatorConstants.BOTTOM_HARD_LIMIT || getPosition() >= Constants.ElevatorConstants.TOP_HARD_LIMIT) {
      masterMotor.disable();
    } else if (getPosition() < Constants.ElevatorConstants.BOTTOM_SOFT_LIMIT) {
      currentPos = Constants.ElevatorConstants.BOTTOM_SOFT_LIMIT+10;
    } else if (getPosition() > Constants.ElevatorConstants.TOP_SOFT_LIMIT) {
      currentPos = Constants.ElevatorConstants.TOP_SOFT_LIMIT-10;    
    } 
    masterMotor.setControl(new MotionMagicVoltage(currentPos));
  }

  public void setPosition(double position) {
    masterMotor.setPosition(position);
  }

  public double getPosition() {
    return masterMotor.getPosition().getValueAsDouble();
  }

  public stateLevel moveWithEncoder(double axisVal) {
    // System.out.println("encoder");
    if (axisVal <= -0.75 && axisVal >= -1) {
      return stateLevel.L1;
    } else if (axisVal <= 0 && axisVal >= -0.75) {
      return stateLevel.L2;
    } else if (axisVal <= 0.75 && axisVal >= 1) {
      return stateLevel.L3;
    } else {
      return stateLevel.L4;
    }
  }

  public boolean getLimitSwitch() {
    return limitSwitch.get();
  }

  public void disable() {
    masterMotor.disable();
  }

  public void changeState(stateReset state) {
    currentState = state;
  }

  public void setLevel(stateLevel level) {
    currentLevel = level;
  }

  public stateReset getState() {
    return currentState;
  }

  public stateLevel getLevel() {
    return currentLevel;
  }

  

  @Override
  public void periodic() {
    elevatorLimitswitchEntry.setBoolean(getLimitSwitch());
    elevatorCurrentPosEntry.setDouble(getPosition());
    elevatorTargetPosEntry.setDouble(currentPos);
    elevatorCurrentStateInitEntry.setString(currentState.toString());
    elevatorCurrentStateLevelEntry.setString(currentLevel.toString());

    System.out.println(configs);

    switch (currentState) {
      case NOT_INITIALIZED:
        break;
      case INITIALIZING:
        break;
      case INITIALIZED:     
      default:
        break;
    }
    // This method will be called once per scheduler run
    //moveWithEncoder(RobotContainer.driver.getRawAxis(0));
  }
}
