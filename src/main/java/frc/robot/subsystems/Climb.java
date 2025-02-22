// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class Climb extends SubsystemBase {


  public enum states {
    NOT_INTIALIZED,
    INTIALIZING,
    INTIALIZED,
    BUTTON_CLICKED_ACTIVATE,
    ClimbConstants_ENDED
  }


  public states current_state = states.INTIALIZING;
  /** Creates a new ClimbConstants. */
  TalonFX motor = new TalonFX(Constants.ClimbConstants.MOTOR_PORT);
  TalonFXConfiguration config = new TalonFXConfiguration();
  public DigitalInput limitswitch1 = new DigitalInput(Constants.ClimbConstants.LIMITSWITCH_PORT);
  double targetPos = 0;
  public Climb() {



    var slot0Configs = config.Slot0; 
    slot0Configs.kP = 1;
    slot0Configs.kI = 0;
    slot0Configs.kD = 0;
    //slot0Configs.kG = -1.152;

    config.MotionMagic.MotionMagicAcceleration = 2;
    config.MotionMagic.MotionMagicCruiseVelocity = 2;
    config.CurrentLimits.SupplyCurrentLimit = 2;
    

    motor.getConfigurator().apply(config);
    motor.setNeutralMode(NeutralModeValue.Brake);
    

  }
//hi
  public void move_motor(double pos) {
    if (current_state == states.INTIALIZED) {
      if (get_pos() < Constants.ClimbConstants.BOTTOM_HARD_LIMIT|| get_pos() > Constants.ClimbConstants.TOP_HARD_LIMIT) {
        motor.stopMotor();
      }
      else if (get_pos() < Constants.ClimbConstants.BOTTOM_SOFT_LIMIT) {
        targetPos = Constants.ClimbConstants.BOTTOM_SOFT_LIMIT - 10;
      }
      else if (get_pos() > Constants.ClimbConstants.TOP_SOFT_LIMIT) {
       targetPos = Constants.ClimbConstants.TOP_SOFT_LIMIT +10;
      }
      
    }
    System.out.println(String.valueOf(pos));
    motor.setControl(new MotionMagicVoltage(pos));
  }

  public void set_state(states target_state) {
    current_state = target_state;
  }

  public boolean getLimitSwitch(){
    return limitswitch1.get();
  }

  public void disable_motor() {
    motor.disable();
  }

  public void set_pos() {
    motor.setPosition(0);
  }
  public states getState() {
    return current_state;
  }

  public double get_pos() {
    return motor.getPosition().getValueAsDouble();
  }

  public double get_current() {
    return motor.getStatorCurrent().getValueAsDouble();
  }




  @Override
  public void periodic() {
    SmartDashboard.putNumber("currentpos", get_pos());
    SmartDashboard.putBoolean("huai te shih", getLimitSwitch());
    SmartDashboard.putNumber("outputcurrent", get_current());
    SmartDashboard.putString("state", current_state.toString());
    SmartDashboard.putNumber("target Pos", targetPos);
    // This method will be called once per scheduler run
    switch (current_state) {
      case NOT_INTIALIZED:
        break;
      case INTIALIZING:
        break;
      case INTIALIZED:
        break;
      case BUTTON_CLICKED_ACTIVATE:
        move_motor(Constants.ClimbConstants.MOVE_TO_CAGE_POSITION);
        targetPos = Constants.ClimbConstants.MOVE_TO_CAGE_POSITION;
        if(get_current() >= Constants.ClimbConstants.CURRENT_LIMIT){
          motor.stopMotor();
        }
        current_state = states.ClimbConstants_ENDED;
      case ClimbConstants_ENDED:
        break;
      default: 


    }
  }
}