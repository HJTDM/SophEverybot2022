//SUBSYSTEM

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Drivetrain extends SubsystemBase {

  CANSparkMax leftFront = new CANSparkMax(13, MotorType.kBrushless); //creates drivetrain motors
  CANSparkMax rightFront = new CANSparkMax(11, MotorType.kBrushless);
  CANSparkMax leftBack = new CANSparkMax(12, MotorType.kBrushless);
  CANSparkMax rightBack = new CANSparkMax(10, MotorType.kBrushless);

  public RelativeEncoder leftEncoder = leftFront.getEncoder();
  public RelativeEncoder rightEncoder = rightFront.getEncoder();

  private static final Drivetrain drivetrain = new Drivetrain();
  public static Drivetrain getInstance(){
    return drivetrain;
  }

  public Drivetrain() {
    formatMotors();
  }

  private void formatMotors(){
    leftFront.restoreFactoryDefaults();
    leftBack.restoreFactoryDefaults();
    rightFront.restoreFactoryDefaults();
    rightBack.restoreFactoryDefaults();
    leftFront.setSmartCurrentLimit(80);
    leftBack.setSmartCurrentLimit(80);
    rightFront.setSmartCurrentLimit(80);
    rightBack.setSmartCurrentLimit(80);
    leftFront.setInverted(false); //inverts motors to right direction (front and back)
    rightFront.setInverted(true);
    leftBack.follow(leftFront); //back motor follows the front
    rightBack.follow(rightFront);
    leftFront.burnFlash();
    rightFront.burnFlash();
    leftBack.burnFlash();
    rightBack.burnFlash();
  }

  public void setBrakeMode(boolean set){
    if(set){
      leftFront.setIdleMode(IdleMode.kBrake);
      leftBack.setIdleMode(IdleMode.kCoast);
      rightFront.setIdleMode(IdleMode.kBrake);
      rightBack.setIdleMode(IdleMode.kCoast);
    }
    else{
      leftFront.setIdleMode(IdleMode.kCoast);
      leftBack.setIdleMode(IdleMode.kCoast);
      rightFront.setIdleMode(IdleMode.kCoast);
      rightBack.setIdleMode(IdleMode.kCoast);
    }
  }
/*
  public void getIdleMode(){
      if(leftFront.getIdleMode() == IdleMode.kBrake){
        SmartDashboard.putString("Left Front", "Brake");
      }
      else{
        SmartDashboard.putString("Left Front", "Coast");
      }
      if(leftBack.getIdleMode() == IdleMode.kBrake){
        SmartDashboard.putString("Left Back", "Brake");
      }
      else{
        SmartDashboard.putString("Left Back", "Coast");
      }
      if(rightFront.getIdleMode() == IdleMode.kBrake){
        SmartDashboard.putString("Right Front", "Brake");
      }
      else{
        SmartDashboard.putString("Right Front", "Coast");
      }
      if(rightBack.getIdleMode() == IdleMode.kBrake){
        SmartDashboard.putString("Right Back", "Brake");
      }
      else{
        SmartDashboard.putString("Right Back", "Coast");
      }
  }
*/
  public void setSpeed(double leftSpeed, double rightSpeed){
    leftFront.set(leftSpeed); //sets the left front motor speed
    rightFront.set(rightSpeed); //sets the right front motor speed

    SmartDashboard.putNumber("Left Motors", leftFront.get());
    SmartDashboard.putNumber("Right Motors", rightFront.get());
  }

  public void arcadeDrive(double throttle, double twist){
    //creates deadband - if joystick input on either axis is less than 0.1 just set to 0
    if(Math.abs(throttle) < 0.1){
      throttle = 0;
    }
    if(Math.abs(twist) < 0.1){
      twist = 0;
    }

    throttle *= Math.abs(throttle) * 0.8; //cubes throttle input
    twist *= Math.abs(twist) * 0.6; //4ths twist input
    setSpeed(throttle + twist, throttle - twist); //sets speed of left and right motor
    
    SmartDashboard.putNumber("Throttle", throttle);
    SmartDashboard.putNumber("Twist", twist);
    SmartDashboard.putNumber("Left Motors", throttle + twist);
    SmartDashboard.putNumber("Right Motors", throttle - twist);
  }


  public double clamp(double min, double max, double value) {
    double clampedValue = Math.max(min, Math.min(value, max));
    return clampedValue;

  }

  public void funkyDrive(double throttle, double twist){
    //creates deadband - if joystick input on either axis is less than 0.1 just set to 0
    if(Math.abs(throttle) < 0.1){
      throttle = 0;
    }
    if(Math.abs(twist) < 0.1){
      twist = 0;
    }

    double right = throttle * (clamp(-1,1, (1 - 2*(twist))));  
    double left = throttle * (clamp(-1,1, (1 + 2*(twist))));  


    if (throttle < 0) {
      setSpeed(right, left);
    } 
    else {
      setSpeed(left, right);
    }
    
    SmartDashboard.putNumber("Throttle", throttle);
    SmartDashboard.putNumber("Twist", twist);
    SmartDashboard.putNumber("Left Motors", left);
    SmartDashboard.putNumber("Right Motors", right);
  }

  public void autonArcadeDrive(double throttle, double twist){
    setSpeed(throttle + twist, throttle - twist); //sets speed of left and right motor

    SmartDashboard.putNumber("Throttle", throttle);
    SmartDashboard.putNumber("Twist", twist);
    SmartDashboard.putNumber("Left Motors", throttle + twist);
    SmartDashboard.putNumber("Right Motors", throttle - twist);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
