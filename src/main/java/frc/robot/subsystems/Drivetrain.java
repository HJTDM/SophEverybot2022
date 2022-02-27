//SUBSYSTEM

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Drivetrain extends SubsystemBase {

  CANSparkMax leftFront = new CANSparkMax(13, MotorType.kBrushless); //creates drivetrain motors
  CANSparkMax rightFront = new CANSparkMax(11, MotorType.kBrushless);
  CANSparkMax leftBack = new CANSparkMax(12, MotorType.kBrushless);
  CANSparkMax rightBack = new CANSparkMax(10, MotorType.kBrushless);

  MotorControllerGroup leftMotors = new MotorControllerGroup(leftFront, leftBack);
  MotorControllerGroup rightMotors = new MotorControllerGroup(rightFront, rightBack);

  public RelativeEncoder leftEncoder = leftFront.getEncoder();
  public RelativeEncoder rightEncoder = rightFront.getEncoder();

  public ADIS16470_IMU gyro = new ADIS16470_IMU();

  private DifferentialDriveOdometry odometry = new DifferentialDriveOdometry(new Rotation2d(0));

  private static final Drivetrain drivetrain = new Drivetrain();
  public static Drivetrain getInstance(){
    return drivetrain;
  }

  public Drivetrain() {
    formatMotors();
    resetEncoders();
    zeroHeading();
  }

  private void formatMotors() {
    leftFront.restoreFactoryDefaults();
    leftBack.restoreFactoryDefaults();
    rightFront.restoreFactoryDefaults();
    rightBack.restoreFactoryDefaults();
    leftFront.setSmartCurrentLimit(50);
    leftBack.setSmartCurrentLimit(50);
    rightFront.setSmartCurrentLimit(50);
    rightBack.setSmartCurrentLimit(50);
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

    throttle *= Math.abs(throttle); //cubes throttle input
    twist *= Math.abs(twist); //4ths twist input
    setSpeed(throttle+twist, throttle-twist); //sets speed of left and right motor
    
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
    setSpeed(throttle+twist, throttle-twist); //sets speed of left and right motor

    SmartDashboard.putNumber("Throttle", throttle);
    SmartDashboard.putNumber("Twist", twist);
    SmartDashboard.putNumber("Left Motors", throttle + twist);
    SmartDashboard.putNumber("Right Motors", throttle - twist);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    odometry.update(
      new Rotation2d(gyro.getAngle()), getLeftEncoderPosition(), getRightEncoderPosition());
    SmartDashboard.putNumber("left front", getLeftEncoderPosition());
    SmartDashboard.putNumber("right front", getRightEncoderPosition());
    SmartDashboard.putNumber("angle", gyro.getAngle());
    SmartDashboard.putNumber("left motor current", leftFront.getOutputCurrent());
    SmartDashboard.putNumber("right motor current", rightFront.getOutputCurrent());
  }

  public Pose2d getPose(){
    return odometry.getPoseMeters();
  }

  public DifferentialDriveWheelSpeeds getWheelSpeeds(){
    return new DifferentialDriveWheelSpeeds(getLeftEncoderVelocity(), getRightEncoderVelocity());
  }
  public void resetOdometry(Pose2d pose){
    resetEncoders();
    zeroHeading();
    odometry.resetPosition(pose, new Rotation2d(gyro.getAngle()));
  }

  public void tankDriveVolts(double leftVolts, double rightVolts) {
    leftMotors.setVoltage(leftVolts);
    rightMotors.setVoltage(rightVolts);
  }

  public void resetEncoders() {
    leftEncoder.setPosition(0);
    rightEncoder.setPosition(0);
  }

  public RelativeEncoder getLeftEncoder(){
    return leftEncoder;
  }

  public RelativeEncoder getrightEncoder(){
    return rightEncoder;
  }

  public double getLeftEncoderPosition(){
    return leftEncoder.getPosition()*Constants.DrivetrainConstants.wheelC/Constants.DrivetrainConstants.gearRatio;
  }

  public double getRightEncoderPosition(){
    return rightEncoder.getPosition()*Constants.DrivetrainConstants.wheelC/Constants.DrivetrainConstants.gearRatio;
  }

  public double getLeftEncoderVelocity(){
    return leftEncoder.getVelocity()*Constants.DrivetrainConstants.wheelC/(Constants.DrivetrainConstants.gearRatio*60);
  }

  public double getRightEncoderVelocity(){
    return rightEncoder.getVelocity()*Constants.DrivetrainConstants.wheelC/(Constants.DrivetrainConstants.gearRatio*60);
  }

  public void zeroHeading() {
    gyro.reset();
  }

  public double getHeading() {
    return gyro.getAngle();
  }

  public double getTurnRate() {
    return gyro.getRate();
  }

  
}
