// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
//import com.revrobotics.RelativeEncoder;
//import com.revrobotics.SparkMaxPIDController;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
//import frc.robot.Constants;
import frc.robot.Constants;


public class Arm extends SubsystemBase {
  /** Creates a new Arm. */
  CANSparkMax armMotor = new CANSparkMax(21, MotorType.kBrushless);

  boolean armIsUp;
  double lastBurstTime = -1.0;

  private static final Arm arm = new Arm();
  public static Arm getInstance(){
    return arm;
  }
  //public RelativeEncoder armEncoder = armMotor.getEncoder();
  //SparkMaxPIDController pid = armMotor.getPIDController();
    
  public Arm() {
    //pid.setP(Constants.P);
    //pid.setOutputRange(-0.2, 0.2);
    armMotor.setIdleMode(IdleMode.kBrake);

  }

  public void setArmMode(boolean armIsUp){
    this.armIsUp = armIsUp;
  }

  public void ArmUp(){
    if (armIsUp == false) {
      lastBurstTime = Timer.getFPGATimestamp();
      armIsUp = true;
    } 
  }

  public void ArmDown(){
    if (armIsUp == true) {
      lastBurstTime = Timer.getFPGATimestamp();
      armIsUp = false;
    } 
  }

  public void ArmHold(){
    if (armIsUp) {
      if (Timer.getFPGATimestamp() - lastBurstTime < Constants.armTimeUp) {
        armMotor.set(Constants.ArmConstants.armTravelUp);
      }
      else{
        armMotor.set(Constants.ArmConstants.armHoldUp);
      }
    } 
    else if (!armIsUp) {
      if (Timer.getFPGATimestamp() - lastBurstTime < Constants.armTimeDown) {
        armMotor.set(Constants.ArmConstants.armTravelDown);
      }
      else{
        armMotor.set(-Constants.ArmConstants.armHoldDown);
      }
    }
    SmartDashboard.putBoolean("Arm is Up?", armIsUp);
  }
/*  public void setPosition(double position) {
    //pid.setReference(position, CANSparkMax.ControlType.kPosition);
  }

  public void setBrake(IdleMode brakeState) {
    armMotor.setIdleMode(brakeState);

  }

  public void armStop(){
    armMotor.stopMotor();

  }
*/
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
