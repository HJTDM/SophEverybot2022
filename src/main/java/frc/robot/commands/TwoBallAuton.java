// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

public class TwoBallAuton extends CommandBase {
  double initTime;
  /** Creates a new TestAuton. */
  public TwoBallAuton() {
    addRequirements(RobotContainer.drivetrain, RobotContainer.intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    initTime = Timer.getFPGATimestamp();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(Timer.getFPGATimestamp() - initTime > 1 && Timer.getFPGATimestamp() - initTime < 2){
      RobotContainer.intake.AutonIntakeIn();
      RobotContainer.drivetrain.arcadeDrive(0.66, 0);
    }
    if(Timer.getFPGATimestamp() - initTime > 2){
      RobotContainer.drivetrain.arcadeDrive(0, 0);
    }
    if(Timer.getFPGATimestamp() - initTime > 3){
      RobotContainer.intake.IntakeStop();
      RobotContainer.arm.ArmUp();
    }
    if(Timer.getFPGATimestamp() - initTime > 3 && Timer.getFPGATimestamp() - initTime < 4.3){
      RobotContainer.drivetrain.arcadeDrive(-0.66, -0.35);
    }
    if(Timer.getFPGATimestamp() - initTime > 4.3){
      RobotContainer.drivetrain.arcadeDrive(0, 0);
    }
    if(Timer.getFPGATimestamp() - initTime > 5 && Timer.getFPGATimestamp() - initTime < 8.2){
      RobotContainer.drivetrain.arcadeDrive(0, 0.6);
    }
    if(Timer.getFPGATimestamp() - initTime > 8.2){
      RobotContainer.drivetrain.arcadeDrive(0, 0);
    }
    if(Timer.getFPGATimestamp() - initTime > 8.2 && Timer.getFPGATimestamp() - initTime < 9.2){
      RobotContainer.drivetrain.arcadeDrive(0.5, 0);
    }
    if(Timer.getFPGATimestamp() - initTime > 9.2 && Timer.getFPGATimestamp() - initTime < 10.2){
      RobotContainer.intake.AutonIntakeOut();
    }
    if(Timer.getFPGATimestamp() - initTime > 10.2){
      RobotContainer.intake.IntakeStop();
    }
    if(Timer.getFPGATimestamp() - initTime > 13 && Timer.getFPGATimestamp() - initTime < 14.5){
      RobotContainer.drivetrain.arcadeDrive(-0.7, 0);
    }
    if(Timer.getFPGATimestamp() - initTime > 14.5){
      RobotContainer.drivetrain.arcadeDrive(0, 0);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
