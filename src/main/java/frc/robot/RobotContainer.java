// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import frc.robot.commands.ArcadeDrive;
import frc.robot.commands.Climb;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  public static final Drivetrain drivetrain = Drivetrain.getInstance(); //creates drivetrain
  public static final Intake intake = Intake.getInstance(); //creates intake
  public static final Arm arm = Arm.getInstance(); //creates arm

  public static final Joystick joystick = new Joystick(0); //creates joystick
  public static final Joystick joystick2 = new Joystick(1);
  public final JoystickButton button1 = new JoystickButton(joystick2, 1); //creates joystick buttons
  public final JoystickButton button2 = new JoystickButton(joystick2, 2);
  public final JoystickButton button3 = new JoystickButton(joystick2, 3);
  public final JoystickButton button9 = new JoystickButton(joystick2, 9);
  public final JoystickButton button10 = new JoystickButton(joystick2, 10);
  public final JoystickButton button11 = new JoystickButton(joystick, 11);
  public final JoystickButton button12 = new JoystickButton(joystick2, 12);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
    drivetrain.setDefaultCommand(new ArcadeDrive()); //sets default command of drivetrain to arcadedrive
    arm.setDefaultCommand(new RunCommand(arm::ArmHold, arm));
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    button2.whenPressed(new InstantCommand(intake::IntakeIn, intake));
    button1.whenPressed(new InstantCommand(intake::IntakeOut, intake));
    button3.whenPressed(new InstantCommand(intake::IntakeStop, intake));
    button10.whenPressed(new InstantCommand(arm::ArmUp, arm));
    button9.whenPressed(new InstantCommand(arm::ArmDown, arm));
    button11.whileHeld(new Climb());
    //dfghjk
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    var autoVoltageConstraint =
    new DifferentialDriveVoltageConstraint(
        new SimpleMotorFeedforward(
            Constants.DrivetrainConstants.kS,
            Constants.DrivetrainConstants.kV,
            Constants.DrivetrainConstants.kA),
            Constants.DrivetrainConstants.kinematics,
            8);

    TrajectoryConfig config =
    new TrajectoryConfig(Constants.DrivetrainConstants.kMaxV,Constants.DrivetrainConstants.kMaxA)
          // Add kinematics to ensure max speed is actually obeyed
          .setKinematics(Constants.DrivetrainConstants.kinematics)
          // Apply the voltage constraint
          .addConstraint(autoVoltageConstraint);
    
    Trajectory exampleTrajectory =
        TrajectoryGenerator.generateTrajectory(
            // Start at the origin facing the +X direction
            new Pose2d(0, 0, new Rotation2d(0)),
            // Pass through these two interior waypoints, making an 's' curve path
            List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
            // End 3 meters straight ahead of where we started, facing forward
            new Pose2d(3, 0, new Rotation2d(0)),
            // Pass config
            config);

    drivetrain.zeroHeading();
    drivetrain.resetOdometry(new Pose2d(0, 0, new Rotation2d(0)));
    
    RamseteCommand ramseteCommand =
        new RamseteCommand(
            exampleTrajectory,
            drivetrain::getPose,
            new RamseteController(Constants.DrivetrainConstants.kRamseteB, Constants.DrivetrainConstants.kRamseteZeta),
            new SimpleMotorFeedforward(
              Constants.DrivetrainConstants.kS,
              Constants.DrivetrainConstants.kV,
              Constants.DrivetrainConstants.kA),
            Constants.DrivetrainConstants.kinematics,
            drivetrain::getWheelSpeeds,
            new PIDController(Constants.DrivetrainConstants.kP, 0, 0),
            new PIDController(Constants.DrivetrainConstants.kP, 0, 0),
            // RamseteCommand passes volts to the callback
            drivetrain::tankDriveVolts,
            drivetrain);

    return ramseteCommand.andThen(() -> drivetrain.tankDriveVolts(0, 0));
  }
}