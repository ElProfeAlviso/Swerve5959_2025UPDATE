// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team5959;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
// Import statements for various WPILib classes and custom classes used in the robot code.
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import java.util.List;

import com.team5959.Constants.ControllerConstants;
import com.team5959.Constants.SwerveConstants;
import com.team5959.subsystems.SwerveChassis;
import com.team5959.commands.SwerveDriveJoystickCmd;
import edu.wpi.first.wpilibj.PS4Controller;


public class RobotContainer {
  // Creacion de objetos de SUBSISTEMAS 
  private final SwerveChassis swerveChassis = new SwerveChassis(); 

  // Creacion de objetos de CONTROLES
  private final PS4Controller control = new PS4Controller(ControllerConstants.kDriverControllerPort);

  // Creacion de objetos de BOTONES para asignar nombres claros 
  private final JoystickButton resetPosButton = new JoystickButton(control, 9);
  private final JoystickButton resetNavxButton = new JoystickButton(control, 10); 
  
   
  public RobotContainer() {

    TrajectoryConfig tConfig = new TrajectoryConfig(2,1).setKinematics(SwerveConstants.DRIVE_KINEMATICS);

    Trajectory robotTrajectory = TrajectoryGenerator.generateTrajectory(
      new Pose2d(0, 0, Rotation2d.fromDegrees(0)),
      List.of(new Translation2d(2, 4),
             new Translation2d(4, 3)),
      new Pose2d(6, 2, Rotation2d.fromDegrees(0)),
       tConfig);

    Trajectory robotTrajectory2 = TrajectoryGenerator.generateTrajectory(
      new Pose2d(0, 0, Rotation2d.fromDegrees(0)),
      List.of(new Translation2d(2, 2),
               new Translation2d(4, 0)),
      new Pose2d(6, 6, Rotation2d.fromDegrees(0)), 
      tConfig);

    // Configurar los comandos predeterminados de los subsistemas. En este caso, el chasis swerve
       swerveChassis.setDefaultCommand(new SwerveDriveJoystickCmd(swerveChassis,
        () -> control.getLeftY(), 
        () -> control.getLeftX(), 
        () -> control.getRightX(), 
        true));
   
       // Configure the trigger bindings method.
    configureBindings();
  }

  // Configurar los enlaces de botones para los comandos usando lambdas o referencias de método
  private void configureBindings() {

    resetNavxButton.onTrue(new InstantCommand(() -> {swerveChassis.resetNavx();swerveChassis.resetDriveEncoders();}));
    resetPosButton.onTrue(new InstantCommand(() -> swerveChassis.resetOdometry(new Pose2d(0, 0, new Rotation2d(0)))));
  
  }
  
  public void periodic(){
        
  }
  
  public Command getAutonomousCommand() {

    swerveChassis.resetNavx();

    

    // 1. Create trajectory settings
        TrajectoryConfig trajectoryConfig = new TrajectoryConfig(
                SwerveConstants.MAX_AUTO_SPEED,
                SwerveConstants.MAX_AUTO_ACCELERATION)
                        .setKinematics(SwerveConstants.DRIVE_KINEMATICS);

        // 2. Generate trajectory
        Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
                new Pose2d(0, 0, new Rotation2d(0)),
                List.of(
                        new Translation2d(1, 2),
                        new Translation2d(2, 0)),
                new Pose2d(3, 3, Rotation2d.fromDegrees(180)),
                trajectoryConfig);

        // 3. Define PID controllers for tracking trajectory
        PIDController xController = new PIDController(SwerveConstants.KP_AUTO_XController, 0, 0);
        PIDController yController = new PIDController(SwerveConstants.KP_AUTO_YController, 0, 0);
        ProfiledPIDController thetaController = new ProfiledPIDController(
                SwerveConstants.KP_AUTO_ROTATION, 0, 0, SwerveConstants.kThetaControllerConstraints);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        // 4. Construct command to follow trajectory
        SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
                trajectory,
                swerveChassis::getPose,
                SwerveConstants.DRIVE_KINEMATICS,
                xController,
                yController,
                thetaController,
                swerveChassis::setModuleStates,
                swerveChassis);

                swerveChassis.publishTrajectory("trajectory", trajectory);

        // 5. Add some init and wrap-up, and return everything
        return new SequentialCommandGroup(
                new InstantCommand(() -> swerveChassis.resetOdometry(trajectory.getInitialPose())),
                swerveControllerCommand,
                new InstantCommand(() -> swerveChassis.stopModules()));
    // An example command will be run in autonomous
    
  }
}
