// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team5959;


// Import statements for various WPILib classes and custom classes used in the robot code.
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;



import com.team5959.Constants.ControllerConstants;
import com.team5959.subsystems.SwerveChassis;
import com.team5959.commands.AutoFollowTrajectoryCmd;
import com.team5959.commands.SwerveDriveJoystickCmd;
import com.team5959.commands.SwerveDriveXLockCmd;

import edu.wpi.first.wpilibj.PS4Controller;


public class RobotContainer {
  // Creacion de objetos de SUBSISTEMAS 
  private final SwerveChassis swerveChassis = new SwerveChassis(); 

  // Creacion de objetos de CONTROLES
  private final PS4Controller control = new PS4Controller(ControllerConstants.kDriverControllerPort);

  // Creacion de objetos de BOTONES para asignar nombres claros 
  private final JoystickButton resetPosButton = new JoystickButton(control, 9);
  private final JoystickButton resetNavxButton = new JoystickButton(control, 10);
  private final JoystickButton lockPositionButton = new JoystickButton(control, 14);
 

  
   
  public RobotContainer() {

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

    resetNavxButton.onTrue(new InstantCommand(() -> {swerveChassis.resetNavx();swerveChassis.resetHeadingHoldAfterGyroReset();}));
    resetPosButton.onTrue(new InstantCommand(() -> {
      // 1. Resetear navX primero
      swerveChassis.resetNavx();
      swerveChassis.resetHeadingHoldAfterGyroReset();
  
      // 2. Ahora que el gyro está a 0, usar esa rotación para odometría
      swerveChassis.resetOdometry(new Pose2d(0, 0, swerveChassis.getRotation2d()));
  
      // 3. Resetear encoders de los módulos
      swerveChassis.resetDriveEncoders();
  }, swerveChassis));
  
    lockPositionButton.whileTrue(new SwerveDriveXLockCmd(swerveChassis));
    

  
  }
  
  public void periodic(){
        
  }
  
  public Command getAutonomousCommand() {
    

        return new AutoFollowTrajectoryCmd(swerveChassis);
    
  }
}
