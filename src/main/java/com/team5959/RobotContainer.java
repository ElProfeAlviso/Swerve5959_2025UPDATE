// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team5959;

// Import statements for various WPILib classes and custom classes used in the robot code.
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.events.EventTrigger;
import com.pathplanner.lib.events.PointTowardsZoneTrigger;

import com.team5959.Constants.ControllerConstants;
import com.team5959.subsystems.SwerveChassis;

import com.team5959.commands.SwerveDriveJoystickCmd;
import com.team5959.commands.SwerveDriveXLockCmd;
import edu.wpi.first.wpilibj.PS4Controller;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class RobotContainer {

  // Selector de comando autónomo
  private final SendableChooser<Command> autoChooser;
  // Creacion de objetos de SUBSISTEMAS
  private final SwerveChassis swerveChassis = new SwerveChassis();

  // Creacion de objetos de CONTROLES
  private final PS4Controller control = new PS4Controller(ControllerConstants.kDriverControllerPort);

  // Creacion de objetos de BOTONES para asignar nombres claros
  private final JoystickButton resetPosButton = new JoystickButton(control, 9);
  private final JoystickButton resetNavxButton = new JoystickButton(control, 10);
  private final JoystickButton lockPositionButton = new JoystickButton(control, 14);

  public RobotContainer() {

    // Registro de comandos nombrados para pathplanner
    NamedCommands.registerCommand("runIntakeCmd", Commands.print("Enciendo intake"));
    NamedCommands.registerCommand("offtakeCmd", Commands.print("Apagando Intake"));
    NamedCommands.registerCommand("scorereef", Commands.runOnce(() -> {
      System.out.println("Anotando en Reef");
    }));
    NamedCommands.registerCommand("getcoral", Commands.runOnce(() -> {
      System.out.println("Solicitando coral");
    }));

    // Registro de triggers de pathplanner
    new EventTrigger("Prepareforscore").onTrue(Commands.runOnce(() -> {
      System.out.println("Preparando Intake event");
    }));
    // Registro de zonas de enfoque
    new PointTowardsZoneTrigger("Reef").whileTrue(Commands.print("Apuntando a Reef"));

    // Crear un comando PathPlannerAuto usando un archivo de ruta guardado llamado
    // "compAuto3"
    PathPlannerAuto compAuto3Command = new PathPlannerAuto("compAuto3");
    // PathPlannerAuto can also be created with a custom command
    // autoCommand = new PathPlannerAuto(new CustomAutoCommand());

    // Bind to different auto triggers
    compAuto3Command.isRunning().onTrue(Commands.print("Autonomo 3 corriendo"));
    compAuto3Command.timeElapsed(2).onTrue(Commands.print("Han pasado 2 segundos"));
    compAuto3Command.timeRange(2, 4).whileTrue(Commands.print("entre 2 y 4 segundos"));
    compAuto3Command.event("Prepareforscore").onTrue(Commands.print("Pasando por el evento"));
    compAuto3Command.pointTowardsZone("Speaker").onTrue(Commands.print("Viendo al Speaker"));
    compAuto3Command.activePath("Azul4").onTrue(Commands.print("Iniciando path 4"));
    compAuto3Command.nearFieldPosition(new Translation2d(2, 2), 0.5).whileTrue(Commands.print("within 0.5m of (2, 2)"));
    compAuto3Command.inFieldArea(new Translation2d(2, 2), new Translation2d(4, 4))
        .whileTrue(Commands.print("in area of (2, 2) - (4, 4)"));

    // Do all other initialization

    // Build an auto chooser. This will use Commands.none() as the default option.
    // autoChooser = AutoBuilder.buildAutoChooser();

    // For convenience a programmer could change this when going to competition.
    boolean isCompetition = true;

    // Build an auto chooser. This will use Commands.none() as the default option.
    // As an example, this will only show autos that start with "comp" while at
    // competition as defined by the programmer
    autoChooser = AutoBuilder.buildAutoChooserWithOptionsModifier(
        (stream) -> isCompetition
            ? stream.filter(auto -> auto.getName().startsWith("comp"))
            : stream);

    SmartDashboard.putData("Auto Chooser", autoChooser);
    SmartDashboard.putData("Command Scheduler", edu.wpi.first.wpilibj2.command.CommandScheduler.getInstance());

    // Configurar los comandos predeterminados de los subsistemas. En este caso, el
    // chasis swerve
    swerveChassis.setDefaultCommand(new SwerveDriveJoystickCmd(swerveChassis,
        () -> control.getLeftY(),
        () -> control.getLeftX(),
        () -> control.getRightX(),
        true));

    // Configure the trigger bindings method.
    configureBindings();
  }

  public SwerveChassis getSwerveChassis() {
    return swerveChassis;
  }

  // Configurar los enlaces de botones para los comandos usando lambdas o
  // referencias de método
  private void configureBindings() {

    resetNavxButton.onTrue(new InstantCommand(() -> {
      swerveChassis.resetNavx();
      swerveChassis.resetHeadingHoldAfterGyroReset();
    }));
    resetPosButton.onTrue(new InstantCommand(() -> {
      // 1. Resetear navX primero
      // swerveChassis.resetNavx();
      swerveChassis.resetHeadingHoldAfterGyroReset();

      // 2. Ahora que el gyro está a 0, usar esa rotación para odometría
      swerveChassis.resetOdometry(new Pose2d(0, 0, swerveChassis.getRotation2d()));

      // 3. Resetear encoders de los módulos
      swerveChassis.resetDriveEncoders();
    }, swerveChassis));

    lockPositionButton.whileTrue(new SwerveDriveXLockCmd(swerveChassis));

  }

  public void periodic() {

  }

  public Command getAutonomousCommand() {

    return autoChooser.getSelected();
    // return new AutoFollowTrajectoryCmd(swerveChassis);

  }
}
