// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team5959.commands;

import edu.wpi.first.wpilibj2.command.Command;
import com.team5959.subsystems.Climber;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ClimberPID extends Command {

  private final Climber climber; // Subsystem that controls the climber
  private double setPoint; // Desired setpoint for the climber
  private double tolerance = 2;
  

  /** Creates a new ClimberPID. */
  public ClimberPID(Climber climberSubsystem, double setPoint){
    this.climber = climberSubsystem;
    this.setPoint = setPoint;
    
    addRequirements(climber);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    climber.setClimberPIDPosition(setPoint); // Set the desired setpoint for the climber

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    
  } 

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    climber.ClimberStopMotor();
    // Stop the climber motor
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return climber.isAtPosition(setPoint, tolerance); // Check if the climber has reached the desired position;
  }
}
