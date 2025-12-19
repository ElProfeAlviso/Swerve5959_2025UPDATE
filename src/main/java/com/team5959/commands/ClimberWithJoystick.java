// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team5959.commands;

import edu.wpi.first.wpilibj2.command.Command;
import com.team5959.subsystems.Climber;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ClimberWithJoystick extends Command {
  private final Climber climber; // The subsystem used by this command 
  private final double speed;

  /** Creates a new ClimberWithJoystick. */
  public ClimberWithJoystick(Climber climber,double speed) {
    this.climber = climber;
    this.speed = speed;

    addRequirements(climber);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    climber.setClimberManualPosition(speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    climber.holdClimberPosition();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
