// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team5959.subsystems;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.hardware.CANrange;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SoftLimitConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkBase.ControlType;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class Climber extends SubsystemBase {

  // Creacion de objeto de sensor de distancia y deteccion de objetos CANrange
  private final CANBus kCANBus = new CANBus("rio");
  private final CANrange canRange = new CANrange(30, kCANBus);

  // Creacion de objeto de Climber
  private final SparkMax climberRightMotor = new SparkMax(16, MotorType.kBrushless); // Motor del climber
  private final SparkMaxConfig climberRightMotorConfig = new SparkMaxConfig(); // Configuración del motor del climber
  private final SparkMax climberLeftMotor = new SparkMax(17, MotorType.kBrushless); // Motor del climber
  private final SparkMaxConfig climberLeftMotorConfig = new SparkMaxConfig(); // Configuración del motor del climber



  private final SoftLimitConfig climberRightSoftLimitsConfig = new SoftLimitConfig(); // Configuración de límites suaves del climber
  private final SoftLimitConfig climberLeftSoftLimitsConfig = new SoftLimitConfig(); // Configuración de límites suaves del climber

  private final SparkClosedLoopController climberPid = climberRightMotor.getClosedLoopController(); // Controlador PID del climber

  private double climberSetPoint= 0; // Variable para almacenar el setpoint del climber.
  private double currentPosition = 0;
  private boolean climberEnablePID = false; // Variable para habilitar o deshabilitar el control PID del climber
  private double climberManualSpeed = 0;

  // Creacion de objeto de Sendable personalizado del Climber PID Sparkmax para envio a elastic.
  // Esto crea un objeto en el dashboard que permite modificar los valores del PID en tiempo real.
  Sendable pidClimberSendable = new Sendable() {
    @Override
    public void initSendable(SendableBuilder climberBuilder) {
      climberBuilder.setSmartDashboardType("Climber PIDController");

      climberBuilder.addDoubleProperty("P", () -> climberRightMotor.configAccessor.closedLoop.getP(),
      x -> {climberRightMotorConfig.closedLoop.p(x);
            climberRightMotor.configure(climberRightMotorConfig, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);
      });

      climberBuilder.addDoubleProperty("I", () -> climberRightMotor.configAccessor.closedLoop.getI(),
      x -> {climberRightMotorConfig.closedLoop.i(x);
            climberRightMotor.configure(climberRightMotorConfig, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);
      });

      climberBuilder.addDoubleProperty("D", () -> climberRightMotor.configAccessor.closedLoop.getD(),
      x -> {climberRightMotorConfig.closedLoop.d(x);
            climberRightMotor.configure(climberRightMotorConfig, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);
      });

      climberBuilder.addDoubleProperty("FF", () -> climberRightMotor.configAccessor.closedLoop.getFF(),
      x -> {climberRightMotorConfig.closedLoop.velocityFF(x);
            climberRightMotor.configure(climberRightMotorConfig, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);
      });
    }
  };
  

  /** Creates a new Elevator. */
  public Climber() {

     // Envía los controles PID del Climber al SmartDashboard para ajustes en tiempo real
    // SmartDashboard.putData("PID Climber", pidClimberSendable);

    // Configuracion de motor de Climber
    climberRightMotorConfig.idleMode(IdleMode.kBrake); // Configura el modo de inactividad en freno
    climberRightMotorConfig.inverted(true); // Invierte el giro del motor
    climberRightMotorConfig.smartCurrentLimit(40); // Establece el límite de corriente

    // Configuracion de motor de Climber
    climberLeftMotorConfig.idleMode(IdleMode.kBrake); // Configura el modo de inactividad en freno
    climberLeftMotorConfig.inverted(true); // Invierte el giro del motor
    climberLeftMotorConfig.smartCurrentLimit(40); // Establece el límite de corriente



    climberRightMotorConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder); // Usa el encoder interno como sensor de retroalimentación
    climberRightMotorConfig.closedLoop.pid(0.35, 0.000001, 0.001); // Valores PID ajustados manualmente (Usando Rev Hardware Client
    climberRightMotorConfig.closedLoop.outputRange(-1, 1); // Rango de salida del controlador PID
    climberSetPoint = 0; // Setpoint inicial del climber

    // Configuración de límites suaves (soft limits) del Climber
    climberRightSoftLimitsConfig.forwardSoftLimitEnabled(true); // Habilita límite suave hacia adelante
    climberRightSoftLimitsConfig.forwardSoftLimit(150); // Posición máxima hacia adelante
    climberRightSoftLimitsConfig.reverseSoftLimitEnabled(true); // Habilita límite suave hacia atrás
    climberRightSoftLimitsConfig.reverseSoftLimit(0); // Posición mínima hacia atrás

    // Configuración de límites suaves (soft limits) del Climber
    climberLeftSoftLimitsConfig.forwardSoftLimitEnabled(true); // Habilita límite suave hacia adelante
    climberLeftSoftLimitsConfig.forwardSoftLimit(150); // Posición máxima hacia adelante
    climberLeftSoftLimitsConfig.reverseSoftLimitEnabled(true); // Habilita límite suave hacia atrás
    climberLeftSoftLimitsConfig.reverseSoftLimit(0); // Posición mínima hacia atrás
    climberLeftMotorConfig.follow(climberRightMotor, true);

    // Aplica la configuración de límites suaves y del motor
    climberRightMotorConfig.apply(climberRightSoftLimitsConfig);
    climberRightMotor.configure(climberRightMotorConfig, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);
    climberRightMotor.getEncoder().setPosition(0); // Resetea la posición del encoder al iniciar

    // Aplica la configuración de límites suaves y del motor
    climberLeftMotorConfig.apply(climberLeftSoftLimitsConfig);
    climberLeftMotor.configure(climberLeftMotorConfig, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);
    climberLeftMotor.getEncoder().setPosition(0); // Resetea la posición del encoder al iniciar
   
    

  }

  // Método para establecer el setpoint del Climber
  public void setClimberPIDPosition(double setPoint) {    
    climberSetPoint = setPoint;
    climberEnablePID = true;    
    climberPid.setReference(setPoint, ControlType.kPosition);
  }
  // Método para habilitar o deshabilitar el control PID del Climber
  public void setEnableClimberPID(boolean enablePID) {
    climberEnablePID = enablePID;
  }

 
   public void ClimberStopMotor() {
    climberRightMotor.stopMotor(); // Método para detener el Climber estableciendo el setpoint a 0
  }

  public boolean isAtPosition (double position, double tolerance) {
    double currentPosition = climberRightMotor.getEncoder().getPosition();
    return Math.abs(currentPosition - position) <= tolerance;
  }   

  public void setClimberManualPosition(double climberManualSpeed) {
    this.climberManualSpeed = climberManualSpeed;
    climberEnablePID = false;
    climberRightMotor.set(climberManualSpeed);  
  }

  public void holdClimberPosition() {
    currentPosition = climberRightMotor.getEncoder().getPosition();
    climberEnablePID = true;    
    climberPid.setReference(currentPosition, ControlType.kPosition);
  }



  

  @Override
  public void periodic() {  
    
    
/* 
   

    // Actualiza el SmartDashboard con la posición del encoder del Climber
    SmartDashboard.putNumber("Climber Position Encoder", climberRightMotor.getEncoder().getPosition());

     //PID Climber Smartdashboard
    SmartDashboard.putNumber("Climber Set Point", climberSetPoint);
    SmartDashboard.putNumber("Climber Encoder", climberRightMotor.getEncoder().getPosition());
    SmartDashboard.putNumber("Climber Output", climberRightMotor.getAppliedOutput());
    SmartDashboard.putBoolean("LLego a posicion", isAtPosition(climberSetPoint, 2));
    SmartDashboard.putBoolean("Climber PID Enabled", climberEnablePID);
    SmartDashboard.putNumber("Climber Manual Speed", climberManualSpeed);
     */
  }
}
