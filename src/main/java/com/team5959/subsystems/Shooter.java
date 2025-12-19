// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team5959.subsystems;

import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {

   // Creacion de objeto de Shooter
  private final SparkMax shooterMotorRight = new SparkMax(20, MotorType.kBrushless); // Motor del shooter
  private final SparkMaxConfig shooterMotorRightConfig = new SparkMaxConfig(); // Configuración del motor del shooter
  private final SparkClosedLoopController shooterRightPid = shooterMotorRight.getClosedLoopController(); // Controlador PID del shooter

  private final SparkMax shooterMotorLeft = new SparkMax(21, MotorType.kBrushless); // Motor del shooter
  private final SparkMaxConfig shooterMotorLeftConfig = new SparkMaxConfig(); // Configuración del motor del shooter

  private double shooterSetPoint = 0;//Variable para almacenar el setpoint del shooter
  private boolean shooterEnabled = false;

  

 

  //Creacion de objeto de Sendable personalizado  del Shooter PID Sparkmax para envio a elastic.
  //Esto crea un objeto en el dashboard que permite modificar los valores del PID en tiempo real.
  Sendable pidShooterSendable = new Sendable() {
    @Override
    public void initSendable(SendableBuilder shooterBuilder) {
      shooterBuilder.setSmartDashboardType("Shooter PIDController");

      shooterBuilder.addDoubleProperty("P", () -> shooterMotorRight.configAccessor.closedLoop.getP(), 
      x -> {shooterMotorRightConfig.closedLoop.p(x);
            shooterMotorRight.configure(shooterMotorRightConfig, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);
      });

      shooterBuilder.addDoubleProperty("I", () -> shooterMotorRight.configAccessor.closedLoop.getI(),
      x -> {shooterMotorRightConfig.closedLoop.i(x);
            shooterMotorRight.configure(shooterMotorRightConfig, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);
      });

      shooterBuilder.addDoubleProperty("D", () -> shooterMotorRight.configAccessor.closedLoop.getD(),
      x -> {shooterMotorRightConfig.closedLoop.d(x);
            shooterMotorRight.configure(shooterMotorRightConfig, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);
      });

      shooterBuilder.addDoubleProperty("FF", () -> shooterMotorRight.configAccessor.closedLoop.getFF(),
      x -> {shooterMotorRightConfig.closedLoop.velocityFF(x);
            shooterMotorRight.configure(shooterMotorRightConfig, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);
      });
    }
  };
  
  
  /** Creates a new Shooter. */
  public Shooter() {
    // Configuracion de motor de Shooter
    // Configura el modo de inactividad, inversión, límite de corriente y sensor de retroalimentación
    shooterMotorRightConfig.idleMode(IdleMode.kBrake); //Configura el modo Libre sin freno
    shooterMotorRightConfig.inverted(false);//Invierte el giro del motor
    shooterMotorRightConfig.smartCurrentLimit(40);//Establece el límite de corriente

    shooterMotorLeftConfig.idleMode(IdleMode.kBrake); //Configura el modo Libre sin freno
    shooterMotorLeftConfig.inverted(false);//Invierte el giro del motor
    shooterMotorLeftConfig.smartCurrentLimit(40);//Establece el límite de corriente
    shooterMotorLeftConfig.follow(shooterMotorRight, true); // El motor izquierdo sigue al derecho

    shooterMotorRightConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder);// Usa el encoder interno como sensor de retroalimentación
    shooterMotorRightConfig.closedLoop.pidf(0.000001, 0, 0, 0.000172); // Valores PID y FF ajustados manualmente
    shooterMotorRightConfig.closedLoop.outputRange(-1, 1); // Rango de salida del controlador PID

    shooterSetPoint = 0; // Setpoint inicial del shooter

    // Aplica la configuración al motor del shooter
    shooterMotorRight.configure(shooterMotorRightConfig, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);
    shooterMotorLeft.configure(shooterMotorLeftConfig, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);

     // Envía los controles PID del Shooter al SmartDashboard para ajustes en tiempo real
     //SmartDashboard.putData("PID Shooter", pidShooterSendable); 

  }

  public void setShooterPIDSpeed(double setPoint) {
    shooterSetPoint = setPoint; // Método para establecer el setpoint del shooter
    shooterEnabled = true;

    // Ajusta el setpoint basado en el voltaje de la batería
    double batteryVoltage = edu.wpi.first.wpilibj.RobotController.getBatteryVoltage();
    double voltageCompensationFactor = 12.0 / batteryVoltage; // Factor de compensación basado en 12V nominales
    double adjustedSetPoint = shooterSetPoint * voltageCompensationFactor;

    shooterRightPid.setReference(adjustedSetPoint, ControlType.kVelocity); // Control PID para el shooter
  }

  public void stopShooter() {
    shooterMotorRight.stopMotor(); // Método para detener el shooter estableciendo el setpoint a 0
    shooterEnabled = false;
  }

  public boolean  isStopped(){
    
    return shooterEnabled;
    // Método para verificar si el shooter está detenido
  }

  public void getSpeed(){
    shooterMotorRight.getEncoder().getVelocity(); // Método para obtener la velocidad actual del shooter
  }

  public boolean isAtSpeed (double speed, double tolerance) {
    double currentSpeed = shooterMotorRight.getEncoder().getVelocity();
    return Math.abs(currentSpeed - speed) <= tolerance;
  }

  public boolean isPIDEnabled() {
    return shooterEnabled;
  }

 

  

  @Override
  public void periodic() {
   
/* 
    //PID Shooter Smartdashboard
    SmartDashboard.putNumber("Shooter Set Point", shooterSetPoint);
    SmartDashboard.putNumber("Shooter Velocity", shooterMotorRight.getEncoder().getVelocity());
    SmartDashboard.putNumber("Shooter Output", shooterMotorRight.getAppliedOutput());
 */
    
  }
}
