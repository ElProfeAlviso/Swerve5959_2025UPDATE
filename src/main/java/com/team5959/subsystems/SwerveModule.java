package com.team5959.subsystems;

//import com.revrobotics.CANSparkMax;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.servohub.ServoHub.ResetMode;
import com.revrobotics.spark.config.SparkMaxConfig;
//import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkBaseConfig;

import com.revrobotics.spark.config.EncoderConfig;

//import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.team5959.SwerveModuleConstants;
import com.team5959.Constants.SwerveConstants;

//import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;

import com.ctre.phoenix6.signals.SensorDirectionValue;

import com.ctre.phoenix6.hardware.CANcoder;

public class SwerveModule {
    /* * * INITIALIZATION * * */

    SparkMaxConfig driveConfig;
    SparkMaxConfig rotationConfig;

    public int moduleID; 
    //initialize motors 
    private SparkMax driveMotor; 
    private SparkMax rotationMotor; 

    //initialize encoders 
    private CANcoder absoluteEncoder; 
    private RelativeEncoder driveEncoder; 

    //init PID Controller for turning 
    private PIDController rotationPID; 

    //init info 
    private double encOffset; 

    /* * * CONSTRUCTOR * * */
    /* 
     * @param moduleID the id of the module 
     * @param moduleConstants a SwerveModuleConstants obj 
     */

    public SwerveModule(int moduleID, SwerveModuleConstants moduleConstants) {
        this.moduleID = moduleID; //used to differentiate between the four swerve modules in the SwerveSubsystem class 
        encOffset = moduleConstants.angleOffset;

        //instantiate drive motor and encoder 
        driveMotor = new SparkMax(moduleConstants.driveMotorID, MotorType.kBrushless); 
        driveConfig = new SparkMaxConfig();
        driveEncoder = driveMotor.getEncoder();

        //instantiate rotation motor and absolute encoder 
        rotationMotor = new SparkMax(moduleConstants.rotationMotorID, MotorType.kBrushless);
        rotationConfig = new SparkMaxConfig();
        absoluteEncoder = new CANcoder(moduleConstants.cancoderID);

        /* * * DRIVE MOTOR * * */
        //CONFIGURATIONS
        
        driveConfig.inverted(moduleConstants.driveInverted);
        driveConfig.idleMode(IdleMode.kBrake);
        driveConfig.smartCurrentLimit(25);
        driveConfig.encoder.positionConversionFactor(SwerveConstants.DRIVE_ENCODER_VELOCITY_CONVERSION);
        driveConfig.encoder.velocityConversionFactor(SwerveConstants.DRIVE_ENCODER_POSITION_CONVERSION);

        driveMotor.configure(driveConfig,null,null );
        //driveMotor.configure(driveConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);


        //driveMotor.setInverted(moduleConstants.driveInverted);
        //driveMotor.setIdleMode(IdleMode.kBrake);
        //driveMotor.setSmartCurrentLimit(25); //set current limit to 25 amps to prevent browning out in the middle of driving 

        //set conversion factor for drive enc 
       // driveEncoder.setVelocityConversionFactor(SwerveConstants.DRIVE_ENCODER_VELOCITY_CONVERSION); //reads velocity in meters per second instead of RPM
        //driveEncoder.setPositionConversionFactor(SwerveConstants.DRIVE_ENCODER_POSITION_CONVERSION); //reads velocity in meters instead of rotations

        /* * * ROTATION MOTOR * * */
        //CONFIGURATIONS
        rotationConfig.inverted(moduleConstants.rotationInverted);
        rotationConfig.idleMode(IdleMode.kBrake);
        rotationConfig.smartCurrentLimit(25);

        rotationMotor.configure(driveConfig, null, null);

        //rotationMotor.setInverted(moduleConstants.rotationInverted);
        //rotationMotor.setIdleMode(IdleMode.kBrake);
        //rotationMotor.setSmartCurrentLimit(25); //set current limit to 25 amps to prevent browning out in the middle of driving

        //configure rotation absolute encoder 
          //absoluteEncoder.getConfigurator().apply(new MagnetSensorConfigs().withAbsoluteSensorRange(AbsoluteSensorRangeValue.Signed_PlusMinusHalf)); //abs enc is now +-180
          absoluteEncoder.getConfigurator().apply(new MagnetSensorConfigs().withAbsoluteSensorDiscontinuityPoint(0.5));


        //absoluteEncoder.getConfigurator().apply(new MagnetSensorConfigs().withMagnetOffset(moduleConstants.angleOffset)); //implements encoder offset
        absoluteEncoder.getConfigurator().apply(new MagnetSensorConfigs().withSensorDirection(SensorDirectionValue.CounterClockwise_Positive)); //positive rotation occurs when magnet is spun counter-clockwise when observer is facing the LED side of CANCoder

        //configure rotation PID controller 
        rotationPID = new PIDController(
            SwerveConstants.KP_TURNING, 
            SwerveConstants.KI_TURNING, 
            SwerveConstants.KD_TURNING);
        rotationPID.enableContinuousInput(-180, 180); //Continuous input considers min & max to be the same point; calculates the shortest route to the setpoint 
    } 

    /* * * GET METHODS * * */
    private double driveVelocity() {
        return driveEncoder.getVelocity();
    }

    private double drivePosition() {
        return driveEncoder.getPosition();
    }

    private double getAbsoluteEncoderDegrees() {
        return (absoluteEncoder.getAbsolutePosition().getValueAsDouble() * 360) - encOffset;
    }

    //returns a new SwerveModuleState representing the current drive velocity and rotation motor angle 
    public SwerveModuleState getState() {
        return new SwerveModuleState(driveVelocity(), Rotation2d.fromDegrees(getAbsoluteEncoderDegrees()));
    }

    //returns a new SwerveModulePosition representing the current drive position and rotation motor angle 
    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(drivePosition(), Rotation2d.fromDegrees(getAbsoluteEncoderDegrees()));
    }

    /* * * SET METHODS * * */

    public void setState(SwerveModuleState desiredState) {
        //optimize state so the rotation motor doesnt have to spin as much 
        SwerveModuleState optimizedState = SwerveModuleState.optimize(desiredState, getState().angle);

        double rotationOutput = rotationPID.calculate(getState().angle.getDegrees(), optimizedState.angle.getDegrees());

        rotationMotor.set(rotationOutput);
        driveMotor.set(optimizedState.speedMetersPerSecond / SwerveConstants.MAX_SPEED * SwerveConstants.VOLTAGE); 

        SmartDashboard.putNumber("S[" + absoluteEncoder.getDeviceID() + "] DESIRED ANG DEG", getState().angle.getDegrees());
    }

    public void setAngle(SwerveModuleState desiredState) {
        SwerveModuleState optimizedState = SwerveModuleState.optimize(desiredState, getState().angle);

        double rotationOutput = rotationPID.calculate(getState().angle.getDegrees(), optimizedState.angle.getDegrees());

        rotationMotor.set(rotationOutput); 
        driveMotor.set(0);
    }

    public void stop(){
        driveMotor.set(0);
        rotationMotor.set(0);
    }

    public void print() {
        SmartDashboard.putNumber("S[" + absoluteEncoder.getDeviceID() + "] ABS ENC DEG", getAbsoluteEncoderDegrees());
        SmartDashboard.putNumber("S["+absoluteEncoder.getDeviceID()+"] DRIVE SPEED", driveVelocity());
        SmartDashboard.putNumber("S["+absoluteEncoder.getDeviceID()+"] ROTATION SPEED", absoluteEncoder.getVelocity().getValueAsDouble());
        SmartDashboard.putString("S["+absoluteEncoder.getDeviceID()+"] CURRENT STATE", getState().toString());

        
    }
}

