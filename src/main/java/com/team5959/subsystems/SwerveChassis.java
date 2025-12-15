package com.team5959.subsystems;

import com.studica.frc.AHRS;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.team5959.Constants.SwerveConstants;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SwerveChassis extends SubsystemBase{

 /* * * INITIALIZATION * * */
 
  //initialize SwerveModules 
  private SwerveModule[] swerveModules; 

  //odometer 
  private SwerveDriveOdometry odometer; 
  private AHRS navx; 

  Field2d field2d = new edu.wpi.first.wpilibj.smartdashboard.Field2d();  

  public SwerveChassis() {

    swerveModules = new SwerveModule[] {
      new SwerveModule(0, SwerveConstants.FrontLeft.constants), //Front Left Module
      new SwerveModule(1, SwerveConstants.BackLeft.constants), //Back Left Module
      new SwerveModule(2, SwerveConstants.FrontRight.constants), //Front Right Module
      new SwerveModule(3, SwerveConstants.BackRight.constants)//Back Right Module
    };

    //instantiate navx 
    navx = new AHRS(AHRS.NavXComType.kMXP_SPI);
    navx.setAngleAdjustment(0); //adjustment may be needed depending on robot orientation

    //instantiate odometer 
    odometer = new SwerveDriveOdometry(
      SwerveConstants.DRIVE_KINEMATICS, 
      getRotation2d(), 
      getModulePositions()
    );

    //reset navx after 1 second to ensure proper initialization
    new Thread(() -> {
      try {
        Thread.sleep(1000);
        resetNavx();
      } catch (Exception e) {
        
      }
    }).start();

      
  }

   //Methods

  public void resetNavx() {
    navx.reset();
  }

  public Rotation2d getRotation2d() {
    return navx.getRotation2d();
  }

  public Pose2d getPose() {
    return odometer.getPoseMeters();
  }
/* 
   public void setPose(Pose2d pose) {
    odometer.resetPosition(getRotation2d(), getModulePositions(), pose);
  }  */

  public void resetOdometry(Pose2d pose) {
    odometer.resetPosition(getRotation2d(), getModulePositions(), pose);
  }

  public ChassisSpeeds getRobotRelativeSpeeds() {
    ChassisSpeeds speeds = SwerveConstants.DRIVE_KINEMATICS.toChassisSpeeds(getModuleStates());
    return new ChassisSpeeds(
      speeds.vxMetersPerSecond,
      speeds.vyMetersPerSecond,
      speeds.omegaRadiansPerSecond
    );
  }

  public void driveRobotRelative(ChassisSpeeds chassis) {
    SwerveModuleState[] state = SwerveConstants.DRIVE_KINEMATICS.toSwerveModuleStates(chassis);

    setModuleStates(state);
  }

  /* * * STATES * * */

  //SET STATES 
  //gets a SwerveModuleStates array from driver control and sets each module to the corresponding SwerveModuleState
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, SwerveConstants.MAX_SPEED);

    for (SwerveModule swerveMod : swerveModules) {
      swerveMod.setState(desiredStates[swerveMod.moduleID]);
    }
  }

  //GET STATES 
  //returns the states of the swerve modules in an array 
  //getState uses drive velocity and module rotation 
  public SwerveModuleState[] getModuleStates() {
    SwerveModuleState[] states = new SwerveModuleState[4]; 

    for (SwerveModule swerveMod : swerveModules) {
      states[swerveMod.moduleID] = swerveMod.getState();
    }

    return states; 
  }

  //GET POSITIONS
  //returns the positions of the swerve modules in an array 
  //getPosition uses drive enc and module rotation 
  public SwerveModulePosition[] getModulePositions() {
    SwerveModulePosition[] positions = new SwerveModulePosition[4]; 

    for (SwerveModule swerveMod : swerveModules) {
      positions[swerveMod.moduleID] = swerveMod.getPosition();
    }

    return positions;
  }


  

  //LOCK 
  public void lock() {
    SwerveModuleState[] states = new SwerveModuleState[4];

    states[0] = new SwerveModuleState(0, new Rotation2d(Math.toRadians(45)));
    states[1] = new SwerveModuleState(0, new Rotation2d(Math.toRadians(-45)));
    states[2] = new SwerveModuleState(0, new Rotation2d(Math.toRadians(45)));
    states[3] = new SwerveModuleState(0, new Rotation2d(Math.toRadians(-45)));

    for (SwerveModule swerveMod : swerveModules) {
      swerveMod.setAngle(states[swerveMod.moduleID]);
    }
  }

  //STRAIGHTEN THE WHEELS 
  public void straightenWheels() { //set all wheels to 0 degrees 
    SwerveModuleState[] states = new SwerveModuleState[4]; 

    states[0] = new SwerveModuleState(0, new Rotation2d(Math.toRadians(0)));
    states[1] = new SwerveModuleState(0, new Rotation2d(Math.toRadians(0)));
    states[2] = new SwerveModuleState(0, new Rotation2d(Math.toRadians(0)));
    states[3] = new SwerveModuleState(0, new Rotation2d(Math.toRadians(0)));

    for (SwerveModule swerveMod : swerveModules) {
      swerveMod.setState(states[swerveMod.moduleID]);
    }
  }

  //DRIVE
  public void drive(double xSpeed, double ySpeed, double zSpeed, boolean fieldOriented){
    SwerveModuleState[] states;
    if (fieldOriented) {
      states = SwerveConstants.DRIVE_KINEMATICS.toSwerveModuleStates(
        ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, zSpeed, getRotation2d())

      );
    } else {
      states = SwerveConstants.DRIVE_KINEMATICS.toSwerveModuleStates(
        new ChassisSpeeds(xSpeed, ySpeed, zSpeed)
      );
    }

    setModuleStates(states);

  }

  //STOP 
  public void stopModules() {
    for (SwerveModule swerveMod : swerveModules) {
      swerveMod.stop();
    }
}

public void resetDriveEncoders() {
  for (SwerveModule swerveMod : swerveModules) {
    swerveMod.resetDriveEncoder();
  }
}

public void publishTrajectory(String name, Trajectory trajectory) {
  field2d.getObject(name).setTrajectory(trajectory);
}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    odometer.update(getRotation2d(), getModulePositions());
    
    for (SwerveModule swerveMod : swerveModules) {
      swerveMod.print();
    }
   
    SmartDashboard.putNumber("NAVX", -navx.getAngle());
    SmartDashboard.putNumber("NAVXYAW", navx.getYaw());
    SmartDashboard.putData("NAVX2D", navx);
    SmartDashboard.putString("POSE INFO", odometer.getPoseMeters().toString());
    SmartDashboard.putNumber("rot 2d", ((getRotation2d().getDegrees() % 360) + 360) % 360);

    SmartDashboard.putNumber("Distancia FL", swerveModules [0].getPosition().distanceMeters);
    SmartDashboard.putNumber("Distancia RL", swerveModules [1].getPosition().distanceMeters);
    SmartDashboard.putNumber("Distancia FR", swerveModules [2].getPosition().distanceMeters);
    SmartDashboard.putNumber("Distancia RR", swerveModules [3].getPosition().distanceMeters);

    

    
    

  // Add Field2d to display odometry on SmartDashboard
   SmartDashboard.putData("Field", field2d);
  field2d.setRobotPose(odometer.getPoseMeters());
    
  }

  /* * * ADDED METHODS * * */
public double deadzone(double num){
  return Math.abs(num) > 0.1 ? num : 0;
}

@SuppressWarnings("unused")
private static double modifyAxis(double num) {
// Square the axis
num = Math.copySign(num * num, num);

return num;
}

}
