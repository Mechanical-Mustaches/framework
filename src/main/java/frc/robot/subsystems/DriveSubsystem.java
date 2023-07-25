// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.HashMap;
import java.util.Map;

import com.ctre.phoenix.sensors.Pigeon2;
import com.ctre.phoenix.sensors.WPI_Pigeon2;
import com.ctre.phoenix.sensors.WPI_PigeonIMU;
import com.ctre.phoenix.unmanaged.Unmanaged;
import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import java.util.stream.Collectors;

import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.hal.simulation.SimDeviceDataJNI;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants;
import frc.robot.Constants.CanConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.IDConstants;
import frc.robot.Constants.PDPConstants;
import frc.robot.Constants.PPConstants;
import frc.robot.Constants.DriveConstants.ModulePosition;
import frc.robot.utils.ModuleMap;
import frc.robot.utils.ShuffleboardContent;
import frc.robot.subsystems.Pigeon;

public class DriveSubsystem extends SubsystemBase {

  public SwerveDriveKinematics kSwerveKinematics = DriveConstants.kSwerveKinematics;

  public boolean isOpenLoop = true;

  private PathPlannerTrajectory testPath = PathPlanner.loadPath("SwerveTest", new PathConstraints(4, 3));

  private final SwerveModuleSparkMax front_left = new SwerveModuleSparkMax(
    IDConstants.FRONT_LEFT_LOCATION, 
    CanConstants.FRONT_LEFT_MODULE_DRIVE_MOTOR, 
    CanConstants.FRONT_LEFT_MODULE_STEER_MOTOR, 
    CanConstants.FRONT_LEFT_MODULE_STEER_CANCODER, 
    DriveConstants.kFrontLeftDriveMotorReversed, 
    DriveConstants.kFrontLeftTurningMotorReversed, 
    PDPConstants.FRONT_LEFT_DRIVE_CHANNEL, 
    PDPConstants.FRONT_LEFT_TURN_CHANNEL, 
    isOpenLoop,
    CanConstants.FRONT_LEFT_MODULE_STEER_OFFSET);

  private final SwerveModuleSparkMax front_right = new SwerveModuleSparkMax(
    IDConstants.FRONT_RIGHT_LOCATION, 
    CanConstants.FRONT_RIGHT_MODULE_DRIVE_MOTOR, 
    CanConstants.FRONT_RIGHT_MODULE_STEER_MOTOR, 
    CanConstants.FRONT_RIGHT_MODULE_STEER_CANCODER, 
    DriveConstants.kFrontRightDriveMotorReversed, 
    DriveConstants.kFrontRightTurningMotorReversed, 
    PDPConstants.FRONT_RIGHT_DRIVE_CHANNEL, 
    PDPConstants.FRONT_RIGHT_TURN_CHANNEL, 
    isOpenLoop,
    CanConstants.FRONT_RIGHT_MODULE_STEER_OFFSET);
  
  private final SwerveModuleSparkMax back_left = new SwerveModuleSparkMax(
    IDConstants.BACK_LEFT_LOCATION, 
    CanConstants.BACK_LEFT_MODULE_DRIVE_MOTOR, 
    CanConstants.BACK_LEFT_MODULE_STEER_MOTOR, 
    CanConstants.BACK_LEFT_MODULE_STEER_CANCODER, 
    DriveConstants.kBackLeftDriveMotorReversed, 
    DriveConstants.kBackLeftTurningMotorReversed, 
    PDPConstants.BACK_LEFT_DRIVE_CHANNEL, 
    PDPConstants.BACK_LEFT_TURN_CHANNEL, 
    isOpenLoop,
    CanConstants.BACK_LEFT_MODULE_STEER_OFFSET);

  private final SwerveModuleSparkMax back_right = new SwerveModuleSparkMax(
    IDConstants.BACK_RIGHT_LOCATION, 
    CanConstants.BACK_RIGHT_MODULE_DRIVE_MOTOR, 
    CanConstants.BACK_RIGHT_MODULE_STEER_MOTOR, 
    CanConstants.BACK_RIGHT_MODULE_STEER_CANCODER, 
    DriveConstants.kBackRightDriveMotorReversed, 
    DriveConstants.kBackRightTurningMotorReversed, 
    PDPConstants.BACK_RIGHT_DRIVE_CHANNEL, 
    PDPConstants.BACK_RIGHT_TURN_CHANNEL, 
    isOpenLoop,
    CanConstants.BACK_RIGHT_MODULE_STEER_OFFSET);
  
    
        
    
  
  
  // The gyro sensor

  //private final AHRS m_gyro = new AHRS(SPI.Port.kMXP, (byte) 200);
    WPI_Pigeon2 gyro = new WPI_Pigeon2(30);
  //Pigeon2 gyro = new Pigeon2(30);

  private final SwerveDrivePoseEstimator m_poseEstimator = new SwerveDrivePoseEstimator(
    kSwerveKinematics,
    gyro.getRotation2d(),
   new SwerveModulePosition[] {
    front_left.getPosition(),
    front_right.getPosition(),
    back_left.getPosition(),
    back_right.getPosition()
   },
    new Pose2d(),
    VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(5)),
    VecBuilder.fill(0.5, 0.5, Units.degreesToRadians(30))
    );

    private PIDController xPID = new PIDController(
        PPConstants.kPXController, PPConstants.kIXController, PPConstants.kDXController);

    private PIDController yPID = new PIDController(
        PPConstants.kPYController, PPConstants.kIYController, PPConstants.kDYController);
    
    private PIDController thetaPID = new PIDController(
        PPConstants.kPThetaController, PPConstants.kIThetaController, PPConstants.kDThetaController);

//   private PIDController m_xController = new PIDController(DriveConstants.kP_X, 0, DriveConstants.kD_X);
//   private PIDController m_yController = new PIDController(DriveConstants.kP_Y, 0, DriveConstants.kD_Y);
  // private ProfiledPIDController m_turnController = new ProfiledPIDController(
  //     DriveConstants.kP_Theta, 0,
  //     DriveConstants.kD_Theta,
  //     Constants.TrapezoidConstants.kThetaControllerConstraints);

  // private final SwerveDrivePoseEstimator m_odometry;
  // SwerveModulePosition[] mpos;

  private boolean showOnShuffleboard = true;

  private SimDouble m_simAngle;// navx sim

  private double m_simYaw;

  public double throttleValue;

  public double targetAngle;

   public boolean m_fieldOriented;

  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {
    
   // SwerveModulePosition[] mpos = new SwerveModulePosition[4];
   // mpos = m_swerveModules.values().stream().map(module -> module.getPosition()).collect(Collectors.toList()).toArray(mpos);
   // m_poseEstimator = new SwerveDrivePoseEstimator(kSwerveKinematics, getHeadingRotation2d(), mpos, getPoseMeters());

    gyro.reset();
    //gyro.getResetCount();
    
   
 

    resetModuleEncoders();

    setIdleMode(true);

    m_fieldOriented=false;

    if (RobotBase.isSimulation()) {

      var dev = SimDeviceDataJNI.getSimDeviceHandle("navX-Sensor[0]");

      m_simAngle = new SimDouble((SimDeviceDataJNI.getSimValueHandle(dev, "Yaw")));
    }

    ShuffleboardContent.initMisc(this);
  }
 
  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed        Speed of the robot in the x direction (forward).
   * @param ySpeed        Speed of the robot in the y direction (sideways).
   * @param rot           Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the
   *                      field.
   */
  @SuppressWarnings("ParameterName")

  // move the robot from gamepad
  public void drive(
      double throttle,
      double strafe,
      double rotation) {
      var swerveModuleStates = kSwerveKinematics.toSwerveModuleStates(
          this.m_fieldOriented
              ? ChassisSpeeds.fromFieldRelativeSpeeds(throttle, strafe, rotation, gyro.getRotation2d())
              : new ChassisSpeeds(throttle, strafe, rotation));
      SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, DriveConstants.kMaxSpeedMetersPerSecond);
      front_left.setDesiredState(swerveModuleStates[0]);
      front_right.setDesiredState(swerveModuleStates[1]);
      back_left.setDesiredState(swerveModuleStates[2]);
      back_right.setDesiredState(swerveModuleStates[3]);

      }
      /**
       * Sets the swerve ModuleStates.
       *
       * @param desiredStates The desired SwerveModule states.
       */
      public void setModuleStates(SwerveModuleState[] desiredStates){
        SwerveDriveKinematics.desaturateWheelSpeeds(
        desiredStates, DriveConstants.kMaxSpeedMetersPerSecond);
          front_left.setDesiredState(desiredStates[0]);
          front_right.setDesiredState(desiredStates[1]);
          back_left.setDesiredState(desiredStates[2]);
          back_right.setDesiredState(desiredStates[3]);
      }

  @Override
  public void periodic() {
    // Update the odometry in the periodic block
    updateOdometry();
    SmartDashboard.putNumber("Yaw",-gyro.getYaw());

  }

  public void updateOdometry() {
    if(checkCANOK()) {

      /* Updates the field relative position of the robot */
      m_poseEstimator.update(
        gyro.getRotation2d(), 
        new SwerveModulePosition[] {
          front_left.getPosition(),
          front_right.getPosition(),
          back_left.getPosition(),
          back_right.getPosition()
        });

    }
  }

  public Pose2d getEstimatedPose(){
    return m_poseEstimator.getEstimatedPosition();
  }

  public void resetOdometry(Pose2d pose){
    m_poseEstimator.resetPosition(getHeadingRotation2d(),
      new SwerveModulePosition[] {
        front_left.getPosition(),
        front_right.getPosition(),
        back_left.getPosition(),
        back_right.getPosition()
      }, 
      pose);
    gyro.reset();
         
  }

  public Pose2d getPoseMeters() {
    return m_poseEstimator.getEstimatedPosition();
  }

  public Pose2d getestimatedPose(){
    return m_poseEstimator.getEstimatedPosition();
  }

 

  // public Pose2d getPose(){
  //   return m_odometry.getPoseMeters();
  // }

  public SwerveDrivePoseEstimator getOdometry() {
    return m_poseEstimator;
  }

  public void setOdometry(Pose2d pose) {
    gyro.reset();
    front_left.resetAngleToAbsolute();
    front_right.resetAngleToAbsolute();
    back_left.resetAngleToAbsolute();
    back_right.resetAngleToAbsolute();
    //gyro.getResetCount();


  }

  public boolean checkCANOK() {
    return RobotBase.isSimulation() ||
      front_left.checkCAN()
      && front_right.checkCAN()
      && back_left.checkCAN()
      && back_right.checkCAN();
  }
 

  public double getHeadingDegrees() {

    return -Math.IEEEremainder((gyro.getAngle()), 360);
   // System.out.println("AbsoluteCompassHeading: " + gyro.getAbsoluteCompassHeading());
    //return -Math.IEEEremainder((gyro.getAbsoluteCompassHeading()), 360);

  }

  public Rotation2d getHeadingRotation2d() {
    return Rotation2d.fromDegrees(getHeadingDegrees());
  }

  /**
   * Sets the swerve ModuleStates.
   *
   * @param desiredStates The desired SwerveModule states.
   */
  public void setSwerveModuleStates(SwerveModuleState[] states, boolean isOpenLoop) {
    SwerveDriveKinematics.desaturateWheelSpeeds(states, DriveConstants.kMaxSpeedMetersPerSecond);
      front_left.setDesiredState(states[front_left.getModulePosition().ordinal()], isOpenLoop);
      front_right.setDesiredState(states[front_right.getModulePosition().ordinal()], isOpenLoop);
      back_left.setDesiredState(states[back_left.getModulePosition().ordinal()], isOpenLoop);
      back_right.setDesiredState(states[back_right.getModulePosition().ordinal()], isOpenLoop);

  }

  public void resetModuleEncoders() {
      front_left.resetAngleToAbsolute();
      front_right.resetAngleToAbsolute();
      back_left.resetAngleToAbsolute();
      back_right.resetAngleToAbsolute();
  }

  /** Zeroes the heading of the robot. */
  public void zeroHeading() {
    gyro.reset();
    // m_gyro.setAngleAdjustment(0);
    //gyro.getResetCount();

  }

  public Translation2d getTranslation() {
    return getPoseMeters().getTranslation();
  }



  public void setSwerveModuleStatesAuto(SwerveModuleState[] states) {
    setSwerveModuleStates(states, false);
  }

  public void stopModules(){
    front_left.stop();
    front_right.stop();
    back_left.stop();
    back_right.stop();
  }

  public PIDController getThetaPidController() {
    return thetaPID;
  }

  public double getX() {
    return getTranslation().getX();
  }

  public double getY() {
    return getTranslation().getY();
  }

  public double reduceRes(double value, int numPlaces) {
    double n = Math.pow(10, numPlaces);
    return Math.round(value * n) / n;
  }

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  // public double getTurnRate() {
  // return m_gyro.getRate() * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
  // }

  public void setIdleMode(boolean brake) {
    front_left.setDriveBrakeMode(brake);
    front_left.setTurnBrakeMode(brake);
    front_right.setDriveBrakeMode(brake);
    front_right.setTurnBrakeMode(brake);
    back_left.setDriveBrakeMode(brake);
    back_left.setTurnBrakeMode(brake);
    back_right.setDriveBrakeMode(brake);
    back_right.setTurnBrakeMode(brake);
  }

  @Override
  public void simulationPeriodic() {
    ChassisSpeeds chassisSpeedSim = kSwerveKinematics.toChassisSpeeds(
       
      new SwerveModuleState[] {
        front_left.getState(),
        front_right.getState(),
        back_left.getState(),
        back_right.getState()
      });
    // want to simulate navX gyro changing as robot turns
    // information available is radians per second and this happens every 20ms
    // radians/2pi = 360 degrees so 1 degree per second is radians / 2pi
    // increment is made every 20 ms so radian adder would be (rads/sec) *(20/1000)
    // degree adder would be radian adder * 360/2pi
    // so degree increment multiplier is 360/100pi = 1.1459

    double temp = chassisSpeedSim.omegaRadiansPerSecond * 1.1459155;

    temp += m_simAngle.get();

    m_simAngle.set(temp);

    Unmanaged.feedEnable(20);
  }

  public void turnModule(ModulePosition frontLeft, double speed) {
    frontLeft.turnMotorMove(speed);
  }

  public void positionTurnModule(ModulePosition m_mp, double angle) {
    m_mp.positionTurn(angle);
  }

  public void driveModule(ModulePosition frontLeft, double speed) {
    frontLeft.turnMotorMove(speed);
  }

  public boolean getTurnInPosition(ModulePosition m_mp, double targetAngle) {
    return m_mp.turnInPosition(targetAngle);
  }

  public double getAnglefromThrottle() {

    return 180 * throttleValue;
  }

  public PIDController getXPID() {
    return xPID;
  }

  public PIDController getYPID(){
    return yPID;
  }

  public void setZeroNOW(){
    front_left.setZero();
    front_right.setZero();
    back_left.setZero();
    back_right.setZero();
  }

  public Command followTrajectoryCommand(PathPlannerTrajectory trajectory, boolean isFirstPath){
    return new SequentialCommandGroup(
      new InstantCommand(() -> {
        if(isFirstPath){
          resetOdometry(trajectory.getInitialHolonomicPose());
        }
      }),
      new PPSwerveControllerCommand(
        trajectory, 
        this::getEstimatedPose,
        kSwerveKinematics,
        getXPID(),
        getYPID(), 
        getThetaPidController(), 
      this::setModuleStates,
      true,
      this),
      

    new InstantCommand(() -> stopModules()));
    // new InstantCommand(() -> stopModules());
  }

  public DriveSubsystem getSwerveModule(ModulePosition i) {
    return null;
  }
}
