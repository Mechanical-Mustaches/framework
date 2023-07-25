package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.sensors.Pigeon2;
import com.ctre.phoenix.sensors.WPI_Pigeon2;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.CanConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ModuleConstants;
import frc.robot.Constants.IDConstants;
import frc.robot.Constants.PDPConstatns;
import frc.robot.Constants.PPConstants;;
// import frc.robot.Constants.AutoConstants;
// import frc.robot.Objects.SwerveModule;


public class DriveSubsystem extends SubsystemBase{
    private boolean fieldRelative;

    public SwerveDriveKinematics kSwerveKinematics = DriveConstants.kSwerveKinematics;

    public boolean isOpenLoop = true;


    private final SwerveMods front_left = new SwerveMods(
        IDConstants.FRONT_LEFT_LOCATION, 
        CanConstants.FRONT_LEFT_MODULE_DRIVE_MOTOR, 
        CanConstants.FRONT_LEFT_MODULE_STEER_MOTOR, 
        CanConstants.FRONT_LEFT_MODULE_STEER_CANCODER, 
        DriveConstants.kFrontLeftDriveMotorReversed, 
        DriveConstants.kFrontLeftTurningMotorReversed, 
        PDPConstatns.FRONT_LEFT_DRIVE_CHANNEL, 
        PDPConstatns.FRONT_LEFT_TURN_CHANNEL, 
        isOpenLoop, 
        CanConstants.FRONT_LEFT_MODULE_STEER_OFFSET);

        private final SwerveMods front_right = new SwerveMods(
            IDConstants.FRONT_RIGHT_LOCATION, 
            CanConstants.FRONT_RIGHT_MODULE_DRIVE_MOTOR, 
            CanConstants.FRONT_RIGHT_MODULE_STEER_MOTOR, 
            CanConstants.FRONT_RIGHT_MODULE_STEER_CANCODER, 
            DriveConstants.kFrontRightDriveMotorReversed, 
            DriveConstants.kFrontRightTurningMotorReversed, 
            PDPConstatns.FRONT_RIGHT_DRIVE_CHANNEL, 
            PDPConstatns.FRONT_RIGHT_TURN_CHANNEL, 
            isOpenLoop, 
            CanConstants.FRONT_RIGHT_MODULE_STEER_OFFSET);

        private final SwerveMods back_left = new SwerveMods(
            IDConstants.BACK_LEFT_LOCATION, 
            CanConstants.BACK_LEFT_MODULE_DRIVE_MOTOR, 
            CanConstants.BACK_LEFT_MODULE_STEER_MOTOR, 
            CanConstants.BACK_LEFT_MODULE_STEER_CANCODER, 
            DriveConstants.kBackLeftDriveMotorReversed, 
            DriveConstants.kBackLeftTurningMotorReversed, 
            PDPConstatns.BACK_LEFT_DRIVE_CHANNEL, 
            PDPConstatns.BACK_LEFT_TURN_CHANNEL, 
            isOpenLoop, 
            CanConstants.BACK_LEFT_MODULE_STEER_OFFSET);

        private final SwerveMods back_right = new SwerveMods(
            IDConstants.BACK_RIGHT_LOCATION, 
            CanConstants.BACK_RIGHT_MODULE_DRIVE_MOTOR, 
            CanConstants.BACK_RIGHT_MODULE_STEER_MOTOR, 
            CanConstants.BACK_RIGHT_MODULE_STEER_CANCODER, 
            DriveConstants.kBackRightDriveMotorReversed, 
            DriveConstants.kBackRightTurningMotorReversed, 
            PDPConstatns.BACK_RIGHT_DRIVE_CHANNEL, 
            PDPConstatns.BACK_RIGHT_TURN_CHANNEL, 
            isOpenLoop, 
            CanConstants.BACK_RIGHT_MODULE_STEER_OFFSET);
        
        WPI_Pigeon2 gyro_bird = new WPI_Pigeon2(30);

        private final SwerveDrivePoseEstimator m_poseEstimator = new SwerveDrivePoseEstimator(
            kSwerveKinematics, 
            gyro_bird.getRotation2d(),
            new SwerveModulePosition[] {
                front_left.getPosition(),
                front_right.getPosition(),
                back_left.getPosition(),
                back_right.getPosition()
               
            },
            new Pose2d(),
            VecBuilder.fill(.05, 0.05, Units.degreesToRadians(5)),
            VecBuilder.fill(.5, 0.5, Units.degreesToRadians(30)));

        public boolean showOnShuffleboard;

        public SimDouble m_simAngle;

        public double throttleValue;

        public boolean m_fieldOriented = false;

        public boolean useVisionOdometry = true;
      
        private double startTime;
      
        private double positionStart;
      
        double positionChange;
      
        private Pose2d visionPoseEstimatedData;
      
        private double latencyMs;
      
        public boolean visionDataAvailable;

        private PIDController xPID = new PIDController(
            PPConstants.kPXController, PPConstants.kIXController, PPConstants.kIXController);

        private PIDController yPID = new PIDController(
            PPConstants.kPYController, PPConstants.kIYController, PPConstants.kDYController);

        private PIDController thetaPID = new PIDController(
            PPConstants.kPThetaController, PPConstants.kIYController, PPConstants.kIThetaController);










}
