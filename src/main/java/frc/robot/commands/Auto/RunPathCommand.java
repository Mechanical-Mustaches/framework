package frc.robot.commands.Auto;

import java.nio.file.Path;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.subsystems.DriveSubsystem;
import pabeles.concurrency.ConcurrencyOps.NewInstance;


public class RunPathCommand extends SequentialCommandGroup{
    
    DriveSubsystem m_driveSubsystem;
    Command m_pathCommad;
    Pose2d m_pose;
    int m_ticks = 0;

    public Rotation2d desiredRotation(){
        m_ticks += 1;
        return Rotation2d.fromDegrees(90);
    }

    public RunPathCommand(DriveSubsystem driveSubsystem, String path){
        try {
         m_driveSubsystem = driveSubsystem;
         Path path2 = Filesystem.getDeployDirectory().toPath().resolve("pathweaver/output/" + path + "wpilib.json");
         Trajectory trajectory = TrajectoryUtil.fromPathweaverJson(path2);   
         SwerveControllerCommand command = new SwerveControllerCommand(trajectory, 
            () -> m_driveSubsystem.getPoseMeters(),
            m_driveSubsystem.kSwerveKinematics,
            new PIDController(m_ticks, m_ticks, m_ticks),
            new PIDController(m_ticks, m_ticks, m_ticks),
            new ProfiledPIDController(0.1, 0, 0, new Constraints(Math.PI, Math.PI)),
            (SwerveModuleState[] states) -> m_driveSubsystem.setDesiredState(states),
            m_driveSubsystem);
         m_pathCommad = command;
         m_pose = trajectory.getInitialPose();
         addCommands(
            new InstantCommand(() -> {
                startup();
            }),
            m_pathCommad,
            new InstantCommand(() -> {
                cleanup();
            })
         );

        }
        catch(Exception e){
            DriverStation.reportError(e.getMessage(), e.getStackTrace());
        }

        public void startup() {
            m_pose = new Pose2d(new Translation2d(m_pose.getX(), m_pose.getY()), new Rotation2d());
            m_driveSubsystem.resetModuleEncoders(m_pose);
        }

        public void cleanup(){
            m_driveSubsystem(0,0,0, false);
        }

    }









}
