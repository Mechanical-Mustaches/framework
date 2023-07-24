// package frc.robot.commands.Auto;

// import com.pathplanner.lib.PathPlanner;
// import com.pathplanner.lib.PathPlannerTrajectory;
// import com.pathplanner.lib.commands.PPSwerveControllerCommand;

// import edu.wpi.first.math.controller.PIDController;
// import edu.wpi.first.SwerveControllerCommand;
// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.math.util.Units;
// import edu.wpi.first.wpilibj2.command.InstantCommand;
// import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
// import frc.robot.subsystems.DriveSubsystem;

// public class followTrajectoryCommand extends SequentialCommandGroup{

//     public followTrajectoryCommand(DriveSubsystem swerveDrive){
        
//         PathPlannerTrajectory trajectory =
//             PathPlanner.loadPath("swerveTest", Units.feetToMeters(2), Units.feetToMeters(2), false);
//             PPSwerveControllerCommand command1 =
//             new PPSwerveControllerCommand(trajectory,
//                 swerveDrive::getPoseMeters,
//                 swerveDrive.getXPidController(),
//                 swerveDrive.getYPidController(),
//                 null, 
//                 swerveDrive::setSwerveModuleStatesAuto,
//                 swerveDrive);
//         addCommands(command1);

//     }
   
           
          


    
    
// }
