// package frc.robot.commands.Auto;
// import java.util.HashMap;

// import com.pathplanner.lib.PathConstraints;
// import com.pathplanner.lib.PathPlanner;
// import com.pathplanner.lib.auto.PIDConstants;
// import com.pathplanner.lib.auto.SwerveAutoBuilder;

// import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.CommandBase;
// import frc.robot.subsystems.DriveSubsystem;

// public final class NewAuto {

//         public static Command runPath(DriveSubsystem driveSubsystem){
//             var trajectory = PathPlanner.loadPath("Basic", PathConstraints(4,1));
//             SwerveAutoBuilder builder = new SwerveAutoBuilder(
//                 driveSubsystem::getPoseMeters,
//                 driveSubsystem::resetModuleEncoders(),
//                 new PIDConstants(0.7, 0.0001, 0),
//                 new PIDConstants(0.1, 0.0001, 0),
//                  driveSubsystem::setOdometry,
//                 new HashMap<String, Command>(),
//                 true,
//                 driveSubsystem
//             );

//             Command fullAuto = builder.fullAuto(trajectory);

//             return fullAuto;
//         }    

//         private NewAuto(){
//             throw new UnsupportedOperationException("This is a utility class!");
//         }
// }
