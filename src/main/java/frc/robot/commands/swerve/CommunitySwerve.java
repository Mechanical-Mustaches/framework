package frc.robot.commands.swerve;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.commands.LoggingCommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class CommunitySwerve extends LoggingCommandBase{
    private DriveSubsystem m_drive;

    public CommunitySwerve(DriveSubsystem m_drive){
        this.m_drive = m_drive;
        addRequirements(m_drive);
    }

    @Override
    public void execute(){
        m_drive.drive(-0.3, 0, 0, true);
    }

    @Override
    public void initialize(){
        super.initialize();
        m_drive.setOdometry(new Pose2d(new Translation2d(0.0, 0.0), new Rotation2d()));
    }

    public boolean isFinished(){
        boolean finished = m_drive.getPoseMeters().getX() < -2.50;
        if(finished){
            System.out.println(this.getClass().getName() + " is finished!");
            m_drive.drive(0, 0, 0, false);
        }
        return finished;
    }

} // <--
