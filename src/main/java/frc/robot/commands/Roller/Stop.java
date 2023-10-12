package frc.robot.commands.Roller;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Shooter;

public class Stop extends CommandBase {
    private boolean isStopping;
    Shooter conveyor;  

    public Stop(Shooter conveyor){
        addRequirements(conveyor);
        this.conveyor = conveyor;
    }

    public void execute(){
        conveyor.Stop();
    }

    public boolean isShooterStopped(){
        return isStopping;
    }
    

}
