package frc.robot.commands.Flywheel;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Flywheel;

public class  stop extends CommandBase {
    private boolean isstop;
    Flywheel flywheel;
    
    public stop(Flywheel flywheel){
        addRequirements(flywheel);
        this.flywheel = flywheel;
    }
public void execute(){
    flywheel.Stop();
}
public boolean isFlywheelStopped(){
    return isstop;
}
}
