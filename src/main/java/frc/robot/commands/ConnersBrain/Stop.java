package frc.robot.commands.ConnersBrain;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Conveyor;

public class Stop extends CommandBase {
    private boolean isstop;
    Conveyor conveyor;

    public Stop(Conveyor conveyor){
        addRequirements(conveyor);
        this.conveyor = conveyor;
    }
public void execute(){
    conveyor.Stop();
}
public boolean isConveyorStopper(){
    return isstop;
}
}

