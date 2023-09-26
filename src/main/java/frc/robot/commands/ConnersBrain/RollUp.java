package frc.robot.commands.ConnersBrain;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Conveyor;

public class RollUp extends CommandBase{
    Conveyor conveyor;
    private boolean is_ConveyorRolling;

    public RollUp(Conveyor conveyor){
        addRequirements(conveyor);

    }
    
}
