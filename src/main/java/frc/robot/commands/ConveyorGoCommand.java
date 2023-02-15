package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Conveyor;

public class ConveyorGoCommand extends CommandBase{
    Conveyor conveyor;

    public ConveyorGoCommand(Conveyor conveyor){
        this.conveyor = conveyor;
    }

    public void execute(){
        conveyor.go();
    }

} // <-- kep brace 
