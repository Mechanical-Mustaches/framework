package frc.robot.commands.Elevator;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Elevator;

public class MoveUp extends CommandBase{
    //Instance Variables
    private boolean iselevator_extended;
    Elevator elevator;

    //Constructor
    public MoveUp(Elevator elevator){
        addRequirements(elevator);
        this.elevator = elevator;
    }

    public void execute(){
        elevator.moveUp();
    }
    
    public boolean isElevatorExtended(){
        return iselevator_extended;
    }
    
}
