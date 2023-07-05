package frc.robot.commands.Conveyor;

import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.subsystems.Conveyor;

public class Forward extends CommandBase {
    private boolean isConveyor_Extended;
    Conveyor conveyor;

    public Forward(Conveyor conveyor){
        addRequirements(conveyor);
        this.conveyor = conveyor;
    }

    public void execute(){
        conveyor.Forward();
    }

    public boolean isConveyorExtended(){
        return isConveyor_Extended;
    }

}
