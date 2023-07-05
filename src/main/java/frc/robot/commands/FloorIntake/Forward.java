package frc.robot.commands.FloorIntake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.FloorIntake;

public class Forward extends CommandBase{
    //Instance Variables 
    private boolean IntakeExtended;
    FloorIntake floorIntake;

    //Constructor
    public Forward(FloorIntake floorIntake){
        addRequirements(floorIntake);
        this.floorIntake = floorIntake;
    }

    public void execute(){
        floorIntake.Forward();
    }

    public boolean isIntakeExtended(){
        return IntakeExtended;
    }
}
