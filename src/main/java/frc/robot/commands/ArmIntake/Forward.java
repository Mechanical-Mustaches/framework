package frc.robot.commands.ArmIntake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmIntake;

public class Forward extends CommandBase{
    //Instance Variables
    private boolean armIntake_Extended = false;
    ArmIntake armIntake;

    //Constructor
    public Forward(ArmIntake armIntake){
        addRequirements(armIntake);
        this.armIntake = armIntake;
    }

    //Executing funcion
    public void execute(){
        armIntake.Forward();
    }

    //Checking to see if function is true or false
    public boolean isIntakeExtended(){
        return armIntake_Extended;
    }

    //Ending command for cycle (needed for auto)
    // public boolean isFinished(){
    //     return 
    // }

}
