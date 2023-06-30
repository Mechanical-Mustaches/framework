package frc.robot.commands.ArmIntake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmIntake;

public class Stop extends CommandBase{
    //Instance Variables
    private boolean armIntake_Stopped = false;
    ArmIntake armIntake;

    //Constructor 
    public Stop(ArmIntake armIntake){
        addRequirements(armIntake);
        this.armIntake = armIntake;
    }

    public void execute(){
        armIntake.Stop();
    }

    public boolean isIntakeStopped(){
        return armIntake_Stopped;
    }
}
