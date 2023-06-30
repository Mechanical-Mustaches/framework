package frc.robot.commands.ArmIntake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmIntake;

public class Reverse extends CommandBase {
    //Instance Variables
    private boolean armIntake_Retract = false;
    ArmIntake armIntake;

    //Constructor
    public Reverse(ArmIntake armIntake){
        addRequirements(armIntake);
        this.armIntake = armIntake;
    }

    public void execute(){
        armIntake.Reverse();
    }

    public boolean isIntakeExtended(){
        return armIntake_Retract;
    }
}
