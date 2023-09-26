package frc.robot.commands.Converyorbrain;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.converyor;

public class RollUp extends CommandBase {
    converyor Converyor;

    private boolean is_converyorRolling;
    public RollUp(converyor Converyor){
     addRequirements(Converyor); 

     this.Converyor = Converyor;

    }
    public void execute (){
Converyor.RollUp();

    }

public boolean isconveryorRolling(){
return is_converyorRolling; 
// True ---> rolling, in process of action
// false ---> I have finished rolling move on to next mission
// one --> Nothing
}

}
