package frc.robot.commands.Compressor;

import frc.robot.subsystems.Compressor;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class Forward extends CommandBase{
    //Instance Variables
    private boolean compressor_Extended = false;
    Compressor compressor;
   
    //Constructor
    public Forward(Compressor compressor){
        //addRequirements(compressor);
        this.compressor = compressor;
    }

    public void execute(){
        compressor.Forward();
    }

    public boolean isCompressorExtended(){
        return compressor_Extended;
    }

}
