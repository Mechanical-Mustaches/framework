package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;

public class Compressor extends Subsystembase{
    //Instance Variables
    private DoubleSolenoid firstSolenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH, 0, 1);
    private DoubleSolenoid secondSolenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH, 3, 4);
    private String name;
    
    //Constructor 
    public Compressor(String name){
        this.name = name;
    }

    //Starting Command
    public void initalize(){
        System.out.println(name + "is booting up");
    }

    //Movement
    public void Forward(){
        firstSolenoid.set(Value.kForward);
        secondSolenoid.set(Value.kForward);
    }

    public void Backward(){
        firstSolenoid.set(Value.kReverse);
        secondSolenoid.set(Value.kReverse);
    }

    public void Off(){
        firstSolenoid.set(Value.kOff);
        secondSolenoid.set(Value.kOff);
    }

}

