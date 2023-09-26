package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

/*
 * converyor:
 * Motor: 10 :)
 * Functions:
 * Roll up 
 * reverse
 */


public class converyor extends SubsystemBase{
    // Private: ONLY THE CLASS IT
    // Public: EVERYONE CAN SEE IT
private String name;

private CANSparkMax fire = new CANSparkMax(10, MotorType.kBrushless);

// Double: decimal --> .0 .001 .002
// Int:whole number --> 1, 34, 420
// Boolean: True or False
//Double, Ints and Boolean ALL RETURN SOMETHING
private double fast;


//Voids DONT RETURN ANYTHING --> setting varibles to things

public converyor (String name, double fast){
    this.name = name;
    this.fast = fast;
}

public void RollUp(){


fire.set(fast);
}

public void Revdown(){
    fire.set(-fast);
}

public void Stop () {
 fire.set(0);
}

}
