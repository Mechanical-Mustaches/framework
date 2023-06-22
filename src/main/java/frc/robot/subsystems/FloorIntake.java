package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
// import com.revrobotics.RelativeEncoder;


public class FloorIntake extends SubsystemBase{
    // Instance variables
    private DoubleSolenoid right = new DoubleSolenoid((PneumaticsModuleType.REVPH), 1, 2);
    private DoubleSolenoid left = new DoubleSolenoid((PneumaticsModuleType.REVPH), 3, 4);
    private CANSparkMax flrInMotor = new CANSparkMax(99, MotorType.kBrushless);
    private double motorSpeed = 0.5;
    // private RelativeEncoder encoder;

    // Constructor
    public FloorIntake() {
        
    }

    // Intake movement
    public void extend() {
        right.set(Value.kForward);
        left.set(Value.kForward);

        flrInMotor.set(motorSpeed);
    }

    public void retract() {
        right.set(Value.kReverse);
        left.set(Value.kReverse);

        flrInMotor.set(-motorSpeed);
    }

    public void stop() {
        right.set(Value.kOff);
        left.set(Value.kOff);

        flrInMotor.set(0);
    }

}
