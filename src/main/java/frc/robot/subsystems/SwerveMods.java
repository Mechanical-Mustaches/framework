package frc.robot.subsystems;

import com.revrobotics.AnalogInput;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import frc.robot.Constants.ModuleConstants;

public class SwerveMods {
    //Instance Variables
    private final CANSparkMax driveMotor;
    private final CANSparkMax turnMotor;

    private final RelativeEncoder driveEncoder;
    private final RelativeEncoder turnEncoder;

    private final PIDController turnPIDController;

    private final AnalogInput absoluteEncoder;
    private final boolean absoluteEncoderReversed;
    private final double absoluteEncoderOffsetRad;

    public SwerveMods(int driveMotorID, int turnMotorID, boolean driveMotorReversed, 
        boolean turningMotorReversed, int absoluteEncoderID, double absoluteEncoderOffset,
        boolean absoluteEncoderReversed){

            this.absoluteEncoderOffsetRad = absoluteEncoderOffsetRad;
            this.absoluteEncoderReversed = absoluteEncoderReversed;
            absoluteEncoder = new AnalogInput(absoluteEncoderID);

            driveMotor = new CANSparkMax(driveMotorID, MotorType.kBrushless);
            turnMotor = new CANSparkMax(turnMotorID, MotorType.kBrushless);

            driveMotor.setInverted(driveMotorReversed);
            turnMotor.setInverted(turningMotorReversed);

            driveEncoder.setPositionConversionFactor(ModuleConstants.kDriveEncoderRot2Meter);

    }



}
