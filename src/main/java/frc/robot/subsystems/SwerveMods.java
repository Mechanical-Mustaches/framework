package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.CTRECanCoder;
import frc.robot.Constants.ModuleConstants;
import frc.robot.utils.AngleUtils;

public class SwerveMods extends SubsystemBase{
    public final CANSparkMax m_driveMotor;
    public final CANSparkMax m_turnMotor;

    public final RelativeEncoder m_driveEncoder;
    public final RelativeEncoder m_turnEncoder;

    private final SparkMaxPIDController m_drivePIDController;
    private final SparkMaxPIDController m_turnPIDController;

    public final CTRECanCoder m_turnCANcoder;

    public int m_moduleNumber;

    private double m_turnEncoderOffset;

    private boolean m_isOpenLoop;

    private final int m_locationIndex;

    private double m_lastAngle;
    public double angle;


      /**
   * Constructs a SwerveModule.
   *
   * @param driveMotorChannel       The channel of the drive motor.
   * @param turningMotorChannel     The channel of the turning motor.
   * @param driveEncoderChannels    The channels of the drive encoder.
   * @param turningCANCoderChannels The channels of the turning encoder.
   * @param driveEncoderReversed    Whether the drive encoder is reversed.
   * @param turningEncoderReversed  Whether the turning encoder is reversed.
   * @param turningEncoderOffset
   */
    public SwerveMods(
        int locationIndex,
        int driveMotorCANChannel,
        int turningMotorCANChannel,
        int cancoderCANChannel,
        boolean driveMotorReversed,
        boolean turningMotorReversed,
        int pdpDriveChannel,
        int pdpTurnChannel,
        boolean isOpenLoop,
        double turningEncoderOffset) {
        
        m_locationIndex = locationIndex;

        m_isOpenLoop = isOpenLoop;

        m_driveMotor = new CANSparkMax(driveMotorCANChannel, MotorType.kBrushless);
        m_turnMotor = new CANSparkMax(turningMotorCANChannel, MotorType.kBrushless);

        m_driveMotor.restoreFactoryDefaults();
        m_turnMotor.restoreFactoryDefaults();

        m_driveEncoder = m_driveMotor.getEncoder();
        m_turnEncoder = m_turnMotor.getEncoder();

        m_drivePIDController = m_driveMotor.getPIDController();
        m_turnPIDController = m_turnMotor.getPIDController();
        m_drivePIDController.setFeedbackDevice(m_driveEncoder);
        m_turnPIDController.setFeedbackDevice(m_turnEncoder);

        m_turnCANcoder = new CTRECanCoder(cancoderCANChannel);
        m_turnCANcoder.configFactoryDefault();
        m_turnCANcoder.configAllSettings(AngleUtils.generateCanCoderConfig());

        m_turnEncoderOffset = turningEncoderOffset;

        m_driveMotor.setInverted(driveMotorReversed);
        m_turnMotor.setInverted(turningMotorReversed);
        
        m_driveEncoder.setPositionConversionFactor(ModuleConstants.kDriveMetersPerEncRev);
        m_driveEncoder.setVelocityConversionFactor(ModuleConstants.kDriveMetersPerEncRev / 60);

        m_turnEncoder.setPositionConversionFactor(ModuleConstants.kTurningDegreesPerEncRev);
        m_turnEncoder.setVelocityConversionFactor(ModuleConstants.kTurningDegreesPerEncRev / 60);

        
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(m_driveEncoder.getVelocity(),
            new Rotation2d(m_turnEncoder.getPosition()));
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(getDriveVelocity(), getHeadingRotation2d());
    }

    public double getHeadingDeg() {
        return getTurnAngleDegs();
    }

    private Rotation2d getHeadingRotation2d() {
        return Rotation2d.fromDegrees(getHeadingDeg());
    }

    public double getTurnAngleDegs(){
        if(RobotBase.isReal()){
            return m_turnEncoder.getPosition();
        }
        else{
            return angle;
        }
    }

    private double getDriveVelocity() {
        if(RobotBase.isReal()){
            return m_driveEncoder.getVelocity();
        }
        else{
            return (m_driveEncoder.getVelocity() * ModuleConstants.kDriveMetersPerEncRev) / 60;
        }
    }
    
}
