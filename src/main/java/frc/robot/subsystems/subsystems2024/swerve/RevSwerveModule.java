
package frc.robot.subsystems.subsystems2024.swerve;

import com.ctre.phoenix6.hardware.CANcoder;

 //ML Drycoded must test
//import com.revrobotics.CANSparkMax;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.ClosedLoopSlot;
//import com.revrobotics.CANSparkBase;
import com.revrobotics.spark.SparkBase;
//import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.spark.SparkBase.ControlType;
//import com.revrobotics.CANSparkBase.FaultID;
import com.revrobotics.spark.SparkBase.Faults;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
//import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.config.BaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.MAXMotionConfig;
import com.revrobotics.spark.config.SmartMotionConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.RelativeEncoder;
//import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.spark.SparkMaxAlternateEncoder;
//import com.revrobotics.SparkPIDController;
import com.revrobotics.spark.SparkClosedLoopController;


import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.util.swerveUtil.CTREModuleState;
import frc.lib.util.swerveUtil.RevSwerveModuleConstants;
import frc.robot.Constants2024;

import static frc.robot.Constants2024.Swerve.DegreesPerTurnRotation;
import static frc.robot.Constants2024.Swerve.angleGearRatio;
import static frc.robot.Constants2024.Swerve.swerveCANcoderConfig;

/**
 * a Swerve Modules using REV Robotics motor controllers and CTRE CANCoder absolute encoders.
 */
public class RevSwerveModule implements SwerveModule
{
    public int moduleNumber;
    private Rotation2d angleOffset;
    // private Rotation2d lastAngle;

    //private CANSparkMax mAngleMotor;
    //private CANSparkMax mDriveMotor;
    private SparkMax mAngleMotor;
    private SparkMax mDriveMotor;
    private SparkMaxConfig config;

    private CANcoder angleEncoder;
    private RelativeEncoder relAngleEncoder;
    private RelativeEncoder relDriveEncoder;

    public SwerveModuleState desiredState;
    private int angleCounter = 0;
    private double prevAngle = 0;
    private int CANCONFIG = 0;
    //SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(Constants.Swerve.driveKS, Constants.Swerve.driveKV, Constants.Swerve.driveKA);

    public RevSwerveModule(int moduleNumber, RevSwerveModuleConstants moduleConstants)
    {
        this.moduleNumber = moduleNumber;
        this.angleOffset = moduleConstants.angleOffset;


        /* Angle Motor Config */
        //mAngleMotor = new CANSparkMax(moduleConstants.angleMotorID, MotorType.kBrushless);
        mAngleMotor = new SparkMax(moduleConstants.angleMotorID, MotorType.kBrushless);
        configAngleMotor();
        if(mAngleMotor.getFaults() != null)
        {
            DriverStation.reportWarning("RevSwerveModule - Sensor Fault on Angle Motor ID:"+mAngleMotor.getDeviceId(), false);
        }
        /* Drive Motor Config */
        //mDriveMotor = new CANSparkMax(moduleConstants.driveMotorID,  MotorType.kBrushless);
        mDriveMotor = new SparkMax(moduleConstants.driveMotorID,  MotorType.kBrushless);
        configDriveMotor();
        if(mDriveMotor.getFaults() != null)
        {
            DriverStation.reportWarning("RevSwerveModule - Sensor Fault on Drive Motor ID:"+mDriveMotor.getDeviceId(), false);
        }

        /* Angle Encoder Config */

        angleEncoder = new CANcoder(moduleConstants.cancoderID);

        configEncoders();


        // lastAngle = getState().angle;
    }


    private void configEncoders()
    {
        // absolute encoder
        relDriveEncoder = mDriveMotor.getEncoder();
        relDriveEncoder.setPosition(0);

        // Comment out because we could not get smartMotion works 
        // We need this feature when using path planner & Trajectory
        // Need solution????
       // relDriveEncoder.setPositionConversionFactor(Constants.Swerve.driveRevToMeters);
        //relDriveEncoder.setVelocityConversionFactor(Constants.Swerve.driveRpmToMetersPerSecond);
        
        //ML New format to configure SparkMax for 2025 DRYCODED
        SparkMaxConfig config = new SparkMaxConfig();
        //config
        //.inverted(true)
        //.idleMode(IdleMode.kBrake);
        config.encoder
            //.positionConversionFactor(Constants.Swerve.driveRevToMeters)
            .velocityConversionFactor(Constants2024.Swerve.driveRpmToMetersPerSecond);
        //config.closedLoop
        //    .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        //    .pid(1.0, 0.0, 0.0);
        
        mDriveMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        relAngleEncoder = mAngleMotor.getEncoder();

        // Comment out because we could not get smartMotion work with conversion value
        //relAngleEncoder.setPositionConversionFactor(Constants.Swerve.DegreesPerTurnRotation);
        // in degrees/sec
        //relAngleEncoder.setVelocityConversionFactor(Constants.Swerve.DegreesPerTurnRotation / 60);

        //ML New format to configure SparkMax for 2025 DRYCODED
        SparkMaxConfig config2 = new SparkMaxConfig();
        //config2
        //.inverted(true)
        //.idleMode(IdleMode.kBrake);
        config2.encoder
            //.positionConversionFactor(Constants.Swerve.driveRevToMeters)
            .velocityConversionFactor(Constants2024.Swerve.DegreesPerTurnRotation / 60);
        //config2.closedLoop
        //    .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        //    .pid(1.0, 0.0, 0.0);
        
        mAngleMotor.configure(config2, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);


        synchronizeEncoders();
        
        // We don't need to burn the flash during the debug period.
        // It could damage the flash and short the life cycle of sparkMax

       // mDriveMotor.burnFlash();
      // mAngleMotor.burnFlash();

    }

    private void configAngleMotor()
    {
        // mAngleMotor.restoreFactoryDefaults();
        // SparkPIDController controller = mAngleMotor.getPIDController();
        
        // controller.setP(Constants.Swerve.angleKP, 0);
        // controller.setI(Constants.Swerve.angleKI,0);
        // controller.setD(Constants.Swerve.angleKD,0);
        // controller.setFF(Constants.Swerve.angleKFF,0);
        // controller.setOutputRange(-Constants.Swerve.anglePower, Constants.Swerve.anglePower);
        // mAngleMotor.setSmartCurrentLimit(Constants.Swerve.angleContinuousCurrentLimit);

        // mAngleMotor.setInverted(Constants.Swerve.angleMotorInvert);
        // mAngleMotor.setIdleMode(Constants.Swerve.angleIdleMode);
        // mAngleMotor.setClosedLoopRampRate(Constants.Swerve.angleRampRate);

        // controller.setSmartMotionMinOutputVelocity(Constants.Swerve.minVel, 0);
        // controller.setSmartMotionMaxVelocity(Constants.Swerve.maxAngleVel, 0);
        // controller.setSmartMotionMaxAccel(Constants.Swerve.maxAngleAccVel, 0);
        // controller.setSmartMotionAllowedClosedLoopError(Constants.Swerve.allowedAngleErrVel, 0);

        // controller.setP(Constants.Swerve.angleKP, 1);
        // controller.setI(Constants.Swerve.angleKI,1);
        // controller.setD(Constants.Swerve.angleKD,1);
        // controller.setFF(Constants.Swerve.angleKFF,1);
        // controller.setSmartMotionMinOutputVelocity(Constants.Swerve.minVel, 1);
        // controller.setSmartMotionMaxVelocity(Constants.Swerve.maxAnglePos, 1);
        // controller.setSmartMotionMaxAccel(Constants.Swerve.maxAngleAccPos, 1);
        // controller.setSmartMotionAllowedClosedLoopError(Constants.Swerve.allowedAngleErrPos, 1);


        SparkClosedLoopController controller = mAngleMotor.getClosedLoopController();
        config = new SparkMaxConfig();
        
        config
        .inverted(Constants2024.Swerve.angleMotorInvert)
        .idleMode(Constants2024.Swerve.angleIdleMode)
        .smartCurrentLimit(Constants2024.Swerve.angleContinuousCurrentLimit)
        .closedLoopRampRate(Constants2024.Swerve.angleRampRate)
        ;
        //config.encoder  // Not sure if needed
            //.positionConversionFactor(Constants.Swerve.driveRevToMeters)
            //.velocityConversionFactor(Constants.Swerve.driveRpmToMetersPerSecond)
            //;
        config.closedLoop
            //.feedbackSensor(FeedbackSensor.kPrimaryEncoder) // Not sure if needed
            .p(Constants2024.Swerve.angleKP)
            .i(Constants2024.Swerve.angleKI)
            .d(Constants2024.Swerve.angleKD)
            .velocityFF(Constants2024.Swerve.angleKFF)
            .outputRange(-Constants2024.Swerve.anglePower, Constants2024.Swerve.anglePower)
            .p(Constants2024.Swerve.angleKP,ClosedLoopSlot.kSlot1)
            .i(Constants2024.Swerve.angleKI,ClosedLoopSlot.kSlot1)
            .d(Constants2024.Swerve.angleKD,ClosedLoopSlot.kSlot1)
            .velocityFF(Constants2024.Swerve.angleKFF,ClosedLoopSlot.kSlot1)
            .maxMotion
                //.minOutputVelocity(Constants.Swerve.minVel) // Not available in maxMotion
                .maxVelocity(Constants2024.Swerve.maxAngleVel)
                .maxAcceleration(Constants2024.Swerve.maxAngleAccVel)
                .allowedClosedLoopError(Constants2024.Swerve.allowedAngleErrVel)
                .maxVelocity(Constants2024.Swerve.maxAngleVel,ClosedLoopSlot.kSlot1)
                .maxAcceleration(Constants2024.Swerve.maxAngleAccVel,ClosedLoopSlot.kSlot1)
                .allowedClosedLoopError(Constants2024.Swerve.allowedAngleErrPos,ClosedLoopSlot.kSlot1)
            ;
        
        mAngleMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        if(mAngleMotor.getFaults() != null)
        {
            DriverStation.reportWarning("configAngleMotor - Sensor Fault on Angle Motor ID:"+mAngleMotor.getDeviceId(), false);
        }

    }

    private void configDriveMotor()
    {
         //mDriveMotor.restoreFactoryDefaults();
        //SparkPIDController controller = mDriveMotor.getPIDController();
        SparkClosedLoopController controller = mDriveMotor.getClosedLoopController();
        //controller.setOutputRange(-Constants.Swerve.drivePower, Constants.Swerve.drivePower);
        //mDriveMotor.setSmartCurrentLimit(Constants.Swerve.driveContinuousCurrentLimit);
        //mDriveMotor.setInverted(Constants.Swerve.driveMotorInvert);
        //mDriveMotor.setIdleMode(Constants.Swerve.driveIdleMode);

        // Speed control parameter is on slot 0
        //controller.setP(Constants.Swerve.driveKP_v,0);
        //controller.setI(Constants.Swerve.driveKI,0);
        //controller.setD(Constants.Swerve.driveKD,0);
        //controller.setFF(Constants.Swerve.driveKFF,0);
        //controller.setSmartMotionMinOutputVelocity(Constants.Swerve.minVel, 0);
        //controller.setSmartMotionMaxVelocity(Constants.Swerve.maxDriveVel, 0);
        //controller.setSmartMotionMaxAccel(Constants.Swerve.maxDriveAccVel, 0);
        //controller.setSmartMotionAllowedClosedLoopError(Constants.Swerve.allowedDriveErrVel, 0);
    
        // position control is on PID parameter slot 1

        //controller.setP(Constants.Swerve.driveKP_p,1);
        //controller.setI(Constants.Swerve.driveKI,1);
        //controller.setD(Constants.Swerve.driveKD,1);
        //controller.setFF(Constants.Swerve.driveKFF,1);
        //controller.setSmartMotionMinOutputVelocity(Constants.Swerve.minVel, 1);
        //controller.setSmartMotionMaxVelocity(Constants.Swerve.maxDrivePos, 1);
        //controller.setSmartMotionMaxAccel(Constants.Swerve.maxDriveAccPos, 1);
        //controller.setSmartMotionAllowedClosedLoopError(Constants.Swerve.allowedDriveErrPos, 1);

        config = new SparkMaxConfig();
        
        config
        .inverted(Constants2024.Swerve.driveMotorInvert)
        .idleMode(Constants2024.Swerve.driveIdleMode)
        .smartCurrentLimit(Constants2024.Swerve.driveContinuousCurrentLimit)
        ;
        //config.encoder  // Not sure if needed
            //.positionConversionFactor(Constants.Swerve.driveRevToMeters)
            //.velocityConversionFactor(Constants.Swerve.driveRpmToMetersPerSecond)
            //;
        config.closedLoop
            //.feedbackSensor(FeedbackSensor.kPrimaryEncoder) // Not sure if needed
            // Speed control parameter is on slot 0
            .p(Constants2024.Swerve.driveKP_v)
            .i(Constants2024.Swerve.driveKI)
            .d(Constants2024.Swerve.driveKD)
            .velocityFF(Constants2024.Swerve.driveKFF)
            .outputRange(-Constants2024.Swerve.drivePower, Constants2024.Swerve.drivePower)
            // position control is on PID parameter slot 1
            .p(Constants2024.Swerve.driveKP_p,ClosedLoopSlot.kSlot1)
            .i(Constants2024.Swerve.driveKI,ClosedLoopSlot.kSlot1)
            .d(Constants2024.Swerve.driveKD,ClosedLoopSlot.kSlot1)
            .velocityFF(Constants2024.Swerve.driveKFF,ClosedLoopSlot.kSlot1)
            .maxMotion
                //.minOutputVelocity(Constants.Swerve.minVel) // Not available in maxMotion
                .maxVelocity(Constants2024.Swerve.maxDriveVel)
                .maxAcceleration(Constants2024.Swerve.maxDriveAccVel)
                .allowedClosedLoopError(Constants2024.Swerve.allowedDriveErrVel)
                .maxVelocity(Constants2024.Swerve.maxDrivePos,ClosedLoopSlot.kSlot1)
                .maxAcceleration(Constants2024.Swerve.maxDriveAccPos,ClosedLoopSlot.kSlot1)
                .allowedClosedLoopError(Constants2024.Swerve.allowedDriveErrPos,ClosedLoopSlot.kSlot1)
            ;
        
        mDriveMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        if(mDriveMotor.getFaults() != null)
        {
            DriverStation.reportWarning("configDriveMotor - Sensor Fault on Drive Motor ID:"+mDriveMotor.getDeviceId(), false);
        }


    }

    public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop)
    {
        /* This is a custom optimize function, since default WPILib optimize assumes continuous controller which CTRE and Rev onboard is not */
        // CTREModuleState actually works for any type of motor.

        // System.out.println("Start of setDesiredState");

        this.desiredState = CTREModuleState.optimize(desiredState, getState().angle);
        // System.out.println("passed this.desiredState = CTREModuleState.optimize(desiredState, getState().angle);");
        setAngle(this.desiredState);
        
        // System.out.println("setAngle(this.desiredState);");
        // Set the isOpenLoop to false so we can trigger the PID control for the velocity
        setSpeed(this.desiredState, false);
        // System.out.println("setSpeed(this.desiredState, false);");
        // System.out.println("End of setDesiredState");
        //if(mDriveMotor.getFault(FaultID.kSensorFault))
        if(mDriveMotor.getFaults() != null)
        {
            // DriverStation.reportWarning("setDesiredState - Sensor Fault on Drive Motor ID:"+mDriveMotor.getDeviceId(), false);
        }

        //if(mAngleMotor.getFault(FaultID.kSensorFault))
        if(mAngleMotor.getFaults() != null)
        {
            // DriverStation.reportWarning("setDesiredState - Sensor Fault on Angle Motor ID:"+mAngleMotor.getDeviceId(), false);
        }
    }

    /**
     * A modified version of {@link #setDesiredState(SwerveModuleState, boolean)} <code>By Robin</code>
     * @param desiredState
     */
    public void setOptimizedAngle(SwerveModuleState desiredState, double angleDegree) {
        this.desiredState = CTREModuleState.optimize(desiredState, getState().angle);
        setAngle(this.desiredState);
        // Set the isOpenLoop to false so we can trigger the PID control for the velocity

        //setPositionOtpimized(angleDegree);

        //if(mDriveMotor.getFault(FaultID.kSensorFault))
        if(mDriveMotor.getFaults() != null)
        {
            DriverStation.reportWarning("Sensor Fault on Drive Motor ID:"+mDriveMotor.getDeviceId(), false);
        }

        //if(mAngleMotor.getFault(FaultID.kSensorFault))
        if(mAngleMotor.getFaults() != null)
        {
            DriverStation.reportWarning("Sensor Fault on Angle Motor ID:"+mAngleMotor.getDeviceId(), false);
        }
    }

    private void setSpeed(SwerveModuleState desiredState, boolean isOpenLoop)
    {

        if(isOpenLoop)
        {
            double percentOutput = desiredState.speedMetersPerSecond / Constants2024.Swerve.maxSpeed;
            mDriveMotor.set(percentOutput);
            return;
        }

        double velocity = desiredState.speedMetersPerSecond;

        //SparkPIDController controller = mDriveMotor.getPIDController();
        SparkClosedLoopController controller = mDriveMotor.getClosedLoopController();
        // Enable the smart Velocity control, which can give us manageable acceleration
        // Otherwise, the motor start/stop abruptly, will damage the motor/gear
        //controller.setReference(velocity, ControlType.kSmartVelocity, 0);
        controller.setReference(velocity, ControlType.kMAXMotionVelocityControl);
    }


    public void setAngle(SwerveModuleState desiredState)
    {
        // Angle recieve shall be Chassis angle
        // absolution value counter from chassis 0

        double setAngle,currentAngle,finalAngle;
        // See angle require a speed which does not make sense
        // Comment it out
        //if(Math.abs(desiredState.speedMetersPerSecond) <= (Constants.Swerve.maxSpeed * 0.01))
        //{
        //    mAngleMotor.stopMotor();
        //    return;
        //}
        //Prevent rotating module if speed is less then 1%. Prevents Jittering.


        // Sync encoder when we need to align the wheel position 
        // Need to test to confirm if it is working.
        synchronizeEncoders();
        Rotation2d angle = desiredState.angle;
       
        //SparkPIDController controller = mAngleMotor.getPIDController();
        SparkClosedLoopController controller = mDriveMotor.getClosedLoopController();
        // Calculate in degree
        setAngle = angle.getDegrees();
        if(setAngle >=360) setAngle = setAngle - 360;
        currentAngle = mAngleMotor.getEncoder().getPosition() * Constants2024.Swerve.DegreesPerTurnRotation;
        if(currentAngle > 360) currentAngle = 360 - currentAngle;
        else if (currentAngle < 0) currentAngle = 360 + currentAngle;
        double deltaAngle = Math.abs(setAngle - currentAngle);

        if(deltaAngle > 180)     // Need to turn move than half round
        {   
            
                if(currentAngle >= setAngle )           //Clockwise
                {
                        finalAngle = currentAngle + 360 - deltaAngle;
                }
                else                                    //Counter Clockwise
                {
                        finalAngle = currentAngle - (360 - deltaAngle);
                }

        }
        else
        {
            finalAngle = setAngle;
        }
        // input angle is degree (0~360), need to convert back to encoder raw position
        // /15 is a experience value from the reading. Need to fine tuning this value

        //controller.setReference (finalAngle / Constants.Swerve.DegreesPerTurnRotation, ControlType.kSmartMotion, 1);
        controller.setReference(finalAngle / Constants2024.Swerve.DegreesPerTurnRotation, ControlType.kMAXMotionPositionControl);
        SmartDashboard.putNumber("Angle Counter",angleCounter++);
         SmartDashboard.putNumber("set angle" + this.moduleNumber,setAngle);
         SmartDashboard.putNumber("currentangle"+ this.moduleNumber,currentAngle);
         SmartDashboard.putNumber("delta angle"+ this.moduleNumber,deltaAngle);
         SmartDashboard.putNumber("Final angle"+ this.moduleNumber,finalAngle);

    }

    public Rotation2d getAngle()
    {
        double returnAngle = relAngleEncoder.getPosition() * Constants2024.Swerve.DegreesPerTurnRotation;

        if(returnAngle > 360) returnAngle = returnAngle -360;
        else if(returnAngle < 0) returnAngle = returnAngle + 360;
        return Rotation2d.fromDegrees(returnAngle);
    }

    public Rotation2d getCanCoder()
    {

        return Rotation2d.fromDegrees(angleEncoder.getAbsolutePosition().getValueAsDouble() * 360);
        //return getAngle();
    }

    public int getModuleNumber()
    {
        return moduleNumber;
    }

    public void setModuleNumber(int moduleNumber)
    {
        this.moduleNumber = moduleNumber;
    }

    // CANCODER setting is default -180 to 180. It could be set from 0 ~ 360
    // The following change is assume the CANCODER is -180to180
    // The code is to round the value between -180 to 180 for NeoEncoder
    public void synchronizeEncoders()
    {
        double NeoEncoderPosition = 0;
        double absolutePosition = 0; 
        double NeoEncoderRawPosition = 0;
        double syncDelayCounter = 0;
        double currentPosition = 0;
        // if can is configure to output -180 to 180
        absolutePosition = getCanCoder().getDegrees() - angleOffset.getDegrees();
        if(absolutePosition >=0)
        {
            NeoEncoderPosition = absolutePosition;
        }
        else
        {
            NeoEncoderPosition = 360 + absolutePosition;
        }
        if(CANCONFIG == 180 && NeoEncoderPosition >=180)
        {
                    NeoEncoderPosition = NeoEncoderPosition - 360;
        }
        NeoEncoderRawPosition = NeoEncoderPosition/DegreesPerTurnRotation;
        SmartDashboard.putNumber("CAN" + this.moduleNumber, getCanCoder().getDegrees());
        SmartDashboard.putNumber("ABS" + this.moduleNumber, absolutePosition);
        SmartDashboard.putNumber("CA1" + this.moduleNumber, NeoEncoderPosition);
        SmartDashboard.putNumber("CA2" + this.moduleNumber, NeoEncoderRawPosition);
            // CanCoder return degree, need to convert back to Neo Raw position
        relAngleEncoder.setPosition(NeoEncoderRawPosition);
        while(Math.abs(relAngleEncoder.getPosition() - NeoEncoderRawPosition)> 0.5)
        {
            syncDelayCounter ++;
            if(syncDelayCounter>3000) break;

        }
        SmartDashboard.putNumber("CA3" + this.moduleNumber, relAngleEncoder.getPosition()); 
        SmartDashboard.putNumber("SyncDelayCounter" + this.moduleNumber, syncDelayCounter); 

    }

    public SwerveModuleState getState()
    {
        return new SwerveModuleState(
                relDriveEncoder.getVelocity(),
                getAngle()
        );
    }

    public double getOmega()
    {
        return angleEncoder.getVelocity().getValueAsDouble()/360;
    }

    public SwerveModulePosition getPosition()
    {
        return new SwerveModulePosition(
                relDriveEncoder.getPosition() * Constants2024.Swerve.driveRevToMeters,
                getAngle()
        );
    }
 
    public void setPosition(double position)
    {
        // Not sure if we need this sync
        //synchronizeEncoders();
        //SparkPIDController controller = mDriveMotor.getPIDController();
        SparkClosedLoopController controller = mDriveMotor.getClosedLoopController();
        double encoderDelta = position / Constants2024.Swerve.driveRevToMeters;
        // Use raw encode position
        double currentPosition = mDriveMotor.getEncoder().getPosition();
        // Try to match the joystick direction
        //controller.setReference (currentPosition - encoderDelta, ControlType.kSmartMotion,1);
        controller.setReference(currentPosition -encoderDelta, ControlType.kMAXMotionPositionControl);
        SmartDashboard.putNumber("SetPosition",encoderDelta);

    }

    /**
     * Based on the optimized angle, turn - or +
     * @param position
     */
    public void setPositionOtpimized(double position)
    {
        // Not sure if we need this sync
        //synchronizeEncoders();
        //SparkPIDController controller = mDriveMotor.getPIDController();
        SparkClosedLoopController controller = mDriveMotor.getClosedLoopController();
        if (desiredState.speedMetersPerSecond < 0) { 
            position = -position;
        }
        double encoderDelta = position / Constants2024.Swerve.driveRevToMeters;
        // Use raw encode position
        double currentPosition = mDriveMotor.getEncoder().getPosition();
        // Try to match the joystick direction
        
            //controller.setReference (currentPosition - encoderDelta, ControlType.kSmartMotion,1);
            controller.setReference(currentPosition -encoderDelta, ControlType.kMAXMotionPositionControl);
        
        
        SmartDashboard.putNumber("SetPosition",encoderDelta);

    }

}
