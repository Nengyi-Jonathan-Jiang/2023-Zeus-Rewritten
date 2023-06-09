package frc.robot.subsystems.Swerve;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

public class SwerveModule {
    private final WPI_TalonFX driver;
    private final WPI_TalonFX rotator;

    private final CANCoder absoluteEncoder;
    private final boolean shouldReverseEncoder;
    private final double encoderCorrection;

    private final StatorCurrentLimitConfiguration StatorCurrentLimitD;
    // private final SupplyCurrentLimitConfiguration SupplyCurrentLimitD;
    // private final StatorCurrentLimitConfiguration StatorCurrentLimitR;
    // private final SupplyCurrentLimitConfiguration SupplyCurrentLimitR;

    private final PIDController rotatorPID;
    public SwerveModule(int driverID, int rotatorID, boolean driveMotorReversed, boolean rotatorMotorReversed,
                        int absoluteEncoderID, double encoderCorrection, boolean shouldReverseEncoder){
        driver = new WPI_TalonFX(driverID, "DriveTrain");
        
        rotator = new WPI_TalonFX(rotatorID, "DriveTrain");
        driver.configFactoryDefault();
        rotator.configFactoryDefault();
        driver.setInverted(driveMotorReversed);
        rotator.setInverted(rotatorMotorReversed);
        setCoastMode();
        driver.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
        rotator.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
        StatorCurrentLimitD = new StatorCurrentLimitConfiguration(true, 33, 32, 0.01);
        // SupplyCurrentLimitD = new SupplyCurrentLimitConfiguration(true, 32, 31, 0.01);
        // StatorCurrentLimitR = new StatorCurrentLimitConfiguration(true, 32, 31, 0.01);
        // SupplyCurrentLimitR = new SupplyCurrentLimitConfiguration(true, 32, 31, 0.01);
        driver.configStatorCurrentLimit(StatorCurrentLimitD);
        // driver.configSupplyCurrentLimit(SupplyCurrentLimitD);
        // rotator.configStatorCurrentLimit(StatorCurrentLimitR);
        // rotator.configSupplyCurrentLimit(SupplyCurrentLimitR);

       

        absoluteEncoder = new CANCoder(absoluteEncoderID, "DriveTrain");
        absoluteEncoder.configAbsoluteSensorRange(AbsoluteSensorRange.Signed_PlusMinus180);
        this.encoderCorrection = encoderCorrection;
        this.shouldReverseEncoder = shouldReverseEncoder;

        rotatorPID = new PIDController(0.425, 0.25, 0);//change 0.425 0.28
        rotatorPID.enableContinuousInput(-Math.PI, Math.PI);

        resetEncoders();
    }

    public void setAutonCurrent(){
         StatorCurrentLimitConfiguration StatorCurrentLimitD = new StatorCurrentLimitConfiguration(false, 33, 32, 0.01);
         driver.configStatorCurrentLimit(StatorCurrentLimitD);
        }
    public void setTeleopCurrent(){
        StatorCurrentLimitConfiguration StatorCurrentLimitD = new StatorCurrentLimitConfiguration(true, 33, 32, 0.01);
        driver.configStatorCurrentLimit(StatorCurrentLimitD);
    }
    public double getDriverEncoderDistance(){
        return driver.getSelectedSensorPosition()* Constants.Swerve.DriverEncoderRatio;
    }
    public double getDriverEncoderVelocity(){
        return driver.getSelectedSensorVelocity()* Constants.Swerve.DriverEncoderRatio;
    }
    public double getRotatorEncoderRotation(){
        return rotator.getSelectedSensorPosition()* Constants.Swerve.RotatorEncoderRatio;
    }
    public double getRotatorEncoderVelocity(){
        return rotator.getSelectedSensorVelocity()* Constants.Swerve.RotatorEncoderRatio;
    }
    public double getAbsoluteEncoderRotation(){
        double pos =absoluteEncoder.getAbsolutePosition()- encoderCorrection;
        pos = Math.toRadians(pos);
        return pos*(shouldReverseEncoder ?-1:1);

    }
    public double getAbsoluteEncoderRotationDegrees(){
        double pos =absoluteEncoder.getAbsolutePosition()- encoderCorrection;
        // pos = Math.toRadians(pos);
        return pos*(shouldReverseEncoder ?-1:1);

    }
    public double getRawAbsoluteEncoderPosition(){
        return absoluteEncoder.getAbsolutePosition();
    }
    public void resetEncoders(){
        resetDriveEncoder(0);
        resetRotatorEncoder(getAbsoluteEncoderRotation());
    }
    public void resetDriveEncoder(double pos){
        driver.setSelectedSensorPosition(pos);
    }
    public void resetRotatorEncoder(double pos){
        rotator.setSelectedSensorPosition(pos/ Constants.Swerve.RotatorEncoderRatio);
    }

    public void setBrakeMode(){
        driver.setNeutralMode(NeutralMode.Brake);
        rotator.setNeutralMode(NeutralMode.Brake);
    }
    public void setCoastMode(){
        driver.setNeutralMode(NeutralMode.Coast);
        rotator.setNeutralMode(NeutralMode.Coast);
    }

    public SwerveModuleState getState(){
        return new SwerveModuleState(getDriverEncoderVelocity(), new Rotation2d(getRotatorEncoderRotation()));
    }

    public SwerveModulePosition getPosition(){
        return new SwerveModulePosition(getDriverEncoderDistance(), new Rotation2d(getRotatorEncoderRotation()));
    }

    public void setDesiredState(SwerveModuleState state){
        if(Math.abs(state.speedMetersPerSecond)<0.001){
            stop();
            return;
        }

        state = SwerveModuleState.optimize(state, getState().angle);
        SmartDashboard.putNumber("ActualAngle ", state.angle.getDegrees());
        SmartDashboard.putNumber("DesiredAngle", getState().angle.getDegrees());
        //change
        driver.set(state.speedMetersPerSecond / Constants.RobotInfo.MaxRobotSpeed);
        rotator.set(rotatorPID.calculate(getRotatorEncoderRotation(), state.angle.getRadians()));
        SmartDashboard.putString("Swerve["+driver.getDeviceID()+"] state",state.toString());

    }
    public void stop(){
        driver.set(0);
        rotator.set(0);
    }
}
