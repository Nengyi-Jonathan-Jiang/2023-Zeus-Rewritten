package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ElevatorSubsystem extends SubsystemBase{
    private final WPI_TalonFX leftMotor;
    private final WPI_TalonFX rightMotor;

    private final ProfiledPIDController PID;
    private final double kp;
    private final double ki;
    private final double kd;
    private final double maxVelocity;
    private final double maxAcceleration;

    private final double encoderRatio;
    private double setpoint;
    private final double forwardSoftLimit;
    private final double backwardSoftLimit;

    private final StatorCurrentLimitConfiguration StatorCurrentLimit;
    private final SupplyCurrentLimitConfiguration SupplyCurrentLimit;
    public ElevatorSubsystem(){
        leftMotor = new WPI_TalonFX(Constants.IDs.ElevatorMotorL, "DriveTrain");
        rightMotor = new WPI_TalonFX(Constants.IDs.ElevatorMotorR, "DriveTrain");
        setpoint = 0;
        encoderRatio = (1/2048.0)*(1/9.0)*(66/4.6);

        kp = 0.23;
        ki = 0.5;
        kd = 0;
        maxVelocity = 40;
        maxAcceleration = 40;
        PID= new ProfiledPIDController(kp, ki, kd, new TrapezoidProfile.Constraints(maxVelocity, maxAcceleration));

        leftMotor.configFactoryDefault();
        rightMotor.configFactoryDefault();

        leftMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
        resetPos(0);
        
        rightMotor.setInverted(true);
        leftMotor.setInverted(false);

        rightMotor.follow(leftMotor);

        StatorCurrentLimit = new StatorCurrentLimitConfiguration(true, 25, 24, 0.01);
        SupplyCurrentLimit = new SupplyCurrentLimitConfiguration(true, 25, 24, 0.01);

        leftMotor.configStatorCurrentLimit(StatorCurrentLimit);
        leftMotor.configSupplyCurrentLimit(SupplyCurrentLimit);
        rightMotor.configStatorCurrentLimit(StatorCurrentLimit);
        rightMotor.configSupplyCurrentLimit(SupplyCurrentLimit);

        leftMotor.configForwardSoftLimitEnable(true);
        leftMotor.configReverseSoftLimitEnable(true);
        forwardSoftLimit = 85455;
        backwardSoftLimit = 0;
        leftMotor.configForwardSoftLimitThreshold(forwardSoftLimit);
        leftMotor.configReverseSoftLimitThreshold(backwardSoftLimit);
        
        setBrakeMode();
    }

    public void resetPos(double pos){
        leftMotor.setSelectedSensorPosition(pos);
    }

    public void setMotorPos(double setpoint){
        this.setpoint = setpoint;
        setVoltage(PID.calculate(getEncoderPos(), setpoint));
    }

    public double getVoltage(){
        return PID.calculate(getEncoderPos(), setpoint);
    }

    public double getEncoderPos(){
        return leftMotor.getSelectedSensorPosition()*encoderRatio;
    }

    public double getRawEncoderPos(){
        return leftMotor.getSelectedSensorPosition();
    }

    public void setVoltage(double volt){
        leftMotor.setVoltage(volt);
    }
    public void setSpeed(double speed){
        leftMotor.set(speed);
    }

    public void stop(){
        leftMotor.stopMotor();
    }

    public void setBrakeMode(){
        leftMotor.setNeutralMode(NeutralMode.Brake);
        rightMotor.setNeutralMode(NeutralMode.Brake);
    }
    public void setCoastMode(){
        leftMotor.setNeutralMode(NeutralMode.Coast);
        rightMotor.setNeutralMode(NeutralMode.Coast);
    }
}
