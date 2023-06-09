package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ArmPivotSubsystem extends SubsystemBase{
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
    private final ArmFeedforward feedforward;

    public ArmPivotSubsystem(){
        leftMotor = new WPI_TalonFX(Constants.IDs.ArmPivotL);
        rightMotor = new WPI_TalonFX(Constants.IDs.ArmPivotR);

        setpoint = 0;
        encoderRatio = (1/2048.0/63.0*26.0/54*2*Math.PI);
        //24-54

        kp = 4.5;
        ki = 0.75;
        kd = 0;
        maxVelocity = Math.PI*0.75;
        maxAcceleration = Math.PI*0.75;
        PID= new ProfiledPIDController(kp, ki, kd, new TrapezoidProfile.Constraints(maxVelocity, maxAcceleration));
        feedforward = new ArmFeedforward(0.065384,0.2 ,2.443, 0.14399);// kg = 0.18423


        leftMotor.configFactoryDefault();
        rightMotor.configFactoryDefault();

        leftMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
        
        resetPos(0);

        StatorCurrentLimit = new StatorCurrentLimitConfiguration(true, 40, 39, 0.01);
        SupplyCurrentLimit = new SupplyCurrentLimitConfiguration(true, 40, 39, 0.01);
        leftMotor.configStatorCurrentLimit(StatorCurrentLimit);
        leftMotor.configSupplyCurrentLimit(SupplyCurrentLimit);
        rightMotor.configStatorCurrentLimit(StatorCurrentLimit);
        rightMotor.configSupplyCurrentLimit(SupplyCurrentLimit);

        rightMotor.setInverted(false);
        leftMotor.setInverted(true);

        rightMotor.follow(leftMotor);

        // leftMotor.configForwardSoftLimitEnable(true);
        // leftMotor.configReverseSoftLimitEnable(true);
        forwardSoftLimit =  1000694;
        backwardSoftLimit = 0;
        // leftMotor.configForwardSoftLimitThreshold(forwardSoftLimit);
        // leftMotor.configReverseSoftLimitThreshold(backwardSoftLimit);

        // setBrakeMode();
        setCoastMode();
    }

    public void resetPos(double pos){
        leftMotor.setSelectedSensorPosition(pos);
    }

    public void setMotorPos(double setpoint){
        this.setpoint = setpoint;
        setVoltage(PID.calculate(getEncoderPos(), setpoint)+feedforward.calculate(setpoint, 0));
    }

    public double getVoltage(){
        return PID.calculate(getEncoderPos(), setpoint)+feedforward.calculate(setpoint, 0);
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
