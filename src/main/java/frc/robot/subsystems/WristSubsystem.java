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

public class WristSubsystem extends SubsystemBase{
    private final WPI_TalonFX wrist;
    private final double encoderRatio;
    private final ProfiledPIDController PID;
    private final double kp;
    private final double ki;
    private final double kd;
    private final double max_velocity;
    private final double max_acceleration;
    private double setpoint;
    private final double forwardSoftLimit;
    private final double backwardSoftLimit;
    private final StatorCurrentLimitConfiguration StatorCurrentLimit;
    private final SupplyCurrentLimitConfiguration SupplyCurrentLimit;
    private final ArmFeedforward feedforward;

    public WristSubsystem(){
        wrist= new WPI_TalonFX(Constants.IDs.WristMotor);
        setBrakeMode();
        wrist.configFactoryDefault();
        wrist.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
        resetPos(0);

        wrist.configForwardSoftLimitEnable(true);
        wrist.configReverseSoftLimitEnable(true);
        forwardSoftLimit= 100319;
        backwardSoftLimit = -30000;//0
        wrist.configForwardSoftLimitThreshold(forwardSoftLimit);
        wrist.configReverseSoftLimitThreshold(backwardSoftLimit);

        encoderRatio = 1/2048.0/81.0*16.0/22*2*Math.PI;
        kp = 4;//3
        ki = 5;//4
        kd = 0;//0
        max_velocity = Math.PI*1;
        max_acceleration = Math.PI/2;
        setpoint = 0;

        StatorCurrentLimit = new StatorCurrentLimitConfiguration(true,40, 39, 0.01);
        SupplyCurrentLimit = new SupplyCurrentLimitConfiguration(true, 40, 39, 0.01);
        wrist.configStatorCurrentLimit(StatorCurrentLimit);
        wrist.configSupplyCurrentLimit(SupplyCurrentLimit);
        PID = new ProfiledPIDController(kp, ki, kd, new TrapezoidProfile.Constraints(max_velocity, max_acceleration));
        feedforward = new ArmFeedforward(0.12676, 0.5, 0.5601, 0.053899);//0.40752
    }

    public void resetPos(double pos){
        wrist.setSelectedSensorPosition(pos);
    }

    public double getPosition(){
        return wrist.getSelectedSensorPosition()*encoderRatio;
    }
    public double getRawPosition(){
        return wrist.getSelectedSensorPosition();
    }
    public double getVelocity(){
        return wrist.getSelectedSensorVelocity()*encoderRatio;
    }

    public double getVoltage(){
        return PID.calculate(getPosition(), setpoint)+feedforward.calculate(setpoint, 0);
    }

    public void setMotorPos(double setpoint){
        this.setpoint = setpoint;
        setVoltage(PID.calculate(getPosition(), setpoint)+feedforward.calculate(setpoint, 0));
    }

    public void setSpeed(double speed){
        wrist.set(speed);
    }
    public void setVoltage(double volt){
        wrist.setVoltage(volt);
    }

    public void setBrakeMode(){
        wrist.setNeutralMode(NeutralMode.Brake);
    }
    public void setCoastMode(){
        wrist.setNeutralMode(NeutralMode.Coast);
    }
    public void stop(){
        wrist.stopMotor();
    }
    
}
