package frc.robot.commands;

import edu.wpi.first.cscore.VideoSource.ConnectionStrategy;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.OI;
import frc.robot.subsystems.WristSubsystem;

public class WristCommand extends CommandBase{
    private final WristSubsystem m_wrist;
    private final OI oi;
    private double pos;
    public WristCommand(WristSubsystem m_wrist, OI oi){
        this.m_wrist = m_wrist;
        this.oi = oi;
        pos= 0;
        addRequirements(m_wrist);
    }
    @Override
    public void initialize(){
        pos = 0;
        m_wrist.stop();
        m_wrist.resetPos(pos);
    }

    public void FloorPickup(){
        pos = Constants.SetPoints.FIWrist;
        // pos = Constants.SetPoints.MWrist;
    }
    public void PlatformPickup(){
        pos = Constants.SetPoints.PIWrist;
    }
    public void Moving(){
        pos = Constants.SetPoints.MWrist;
    }
    public void LowPlace(){
        pos = Constants.SetPoints.LSWrist;
        // pos = Constants.SetPoints.MWrist;
    }
    public void MidPlace(){
        pos = Constants.SetPoints.MSWrist;
    }
    public void HighPlace(){
        pos = Constants.SetPoints.HSWrist;
    }
    @Override
    public void execute(){

        
        if(oi.getButton(1, Constants.ControllerConstants.Buttons.LEFT_BUMPER).getAsBoolean()){
            MidPlace();
        }
        else if(oi.getButton(1, Constants.ControllerConstants.Buttons.RIGHT_BUMPER).getAsBoolean()){
            HighPlace();
        }
        else if(oi.getButton(1, Constants.ControllerConstants.Buttons.B_BUTTON).getAsBoolean()){
            LowPlace();
        }
        else if(oi.getAxis(1, Constants.ControllerConstants.Axes.RIGHT_TRIGGER)>0.2){
            PlatformPickup();
        }
        else if(oi.getAxis(1, Constants.ControllerConstants.Axes.LEFT_TRIGGER)>0.2){
            FloorPickup();
        }
        else if(oi.getButton(1, Constants.ControllerConstants.Buttons.Y_BUTTON).getAsBoolean()){
            Moving();
        }
        else if(oi.getAxis(1, Constants.ControllerConstants.Axes.LEFT_STICK_Y)>0.1){
            pos-=0.025;
        }
        else if(oi.getAxis(1, Constants.ControllerConstants.Axes.LEFT_STICK_Y)<-0.1){
            pos+=0.025;
        }

        // if(oi.getButton(1, Constants.ControllerConstants.Buttons.A_BUTTON).getAsBoolean()){
        //     pos = 1;
        // }
        // if(oi.getButton(1, Constants.ControllerConstants.Buttons.B_BUTTON).getAsBoolean()){
        //     pos = 0;
        // }
        // pos = 0;
        SmartDashboard.putNumber("WristDesired ENcoder", pos);
        m_wrist.setMotorPos(pos);
    }
    @Override
    public void end(boolean end){
        m_wrist.stop();
    }

}
