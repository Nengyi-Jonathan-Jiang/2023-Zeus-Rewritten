package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.OI;
import frc.robot.subsystems.ArmPivotSubsystem;

public class ArmPivotCommand extends CommandBase{
    private final ArmPivotSubsystem m_armPivot;
    private final OI oi;
    private double pos;

    public ArmPivotCommand(ArmPivotSubsystem m_armPivot, OI oi){
        this.oi = oi;
        this.m_armPivot = m_armPivot;
        pos = 0;
        addRequirements(m_armPivot);
    }
    @Override
    public void initialize(){
        m_armPivot.stop();
        pos = 0;
        m_armPivot.resetPos(pos);
    }

    public void FloorPickup(){
        pos = Constants.SetPoints.FIArm;
        // pos = Constants.SetPoints.MArm;
    }
    public void PlatformPickup(){
        pos = Constants.SetPoints.PIArm;
    }
    public void Moving(){
        pos = Constants.SetPoints.MArm;
    }
    public void LowPlace(){
        pos = Constants.SetPoints.LSArm;
        // pos = Constants.SetPoints.MArm;
    }
    public void MidPlace(){
        pos = Constants.SetPoints.MSArm;
    }
    public void HighPlace(){
        pos = Constants.SetPoints.HSArm;
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
        
        else if(oi.getAxis(1, Constants.ControllerConstants.Axes.RIGHT_STICK_Y)>0.1){
            pos-=0.025;
            // pivot.setSpeed(oi.getAxis(1, Constants.Axes.RIGHT_STICK_Y)*0.1);
        }
        else if(oi.getAxis(1, Constants.ControllerConstants.Axes.RIGHT_STICK_Y)<-0.1){
            pos+=0.025;
            // pivot.setSpeed(oi.getAxis(1, Constants.Axes.RIGHT_STICK_Y)*0.1);
        }
        // if(oi.getButton(0, Constants.ControllerConstants.Buttons.B_BUTTON).getAsBoolean()){
        //     pos = -0.1;
        // }else if(oi.getButton(0, Constants.ControllerConstants.Buttons.A_BUTTON).getAsBoolean()){
        //     pos = 0;
        //     m_armPivot.resetPos(pos);
        // }
        // if(oi.getButton(1, Constants.ControllerConstants.Buttons.A_BUTTON).getAsBoolean()){
        //     pos = Math.PI/4;
        // }
        // if(oi.getButton(1, Constants.ControllerConstants.Buttons.B_BUTTON).getAsBoolean()){
        //     pos = Math.PI/2;
        // }
        // if(oi.getButton(1, Constants.ControllerConstants.Buttons.X_BUTTON).getAsBoolean()){
        //     pos = 0;
        // }
        // if(oi.getButton(1, Constants.ControllerConstants.Buttons.Y_BUTTON).getAsBoolean()){
        //     pos = Math.PI*3/4;
        // }
        // if(oi.getButton(1, Constants.ControllerConstants.Buttons.A_BUTTON).getAsBoolean()){
        //     pos = Math.PI/4;
        // }
        // if(oi.getButton(1, Constants.ControllerConstants.Buttons.B_BUTTON).getAsBoolean()){
        //     pos = Math.PI/6;
        // }
        // if(oi.getButton(1, Constants.ControllerConstants.Buttons.Y_BUTTON).getAsBoolean()){
        //     pos = Math.PI/2;
        // }
        // if(oi.getButton(1,Constants.ControllerConstants.Buttons.X_BUTTON).getAsBoolean()){
        //     pos = 0;
        // }
        SmartDashboard.putNumber("PosArm", pos);
        m_armPivot.setMotorPos(pos);
    }
    @Override
    public void end(boolean end){
        m_armPivot.stop();
    }
}
