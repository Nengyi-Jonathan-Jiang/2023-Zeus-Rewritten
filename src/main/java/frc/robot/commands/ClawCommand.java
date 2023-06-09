package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.OI;
import frc.robot.subsystems.ClawSubsystem;

public class ClawCommand extends CommandBase {
    private final OI oi;
    private final ClawSubsystem m_claw;
    private double lastTimeClawClosed;
    public ClawCommand(ClawSubsystem m_claw, OI oi){
        this.m_claw = m_claw;
        this.oi = oi;
        lastTimeClawClosed = 0;
        addRequirements(m_claw);
    }
    @Override
    public void initialize(){
       
    }
    @Override
    public void execute(){
        // if(oi.getButton(1, Constants.ControllerConstants.Buttons.A_BUTTON).getAsBoolean()){
        //     m_claw.open();
        //     m_claw.setSpeed(0.2);
        // }
        // if(oi.getButton(1, Constants.ControllerConstants.Buttons.B_BUTTON).getAsBoolean()){
        //     m_claw.close();
        //     m_claw.setSpeed(0.2);
            
        // }
        // if(oi.getButton(1, Constants.ControllerConstants.Buttons.X_BUTTON).getAsBoolean()){
        //     m_claw.open();
        //     m_claw.setSpeed(0);
            
        // }
        // if(oi.getButton(1, Constants.ControllerConstants.Buttons.Y_BUTTON).getAsBoolean()){
        //     m_claw.close();
        //     m_claw.setSpeed(-0.7);
            
        // }
        double speed = 0.4;
    if(Math.abs(oi.getAxis(0, Constants.ControllerConstants.Axes.LEFT_TRIGGER))>0.01){
        Constants.ActiveVariables.isCone = false;
    }
    if(Math.abs(oi.getAxis(0, Constants.ControllerConstants.Axes.RIGHT_TRIGGER))>0.01){
        Constants.ActiveVariables.isCone = true;
        lastTimeClawClosed =Timer.getFPGATimestamp();
    }
        if(oi.getButton(0, Constants.ControllerConstants.Buttons.LEFT_BUMPER).getAsBoolean()){
            
            Constants.LED.LEDStatus = 2;
        }
        else  if(oi.getButton(0, Constants.ControllerConstants.Buttons.RIGHT_BUMPER).getAsBoolean()){
           
            Constants.LED.LEDStatus = 3;
        }
        if(oi.getButton(1, Constants.ControllerConstants.Buttons.Y_BUTTON).getAsBoolean()){
            Constants.ActiveVariables.clawRunning = true;
        }
        //intake
        if(Constants.ActiveVariables.isCone){
            m_claw.close();
            if(Timer.getFPGATimestamp()<=lastTimeClawClosed+1){
                speed = 0.15;
            }else{
            speed = 0;
            }
        }else{
            m_claw.open();
        }
       
        if(Constants.ActiveVariables.clawRunning){
        // speed = 0.4;
        if(oi.getButton(1, Constants.ControllerConstants.Buttons.X_BUTTON).getAsBoolean()){
                speed = -0.2;
                Constants.ActiveVariables.isCone = false;
                m_claw.open();
        }
    }
    else {
        speed = 0;
    }

        
        // if(oi.getButton(1, Constants.ControllerConstants.Buttons.A_BUTTON).getAsBoolean()){
        //     m_claw.open();
        //     speed = 0;
        // }
        // else if(oi.getButton(1, Constants.ControllerConstants.Buttons.X_BUTTON).getAsBoolean()){
        //     m_claw.close();
            
        // }
        m_claw.setSpeed(speed);


    }
    @Override
    public void end(boolean interrupted){
        m_claw.open();
        m_claw.stop();
    }

}
