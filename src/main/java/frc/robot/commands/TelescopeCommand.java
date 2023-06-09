package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.OI;
import frc.robot.subsystems.TelescopeSubsystem;

public class TelescopeCommand extends CommandBase {
    private final OI oi;
    private final TelescopeSubsystem m_telescope;
    private boolean extended;
    public TelescopeCommand(TelescopeSubsystem m_telescope, OI oi){
        this.oi = oi;
        this.m_telescope = m_telescope;
        extended = false;
        addRequirements(m_telescope);
    }
    @Override
    public void initialize(){

    }
    public void FloorPickup(){
        extended = Constants.SetPoints.FIExtension;
    }
    public void PlatformPickup(){
        extended = Constants.SetPoints.PIExtension;
    }
    public void Moving(){
        extended = Constants.SetPoints.MExtension;
    }
    public void LowPlace(){
        extended = Constants.SetPoints.LSExtension;
    }
    public void MidPlace(){
        extended = Constants.SetPoints.MSExtension;
    }
    public void HighPlace(){
        extended = Constants.SetPoints.HSExtension;
    }
    @Override
    public void execute(){
        // if(oi.getButton(1, Constants.ControllerConstants.Buttons.B_BUTTON).getAsBoolean()){
        //     m_telescope.extend();
        //  }
        //  if(oi.getButton(1, Constants.ControllerConstants.Buttons.A_BUTTON).getAsBoolean()){
        //     m_telescope.retract();
        // }
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
        else if(oi.getPovButton(1, 90).getAsBoolean()){
            extended = true;
        }
        else if(oi.getPovButton(1, 270).getAsBoolean()){
            extended = false;
        }
        else if(m_telescope.isOpen()!=extended){
            m_telescope.setState(extended);
        }

    }
    @Override
    public void end(boolean end){
        m_telescope.retract();
    }
}
