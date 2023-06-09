package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.OI;
import frc.robot.subsystems.ElevatorSubsystem;

public class ElevatorCommand extends CommandBase {
    private final ElevatorSubsystem m_elevator;
    private final OI oi;
    private double pos;
    public ElevatorCommand(ElevatorSubsystem m_elevator, OI oi){
        this.m_elevator = m_elevator;
        this.oi = oi;
        pos = 0;
        addRequirements(m_elevator);
    }
    @Override
    public void initialize(){
        pos = 0;
        m_elevator.stop();
        m_elevator.resetPos(pos);
    }

    public void FloorPickup(){
        pos = Constants.SetPoints.FIElevator;
        // pos = 0;
    }
    public void PlatformPickup(){
        pos = Constants.SetPoints.PIElevator;
        // pos = 0;
    }
    public void Moving(){
        pos = Constants.SetPoints.MElevator;
        // pos = 0;
    }
    public void LowPlace(){
        pos = Constants.SetPoints.LSElevator;
        // pos = 0;
    }
    public void MidPlace(){
        pos = Constants.SetPoints.MSElevator;
        // pos = 0;
    }
    public void HighPlace(){
        pos = Constants.SetPoints.HSElevator;
        // pos = 0;
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
        else if(oi.getPovButton(1, 0).getAsBoolean()){
            pos+=1;
        }
        else if(oi.getPovButton(1, 180).getAsBoolean()){
            pos-=1;
        }
        // if(oi.getButton(1, Constants.ControllerConstants.Buttons.A_BUTTON).getAsBoolean()){
        //     pos = 25;
        // }
        // if(oi.getButton(1, Constants.ControllerConstants.Buttons.X_BUTTON).getAsBoolean()){
        //     pos = 40;
        // }
        // if(oi.getButton(1, Constants.ControllerConstants.Buttons.Y_BUTTON).getAsBoolean()){
        //     pos = 50;
        // }
        // if(oi.getButton(1 , Constants.ControllerConstants.Buttons.B_BUTTON).getAsBoolean()){
        //     pos = 5;
        // }
            m_elevator.setMotorPos(pos);
    }
    @Override
    public void end(boolean end){
        m_elevator.stop();
    }
}
