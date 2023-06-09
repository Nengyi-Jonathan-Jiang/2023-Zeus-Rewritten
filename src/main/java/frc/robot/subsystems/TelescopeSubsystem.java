package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class TelescopeSubsystem extends SubsystemBase{
    private final Solenoid m_solenoid;
    public TelescopeSubsystem(){
        m_solenoid = new Solenoid(Constants.IDs.PneumaticHub,PneumaticsModuleType.REVPH, Constants.IDs.ArmPiston);
        retract();
    }
    public void extend(){
        m_solenoid.set(true);
    }
    public void retract(){
        m_solenoid.set(false);
    }
    public void setState(boolean state){
        if(state){
            extend();
        }else{
            retract();
        }
    }
    public boolean isOpen(){
        return m_solenoid.get();
    }
}
