package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.ArmPivotSubsystem;
import frc.robot.subsystems.ClawSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.TelescopeSubsystem;
import frc.robot.subsystems.WristSubsystem;

public class CubePlacement extends CommandBase{
    ElevatorSubsystem m_ele;
    ArmPivotSubsystem m_arm;
    WristSubsystem m_wrist;
    TelescopeSubsystem m_tele;
    ClawSubsystem m_claw;
    public CubePlacement(ElevatorSubsystem m_ele, ArmPivotSubsystem m_arm, WristSubsystem m_wrist, TelescopeSubsystem m_tele, ClawSubsystem m_claw){
        this.m_ele = m_ele;
        this.m_arm = m_arm;
        this.m_wrist = m_wrist;
        this.m_tele = m_tele;
        this.m_claw = m_claw;
        addRequirements(m_ele, m_arm, m_wrist,m_tele,m_claw);
    }
    @Override
    public void initialize() {
        m_claw.close();
        m_claw.setSpeed(0);
        m_ele.setMotorPos(5);
        m_wrist.setMotorPos(Math.PI/6);
        m_tele.setState(false);
    }
    @Override
    public void execute() {
        if(!m_claw.getState()){
            m_ele.setMotorPos(Constants.SetPoints.HSElevator);
            m_arm.setMotorPos(Constants.SetPoints.HSArm);
            m_wrist.setMotorPos(Constants.SetPoints.HSWrist);
            m_tele.setState(Constants.SetPoints.HSExtension);
            if(m_arm.getEncoderPos()>Constants.SetPoints.HSArm){
                m_claw.open();
                m_claw.setSpeed(0.1);
            }
        }else{
            m_ele.setMotorPos(0);
        m_arm.setMotorPos(0);
        m_wrist.setMotorPos(Constants.SetPoints.MWrist);
        m_tele.setState(false);
       m_claw.setSpeed(0);
        }
        
    }

    @Override
    public void end(boolean interrupted) {
        // m_ele.setMotorPos(0);
        // m_arm.setMotorPos(0);
        // m_wrist.setMotorPos(Math.PI/4);
    //     m_tele.setState(false);
    //    m_claw.setSpeed(0);
    //    m_claw.close();
    }
}
