package frc.robot.commands;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Swerve.SwerveSubsystem;

public class AutoBalance extends CommandBase{
    private final SwerveSubsystem m_swerve;
    public AutoBalance(SwerveSubsystem m_swerve){
        this.m_swerve = m_swerve;
        addRequirements(m_swerve);
    }
    @Override
    public void execute() {
        m_swerve.AutoBalance();   
    }
    @Override
    public void end(boolean interrupted) {
        m_swerve.stopModules();
    }
}
