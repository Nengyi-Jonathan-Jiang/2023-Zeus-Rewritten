package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.OI;
import frc.robot.Constants.ControllerConstants;
import frc.robot.subsystems.Swerve.SwerveSubsystem;

public class SwerveCommand extends CommandBase{
    private final SwerveSubsystem m_swerve;
    private final OI oi;
    private final SlewRateLimiter xLimiter, yLimiter;
    private final double xLimit = 0.65;//change for all 3
    private final double yLimit = 0.65;
    // private final double turningLimit = 0.1;
    public SwerveCommand(SwerveSubsystem m_swerve, OI oi){
        this.m_swerve = m_swerve;
        this.oi = oi;
        xLimiter = new SlewRateLimiter(xLimit);
        yLimiter = new SlewRateLimiter(yLimit);
        // turningLimiter = new SlewRateLimiter(turningLimit);
        addRequirements(m_swerve);
    }
    @Override
    public void initialize(){

    }
    @Override
    public void execute(){
        double kp = 0.05;//0.05
        double ki = 0.05775;//0.058
        double kpr = 0.1;
        double kir = 0;
        double MAXAdjustmentSpeed = 0.55;
        double MAXAdjustmentRot = 0;
        PIDController PID= new PIDController(kp, ki, 0);
        PIDController PIDX= new PIDController(2.25, 2.5, 0);//make larger
        
        PIDController PIDY= new PIDController(2.5, 2, 0);//make larger
        PIDController PIDRot= new PIDController(0.075, 0.1, 0);
        m_swerve.update();
        int sign = 0;
        double distPickup=7.295;
        double distPlace = -6.7;//-6.6
        double leftOffset = 0;
        double rightOffset = 3.15;
        SmartDashboard.putNumber("value to go forward", PIDX.calculate(m_swerve.getPose().getX(),distPickup));
        // SmartDashboard.putNumber("value to go forward", PID.calculate(m_swerve.reflectiveOffset()));
        if(oi.getButton(0, ControllerConstants.Buttons.X_BUTTON).getAsBoolean()){
            m_swerve.setReflectiveMode();
            // m_swerve.reflectiveOffset();
            PID.setSetpoint(0);
            // if(Constants.activeVariables.isRed){de
            //     PIDRot.setSetpoint(Math.PI);
            // }else{
                PIDRot.setSetpoint(0);
                PIDX.setSetpoint(distPlace);
            // }
            ChassisSpeeds chassisSpeeds;
            // if(Constants.activeVariables.isRed){
            //     sign = 1;
            // }else{
                sign = 1;
            // }
            double xpid = (PIDX.calculate(m_swerve.getPose().getX())<0)?Math.max(PIDX.calculate(m_swerve.getPose().getX()), -MAXAdjustmentSpeed):Math.min(PIDX.calculate(m_swerve.getPose().getX()), MAXAdjustmentSpeed);
            
            double ypid = (PID.calculate(m_swerve.reflectiveOffset())<0)?Math.max(sign*PID.calculate(m_swerve.reflectiveOffset()), -MAXAdjustmentSpeed):Math.min(PID.calculate(m_swerve.reflectiveOffset()), MAXAdjustmentSpeed);

            chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(xpid,ypid , PIDRot.calculate(m_swerve.getHeading()), m_swerve.getRotation2D());
            SwerveModuleState[] modulestates = Constants.RobotInfo.DriveKinematics.toSwerveModuleStates(chassisSpeeds);
            
    
            m_swerve.setModuleStates(modulestates);
        }
        else if(oi.getButton(0, ControllerConstants.Buttons.Y_BUTTON).getAsBoolean()){
            m_swerve.setAprilMode();
            PID.setSetpoint(0);
            // if(Constants.activeVariables.isRed){
            //     PIDRot.setSetpoint(Math.PI);
            // }else{
                PIDRot.setSetpoint(0);
                PIDX.setSetpoint(distPlace);
            // }
            ChassisSpeeds chassisSpeeds;
            // if(Constants.activeVariables.isRed){
            //     sign = 1;
            // }else{
                sign = 1;
            // }

            double xpid = (PIDX.calculate(m_swerve.getPose().getX())<0)?Math.max(PIDX.calculate(m_swerve.getPose().getX()), -MAXAdjustmentSpeed):Math.min(PIDX.calculate(m_swerve.getPose().getX()), MAXAdjustmentSpeed);
            
            double ypid = (PID.calculate(m_swerve.aprilOffset())<0)?Math.max(sign*PID.calculate(m_swerve.aprilOffset()), -MAXAdjustmentSpeed):Math.min(PID.calculate(m_swerve.aprilOffset()), MAXAdjustmentSpeed);

            chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(xpid,ypid , PIDRot.calculate(m_swerve.getHeading()), m_swerve.getRotation2D());
            SwerveModuleState[] modulestates = Constants.RobotInfo.DriveKinematics.toSwerveModuleStates(chassisSpeeds);
    
            m_swerve.setModuleStates(modulestates);
        }else if(oi.getButton(0, ControllerConstants.Buttons.A_BUTTON).getAsBoolean()){
            if(m_swerve.getPose().getX()>distPickup-0.1){
                Constants.LED.LEDStatus = 1;
            }
            m_swerve.setAprilMode();
            PIDY.setSetpoint(rightOffset);
            // if(Constants.activeVariables.isRed){
            //     PIDRot.setSetpoint(Math.PI);
            // }else{
                // PIDRot.setSetpoint(180);
                PIDX.setSetpoint(distPickup);
            // }
            ChassisSpeeds chassisSpeeds;
            // if(Constants.activeVariables.isRed){
            //     sign = 1;
            // }else{
                sign = 1;
            // }

            double xpid = (PIDX.calculate(m_swerve.getPose().getX())<0)?Math.max(PIDX.calculate(m_swerve.getPose().getX()), -MAXAdjustmentSpeed):Math.min(PIDX.calculate(m_swerve.getPose().getX()), MAXAdjustmentSpeed);
            
            double ypid = (PIDY.calculate(m_swerve.getPose().getY())<0)?Math.max(sign*PIDY.calculate(m_swerve.getPose().getY()), -MAXAdjustmentSpeed):Math.min(PIDY.calculate(m_swerve.getPose().getY()), MAXAdjustmentSpeed);

            ypid = 0;
            // if(m_swerve.getMoreHeading()){

            // }
            if(m_swerve.getMoreHeading()<0){
                PIDRot.setSetpoint(-180);
            }else{
                PIDRot.setSetpoint(180);
            }
            double apid = PIDRot.calculate(m_swerve.getMoreHeading());

            chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(xpid,ypid , apid, m_swerve.getRotation2D());
            SwerveModuleState[] modulestates = Constants.RobotInfo.DriveKinematics.toSwerveModuleStates(chassisSpeeds);
    
            m_swerve.setModuleStates(modulestates);
        }

        else if(oi.getButton(0, ControllerConstants.Buttons.B_BUTTON).getAsBoolean()){
            m_swerve.AutoBalance();
        }else{
            m_swerve.setAprilMode();
            double xSpeed;
            double ySpeed;
            double turningSpeed;
        // if(Constants.activeVariables.isRed){
            xSpeed = oi.getAxis(0, ControllerConstants.Axes.LEFT_STICK_X)*.4;
            ySpeed = -oi.getAxis(0, ControllerConstants.Axes.LEFT_STICK_Y)*.4;
            turningSpeed = oi.getAxis(0, ControllerConstants.Axes.RIGHT_STICK_X);
        // }else{
        //      xSpeed = -oi.getAxis(0, Constants.ControllerConstants.Axes.LEFT_STICK_X);
        //     ySpeed = oi.getAxis(0, Constants.ControllerConstants.Axes.LEFT_STICK_Y);
        //     turningSpeed = oi.getAxis(0, Constants.ControllerConstants.Axes.RIGHT_STICK_X);
        // }


        xSpeed = xLimiter.calculate(xSpeed)*Constants.RobotInfo.MaxRobotSpeed;
        ySpeed = yLimiter.calculate(ySpeed)*Constants.RobotInfo.MaxRobotSpeed;
        // xSpeed*=Constants.RobotInfo.MAX_ROBOT_SPEED;
        // ySpeed*=Constants.RobotInfo.MAX_ROBOT_SPEED;
        turningSpeed = turningSpeed*Constants.RobotInfo.MaxRobotSpeed;

        ChassisSpeeds chassisSpeeds;
        chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(-ySpeed, xSpeed, turningSpeed, m_swerve.getRotation2D());

        SwerveModuleState[] modulestates = Constants.RobotInfo.DriveKinematics.toSwerveModuleStates(chassisSpeeds);

        m_swerve.setModuleStates(modulestates);
        }
    }
    @Override
    public void end(boolean end){
        m_swerve.stopModules();
    }
}
