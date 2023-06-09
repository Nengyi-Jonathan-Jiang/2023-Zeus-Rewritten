package frc.robot.subsystems.Swerve;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ActiveVariables;

public class SwerveSubsystem extends SubsystemBase{
    
    //m1 FL, m2 FR, m3 BL, m4 BR
    public final SwerveModule frontLeft = new SwerveModule(
        Constants.IDs.Swerve1Drive, Constants.IDs.Swerve1Rot,
        false, false, Constants.IDs.Swerve1Enc,
        Constants.Swerve.M1Offset, true);

    private final SwerveModule frontRight = new SwerveModule(
        Constants.IDs.Swerve2Drive, Constants.IDs.Swerve2Rot,
        false, false, Constants.IDs.Swerve2Enc,
        Constants.Swerve.M2Offset, true);

    private final SwerveModule backLeft = new SwerveModule(
        Constants.IDs.Swerve3Drive, Constants.IDs.Swerve3Rot,
        false, false, Constants.IDs.Swerve3Enc,
        Constants.Swerve.M3Offset, true);

    private final SwerveModule backRight = new SwerveModule(
        Constants.IDs.Swerve4Drive, Constants.IDs.Swerve4Rot,
        false, false, Constants.IDs.Swerve4Enc,
        Constants.Swerve.M4Offset, true);

    public AHRS gyro = new AHRS(SPI.Port.kMXP);
    private final PIDController PID;
    private final Field2d m_field = new Field2d();
    

    private final SwerveDriveOdometry odometer;

    public SwerveSubsystem(){
        // if(Constants.activeVariables.isRed){
        //     odometer = new SwerveDriveOdometry(Constants.RobotInfo.DriveKinematics, 
        //     new Rotation2d(Math.PI),getModulePositions() );
        // }else{
            odometer = new SwerveDriveOdometry(Constants.RobotInfo.DriveKinematics, 
            new Rotation2d(),getModulePositions() );
        // }
        SmartDashboard.putData("Field", m_field);
        // PID = new PIDController(0.32, 0, 0); //change without weights
        PID = new PIDController(0.29, 0, 0); //change with weights
        new Thread(()->{
            try{
                Thread.sleep(1000);
                zeroHeading();
            }catch(Exception e){}
        }).start();
    }
    public void zeroHeading(){
        gyro.reset();
    }

    public void resetEncoders(){
        frontLeft.resetEncoders();
        frontRight.resetEncoders();
        backLeft.resetEncoders();
        backRight.resetEncoders();
    }
    public SwerveModulePosition[] getModulePositions(){
        return new SwerveModulePosition[]
        {frontLeft.getPosition(), frontRight.getPosition(), backLeft.getPosition(), backRight.getPosition()};
    }
    public double getHeading(){
        return Math.IEEEremainder(gyro.getAngle(),360);
    }
    public double getMoreHeading(){
        return Math.IEEEremainder(gyro.getAngle(),720);
    }
    public Rotation2d getRotation2D(){
        // if(Constants.activeVariables.isRed){
        //     return Rotation2d.fromDegrees(getHeading()+180);
        // }
        return Rotation2d.fromDegrees(getHeading()+180);
    }
    public Pose2d getPose(){
        return odometer.getPoseMeters();
    }
    public void resetOdometry(Pose2d pose){
        odometer.resetPosition(getRotation2D(), getModulePositions(), pose);
    }
    public void update(){
        try{
        if(ActiveVariables.AprilTag&&aprilLoc().length!=0&&aprilLoc()[0]!=0&&aprilLoc()[1]!=0)
        if(ActiveVariables.isRed)
        odometer.resetPosition(getRotation2D(), getModulePositions(), new Pose2d(-aprilLoc()[0], aprilLoc()[1], getRotation2D()));
        else
        odometer.resetPosition(getRotation2D(), getModulePositions(), new Pose2d(aprilLoc()[0], aprilLoc()[1], getRotation2D()));
        else
        // if(Constants.activeVariables.isRed){
            odometer.update(getRotation2D(), getModulePositions());
        // }else{
        //     odometer.update(getRotation2D(), getModulePositions());
        // }
        
        m_field.setRobotPose(getPose());
    }catch(Exception e){}
    }
    @Override
    public void periodic(){
        
        
        SmartDashboard.putNumber("Robot Heading", getHeading());
        SmartDashboard.putString("Robot Location", getPose().getTranslation().toString());
        SmartDashboard.putNumber("Module1AbsEnc", frontLeft.getAbsoluteEncoderRotation());
        SmartDashboard.putNumber("Module2AbsEnc", frontRight.getAbsoluteEncoderRotation());
        SmartDashboard.putNumber("Module3AbsEnc", backLeft.getAbsoluteEncoderRotation());
        SmartDashboard.putNumber("Module4AbsEnc", backRight.getAbsoluteEncoderRotation());

        SmartDashboard.putNumber("Module1MotEnc", frontLeft.getRotatorEncoderRotation());
        SmartDashboard.putNumber("Module2MotEnc", frontRight.getRotatorEncoderRotation());
        SmartDashboard.putNumber("Module3MotEnc", backLeft.getRotatorEncoderRotation());
        SmartDashboard.putNumber("Module4MotEnc", backRight.getRotatorEncoderRotation());

        // SmartDashboard.putNumber("Distance ", frontLeft.)
    }

    public void stopModules(){
        frontLeft.stop();
        frontRight.stop();
        backLeft.stop();
        backRight.stop();
    }
    public double getYaw(){
        return gyro.getYaw();

    }
    public double getPitch(){
        return gyro.getPitch();
    }
    public double getRoll(){
        return gyro.getRoll();
    }
    public void AutoBalance(){
        double speed = PID.calculate(getRoll(), 0)/12;
        SwerveModuleState temp = new SwerveModuleState(-speed, new Rotation2d(0));
        setModuleStates(new SwerveModuleState[]{temp, temp, temp,temp});
    }

    public void setModuleStates(SwerveModuleState[]desiredStates){
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.RobotInfo.MaxRobotSpeed);
        frontLeft.setDesiredState(desiredStates[0]);
        frontRight.setDesiredState(desiredStates[1]);
        backLeft.setDesiredState(desiredStates[2]);
        backRight.setDesiredState(desiredStates[3]);
    }

    
    public void setReflectiveMode(){
        var table =NetworkTableInstance.getDefault().getTable("limelight-slhs");
        table.getEntry("pipeline").setNumber(1);
        ActiveVariables.AprilTag = false;
    }

    public void setAprilMode(){
        var table =NetworkTableInstance.getDefault().getTable("limelight-slhs");
        table.getEntry("pipeline").setNumber(0);
        ActiveVariables.AprilTag = true;
    }
    public double reflectiveOffset(){
        var table =NetworkTableInstance.getDefault().getTable("limelight-slhs");
        table.getEntry("pipeline").setNumber(1);
        return table.getEntry("tx").getDouble(0);
    }
    public double aprilOffset(){
        var table =NetworkTableInstance.getDefault().getTable("limelight-slhs");
        table.getEntry("pipeline").setNumber(0);
        return table.getEntry("tx").getDouble(0);
    }
    public double[] aprilLoc(){
        try{
        var table =NetworkTableInstance.getDefault().getTable("limelight-slhs");
        table.getEntry("pipeline").setNumber(0);
        if(table.getEntry("tv").getDouble(-1)==1)
        return table.getEntry("botpose").getDoubleArray(new double[]{0,0,0});
        return new double[]{0,0,0};
        }catch(Exception e){
            return new double[]{0,0,0};
        }
    }
    public void setAutonCurrent(){
        frontLeft.setAutonCurrent();
        frontRight.setAutonCurrent();
        backLeft.setAutonCurrent();
        backRight.setAutonCurrent();
    }
    public void setTeleopCurrent(){
        frontLeft.setTeleopCurrent();
        frontRight.setTeleopCurrent();
        backLeft.setTeleopCurrent();
        backRight.setTeleopCurrent();
    }
}

