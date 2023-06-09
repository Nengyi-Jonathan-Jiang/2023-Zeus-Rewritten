
package frc.robot;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.ArmPivotCommand;
import frc.robot.commands.AutoBalance;
import frc.robot.commands.ClawCommand;
import frc.robot.commands.CubePlacement;
import frc.robot.commands.ElevatorCommand;
import frc.robot.commands.SwerveCommand;
import frc.robot.commands.TelescopeCommand;
import frc.robot.commands.WristCommand;
import frc.robot.subsystems.ArmPivotSubsystem;
import frc.robot.subsystems.ClawSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.TelescopeSubsystem;
import frc.robot.subsystems.WristSubsystem;
import frc.robot.subsystems.Swerve.SwerveSubsystem;

public class RobotContainer {
  public WristSubsystem m_wrist;
  public ClawSubsystem m_claw;
  public TelescopeSubsystem m_telescope;
  public ElevatorSubsystem m_elevator;
  public ArmPivotSubsystem m_armPivot;
  public SwerveSubsystem m_swerve;
  public OI m_oi;
  private final String middleAuto = "Middle";
  private final String leftAuto = "Left";
  private final String rightAuto = "Right";
  private String autoSelected;
  private final SendableChooser<String> m_chooser=  new SendableChooser<>();


  public RobotContainer() {
    m_chooser.setDefaultOption("Middle Auton", middleAuto);
    m_chooser.addOption("Left Auton", leftAuto);
    m_chooser.addOption("Right Auto", rightAuto);
    SmartDashboard.putData("Auto choices", m_chooser);
    m_wrist = new WristSubsystem();
    m_claw = new ClawSubsystem();
    m_telescope = new TelescopeSubsystem();
    m_elevator = new ElevatorSubsystem();
    m_armPivot = new ArmPivotSubsystem();
    m_swerve = new SwerveSubsystem();
    m_oi = new OI();
    setDefaultCommands();
    configureBindings();
  }
  private void setDefaultCommands(){
    m_swerve.setDefaultCommand(new SwerveCommand(m_swerve, m_oi));
    m_wrist.setDefaultCommand(new WristCommand(m_wrist, m_oi));
    m_claw.setDefaultCommand(new ClawCommand(m_claw, m_oi));
     m_telescope.setDefaultCommand(new TelescopeCommand(m_telescope, m_oi));
    m_elevator.setDefaultCommand(new ElevatorCommand(m_elevator, m_oi));
    m_armPivot.setDefaultCommand(new ArmPivotCommand(m_armPivot, m_oi));
  }
  private void configureBindings() {
  }

  public PathPlannerTrajectory middlePath(){
    return PathPlanner.generatePath(
      new PathConstraints(Constants.RobotInfo.Auton.maxSpeed, Constants.RobotInfo.Auton.maxAcceleration), 
      new PathPoint(new Translation2d(0, 0), Rotation2d.fromDegrees(0), Rotation2d.fromDegrees(0)),
      new PathPoint(new Translation2d(-1.75,0 ), Rotation2d.fromDegrees(0), Rotation2d.fromDegrees(0))//-3,-0.5
      );
  }

  public PathPlannerTrajectory leftPath(){
    return PathPlanner.generatePath(
      new PathConstraints(Constants.RobotInfo.Auton.maxSpeed, Constants.RobotInfo.Auton.maxAcceleration), 
      new PathPoint(new Translation2d(0, 0), Rotation2d.fromDegrees(0), Rotation2d.fromDegrees(0)),
      new PathPoint(new Translation2d(-0.1,0.23 ), Rotation2d.fromDegrees(0), Rotation2d.fromDegrees(0))//-3,-0.5
      , new PathPoint(new Translation2d(-2,0 ), Rotation2d.fromDegrees(0), Rotation2d.fromDegrees(0))
      );
  }
  public PathPlannerTrajectory rightPath(){
    return PathPlanner.generatePath(
      new PathConstraints(Constants.RobotInfo.Auton.maxSpeed, Constants.RobotInfo.Auton.maxAcceleration), 
      new PathPoint(new Translation2d(0, 0), Rotation2d.fromDegrees(0), Rotation2d.fromDegrees(0)),
      new PathPoint(new Translation2d(-0.1,-0.23 ), Rotation2d.fromDegrees(0), Rotation2d.fromDegrees(0))//-3,-0.5
      , new PathPoint(new Translation2d(-2,0 ), Rotation2d.fromDegrees(0), Rotation2d.fromDegrees(0))
      );
  }
  public Command getAutonomousCommand() {
    TrajectoryConfig trajectoryConfig = new TrajectoryConfig(
      Constants.RobotInfo.Auton.maxSpeed, 
      Constants.RobotInfo.Auton.maxAcceleration).setKinematics(Constants.RobotInfo.DriveKinematics);

      //create traj here
      //MUST CHANGE BEFORE MATCH
      autoSelected = m_chooser.getSelected();
  PathPlannerTrajectory trajectory = (autoSelected.equals(middleAuto)?middlePath():(autoSelected.equals(leftAuto)?leftPath():rightPath()));


      PIDController xController = new PIDController(0.1, 0.05, 0);//change
      PIDController yController = new PIDController(0.1, 0.05, 0); //change
      ProfiledPIDController thetaController = new ProfiledPIDController(
        1,0.5,0, new TrapezoidProfile.Constraints(Constants.RobotInfo.Auton.maxRotSpeed, 
        Constants.RobotInfo.Auton.maxRotAccel)); // change
      thetaController.enableContinuousInput(-Math.PI, Math.PI);

      SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
        trajectory, 
        m_swerve::getPose, 
        Constants.RobotInfo.DriveKinematics, 
        xController, 
        yController,
        thetaController,
        m_swerve::setModuleStates,
        m_swerve);
      
    return new CubePlacement(m_elevator, m_armPivot, m_wrist, m_telescope, m_claw).withTimeout(4)
    .andThen(
      new SequentialCommandGroup(new InstantCommand(()->m_swerve.resetOdometry(trajectory.getInitialPose())),
      swerveControllerCommand,
      new InstantCommand(() ->m_swerve.stopModules()))).andThen(new AutoBalance(m_swerve)).withTimeout(15);
  }
  public void disabledMotorMode(){

  }
  public void enableMotorMode(){

  }
}
