
package frc.robot;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;
  private static AddressableLED m_Led;
  private static AddressableLEDBuffer m_LedBuffer;
  private static int starterLED;
  private int counter = 0;
  private double time = 0;

  @Override
  public void robotInit() {

    m_robotContainer = new RobotContainer();
    m_Led = new AddressableLED(1);
    m_LedBuffer = new AddressableLEDBuffer(60);
    m_Led.setLength(m_LedBuffer.getLength());
    m_Led.setData(m_LedBuffer);
    m_Led.start();

    // m_LedBuffer.setRGB(1, 200,200,200);
    // m_Led.setData(m_LedBuffer);
    starterLED = 0;
  }

  @Override
  public void robotPeriodic() {
    if(DriverStation.getAlliance().toString().equals("Red")){
      Constants.ActiveVariables.isRed = true;
    }else{
      Constants.ActiveVariables.isRed = false;
    }
    SmartDashboard.putNumber("Roll", m_robotContainer.m_swerve.getRoll());
    // SmartDashboard.putNumber("rot",m_robotContainer.m_swerve.frontLeft.getDriverEncoderDistance());
    SmartDashboard.putNumber("ElevatorRawEncoder", m_robotContainer.m_elevator.getRawEncoderPos());
    SmartDashboard.putNumber("ElevatorEncoder", m_robotContainer.m_elevator.getEncoderPos());

    SmartDashboard.putNumber("WristRawEncoder", m_robotContainer.m_wrist.getRawPosition());
    SmartDashboard.putNumber("WristEncoder", m_robotContainer.m_wrist.getPosition());

    SmartDashboard.putNumber("ArmRawPosition", m_robotContainer.m_armPivot.getRawEncoderPos());
    SmartDashboard.putNumber("ArmEncoder", m_robotContainer.m_armPivot.getEncoderPos());

    SmartDashboard.putNumber("LeftCurrent", m_robotContainer.m_claw.getLCurrent());
    SmartDashboard.putNumber("RightCurrent", m_robotContainer.m_claw.getRCurrent());

    SmartDashboard.putNumber("ArmVoltage", m_robotContainer.m_armPivot.getVoltage());
    SmartDashboard.putNumber("RobotHeading", m_robotContainer.m_swerve.getHeading());

   SmartDashboard.putString("RobotPos", m_robotContainer.m_swerve.getPose().toString());

   SmartDashboard.putString("Color", DriverStation.getAlliance().toString());
   SmartDashboard.putString("Heading", m_robotContainer.m_swerve.getMoreHeading()+"");

   SmartDashboard.putNumber("RobotZ", m_robotContainer.m_swerve.gyro.getDisplacementZ());

   SmartDashboard.putString("OutputCurrentClaw", "Left:"+m_robotContainer.m_claw.getLCurrent()+" Right:"+m_robotContainer.m_claw.getRCurrent());

  // SmartDashboard.putString("robottemp", m_robotContainer.m_swerve.aprilLoc().toString());
  //  try{
    // if(m_robotContainer.m_swerve.aprilLoc().length!=1){
    // SmartDashboard.putNumber("RobotX", m_robotContainer.m_swerve.aprilLoc()[0]);
    // SmartDashboard.putNumber("RobotY", m_robotContainer.m_swerve.aprilLoc()[1]);
    // }
  //  }catch(Exception e){}
    // setLEDs();
    CommandScheduler.getInstance().run();
    if(time==0||Timer.getFPGATimestamp()-time>0.1){
      time = Timer.getFPGATimestamp();
    if (Constants.LED.LEDStatus != 0) {
      for (var i = 0; i < m_LedBuffer.getLength(); i++) {
        if (Constants.LED.LEDStatus == 1) {
          m_LedBuffer.setRGB(i, Constants.LED.GreenR, Constants.LED.GreenG, Constants.LED.GreenB);
        } else if (Constants.LED.LEDStatus == 2) {
          m_LedBuffer.setRGB(i, Constants.LED.PurpleR, Constants.LED.PurpleG, Constants.LED.PurpleB);
        } else if (Constants.LED.LEDStatus == 3) {
          m_LedBuffer.setRGB(i, Constants.LED.YellowR, Constants.LED.YellowG, Constants.LED.YellowB);
        } else if (Constants.LED.LEDStatus == 4) {
          m_LedBuffer.setRGB(i, Constants.LED.RedR, Constants.LED.RedG, Constants.LED.RedB);
        }
      }
    } else {
      boolean blue = false;
      // if(Timer.getFPGATimestamp()%1==0)

      for (var i = 0; i < m_LedBuffer.getLength(); i++) {
       
          // m_LedBuffer.setRGB((i), 141, 44,114);
          // m_LedBuffer.setHSV(i, 235, 97, 50);
        // if(counter == -2){
        //   m_LedBuffer.setRGB((starterLED+i)%53, 3, 23,252);
        // }
        //   else if(counter == -1){
        //   m_LedBuffer.setRGB((starterLED+i)%53, 3, 23,252);
        // }
        // else if(counter == 0){
        // m_LedBuffer.setRGB((starterLED+i)%53, 3, 23,252);
        // }
        

        // else if(counter ==1){
        //   m_LedBuffer.setRGB((starterLED+i)%53, 31, 27,224);
        // }
        // else if(counter ==2){
        //   m_LedBuffer.setRGB((starterLED+i)%53, 58, 31,197);
        // }
        // else if(counter ==3){
        //   m_LedBuffer.setRGB((starterLED+i)%53, 86, 36,169);
        // }
        // else if(counter ==4){
        //   m_LedBuffer.setRGB((starterLED+i)%53, 114, 40,141);
        // }
        // else if(counter ==5){
        //   m_LedBuffer.setRGB((starterLED+i)%53, 141, 44,114);
        // }
        // else if(counter ==6){
        //   m_LedBuffer.setRGB((starterLED+i)%53, 169, 48,86);
        // }
        // else if(counter ==7){
        //   m_LedBuffer.setRGB((starterLED+i)%53, 197, 53,58);
        // }
        // else if(counter ==8){
        //   m_LedBuffer.setRGB((starterLED+i)%53, 224, 57,31);
        // }
        

        // else if(counter ==9){
        //   m_LedBuffer.setRGB((starterLED+i)%53, 252, 61,3);
        // }
        // else if(counter ==10){
        //   m_LedBuffer.setRGB((starterLED+i)%53, 252, 61,3);
        // }
        // else if(counter ==11){
        //   m_LedBuffer.setRGB((starterLED+i)%53, 252, 61,3);
          
        // }


        // else if(counter ==12){
        //   m_LedBuffer.setRGB((starterLED+i)%53, 224, 57,31);
        // }
        // else if(counter ==13){
          
        //   m_LedBuffer.setRGB((starterLED+i)%53, 197, 53,58);
        // }
        // else if(counter ==14){
          
        //   m_LedBuffer.setRGB((starterLED+i)%53, 169, 48,86);
        // }
        // else if(counter ==15){
          
        //   m_LedBuffer.setRGB((starterLED+i)%53, 141, 44,114);
        // }
        // else if(counter ==16){
          
        //   m_LedBuffer.setRGB((starterLED+i)%53, 114, 40,141);
        // }
        // else if(counter ==17){
          
        //   m_LedBuffer.setRGB((starterLED+i)%53, 86, 36,169);
        // }
        // else if(counter ==18){
          
        //   m_LedBuffer.setRGB((starterLED+i)%53, 58, 31,197);
        // }
        // else if(counter ==19){
          
        //   m_LedBuffer.setRGB((starterLED+i)%53, 31, 27,224);
        //   counter = -3;
        // }
        // counter++;


        if (i % 7 == 0) {
          blue = !blue;
        }
        if (blue) {
          m_LedBuffer.setRGB((starterLED + i) % 60, Constants.LED.BlueR, Constants.LED.BlueG,
              Constants.LED.BlueB);
        } else {
          m_LedBuffer.setRGB((starterLED + i) % 60, Constants.LED.OrangeR, Constants.LED.OrangeG,
              Constants.LED.OrangeB);
        }
      }
      starterLED++;
    }
    
    m_Led.setData(m_LedBuffer);
    
    }

  }

  @Override
  public void disabledInit() {
    Constants.LED.LEDStatus = 0;
  }

  @Override
  public void disabledPeriodic() {
  }

  @Override
  public void autonomousInit() {
    m_robotContainer.m_swerve.resetEncoders();
    m_robotContainer.m_swerve.setAutonCurrent();
    Constants.LED.LEDStatus = 4;
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
    m_robotContainer.m_swerve.resetEncoders();
    m_robotContainer.m_swerve.setTeleopCurrent();
  }

  @Override
  public void teleopPeriodic() {
  }

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {
  }

  @Override
  public void simulationInit() {
  }

  @Override
  public void simulationPeriodic() {
  }

  public static void setLEDs() {
    if (Constants.LED.LEDStatus != 0) {
      for (var i = 0; i < m_LedBuffer.getLength(); i++) {
        if (Constants.LED.LEDStatus == 1) {
          m_LedBuffer.setRGB(i, Constants.LED.GreenR, Constants.LED.GreenG, Constants.LED.GreenB);
        } else if (Constants.LED.LEDStatus == 2) {
          m_LedBuffer.setRGB(i, Constants.LED.PurpleR, Constants.LED.PurpleG, Constants.LED.PurpleB);
        } else if (Constants.LED.LEDStatus == 3) {
          m_LedBuffer.setRGB(i, Constants.LED.YellowR, Constants.LED.YellowG, Constants.LED.YellowB);
        } else if (Constants.LED.LEDStatus == 4) {
          m_LedBuffer.setRGB(i, Constants.LED.RedR, Constants.LED.RedG, Constants.LED.RedB);
        }
      }
    } else {
      boolean blue = false;
      for (var i = 0; i < m_LedBuffer.getLength(); i++) {
        if (i % 7 == 0) {
          blue = !blue;
        }
        if (blue) {
          m_LedBuffer.setRGB((starterLED + i) % 60, Constants.LED.BlueR, Constants.LED.BlueG,
              Constants.LED.BlueB);
        } else {
          m_LedBuffer.setRGB((starterLED + i) % 60, Constants.LED.OrangeR, Constants.LED.OrangeG,
              Constants.LED.OrangeB);
        }
      }
      starterLED++;
    }
    m_Led.setData(m_LedBuffer);
  }

}
