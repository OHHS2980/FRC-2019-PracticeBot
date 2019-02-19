/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.SerialPort.Port;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;

public class Robot extends TimedRobot {
  private static final int kMotorPort0 = 0;
  private static final int kMotorPort1 = 1;
  private static final int kMotorPort2 = 2;
  private static final int kMotorPort3 = 3;
  private static final int kJoystickPort = 0;

  private SpeedController m_motor0; //fl
  private SpeedController m_motor1; //bl
  private SpeedController m_motor2; //fr
  private SpeedController m_motor3; //br
  private DifferentialDrive m_drive;

  private Joystick m_joystick;

  private double startAngle = 0;
  private int[] ball = new int[4];
  private int[] panel1 = new int[4];
  private int[] panel2 = new int[4];
  private int center = 159;
  private ADXRS450_Gyro gyro;
  private final Timer m_timer = new Timer();
  SerialPort spi;
  

  @Override
  public void robotInit() {
    m_motor0 = new Spark(kMotorPort0); //fl
    m_motor1 = new Spark(kMotorPort1); //bl
    SpeedControllerGroup m_left = new SpeedControllerGroup(m_motor0, m_motor1);

    m_motor2 = new Spark(kMotorPort2); //fr
    m_motor3 = new Spark(kMotorPort3); //br
    SpeedControllerGroup m_right = new SpeedControllerGroup(m_motor2, m_motor3);
    m_drive = new DifferentialDrive(m_right, m_left);
    m_joystick = new Joystick(kJoystickPort);
    spi = new SerialPort(115200, Port.kUSB1);
    spi.enableTermination();
    CameraServer.getInstance().startAutomaticCapture(0);
    gyro = new ADXRS450_Gyro();
    gyro.getAngle();
    gyro.calibrate();
    gyro.reset();
  }


  @Override
  public void teleopInit() {
    m_timer.reset();
    m_timer.start();
    gyro.reset();
    startAngle = gyro.getAngle();
  }

  @Override
  public void teleopPeriodic() {
    String s = spi.readString();
    pixyData(s);
    if(s!="")
      SmartDashboard.putString("value", s);
    m_drive.arcadeDrive(m_joystick.getY(), m_joystick.getZ());
    if(m_joystick.getRawButtonPressed(1)) {
      hatchPlace();
    }
    if(m_joystick.getRawButtonPressed(2)) {
      turn90();
    } 
  }
  public void autonomousInit() {
    m_drive.setSafetyEnabled(true);
  }
  public void autonomousPeriodic() {
    String s = spi.readString();
    if(!s.isEmpty()) {
      SmartDashboard.putString("value", s);
      pixyData(s);
    }
    //m_drive.arcadeDrive(0,0);
    //center is at 159
    SmartDashboard.putNumber("x", ball[0]-165);
    SmartDashboard.putNumber("x corrected", ball[0]-center);
    double turn;
    double drive = 0;
    if(ball[0]-center<-7) {
      turn = -0.5;
      if(ball[0]-center>-20)
        turn = -0.45;
    }
    else if(ball[0]-center>7) {
      turn = 0.5;
      if(ball[0]-159<20)
        turn = 0.45;
    }
    else {
      turn = 0;
      drive = 0.5;
    }
    m_drive.arcadeDrive(drive,turn);
  }
  public void pixyData(String s){
    SmartDashboard.putNumber("length", s.length());
    if(!s.isEmpty() && s.length()==23 && s.charAt(0)=='P') {
      SmartDashboard.putString("parsed", s);
      int sig = Character.getNumericValue(s.charAt(3));

      //format "P1S0X000Y000W000H000N0\n"
      //        0123456789012345678901
      int x = Integer.valueOf(s.substring(5,8));
      int y = Integer.valueOf(s.substring(9,12));
      int width = Integer.valueOf(s.substring(13, 16));
      int height = Integer.valueOf(s.substring(17,20));
      int sigNum = Integer.valueOf(s.substring(21,22));
      /*int x = Integer.valueOf(s.substring(s.indexOf("X")+1,s.indexOf("Y")));
      int y = Integer.valueOf(s.substring(s.indexOf("Y")+1,s.indexOf("W")));
      int width = Integer.valueOf(s.substring(s.indexOf("W")+1,s.indexOf("H")));
      int height = Integer.valueOf(s.substring(s.indexOf("H")+1,s.indexOf("\n")));*/

      if(sig==1) {
        ball[0] = x;
        ball[1] = y;
        ball[2] = width;
        ball[3] = height;
      }
      else if(sig==2) {
        if(sigNum==0) {
          panel1[0] = x;
          panel1[1] = y;
          panel1[2] = width;
          panel1[3] = height;
        }
        else if(sigNum==1) {
          panel2[0] = x;
          panel2[1] = y;
          panel2[2] = width;
          panel2[3] = height;
        }/*
        if(panel2[0]<panel1[0]) {
          int temp[] = panel1.clone();
          panel1 = panel2.clone();
          panel2 = temp.clone();
        }*/
      }

    }
  }
  public void hatchPlace() {
    double start = m_timer.get();
    String s = spi.readString();
    if(!s.isEmpty()) {
      SmartDashboard.putString("value", s);
      pixyData(s);
    }
    while(Math.abs(159-(panel2[0]+panel1[0])/2)>3) {
      //get x value from pixy
      double turn = 0;
      if((panel2[0]+panel1[0])/2<159) {
        turn = 0.5;
      }
      else {
        turn = -0.5;
      }
      m_drive.arcadeDrive(turn, 0);
      if(m_timer.get()-start>2.5 || m_joystick.getRawButtonPressed(8)) {
        return;
      }
    }
    while(Math.abs(panel2[0]-panel1[0])>25) {
      m_drive.arcadeDrive(0.5, 0);
      if(m_timer.get()-start>4.5|| m_joystick.getRawButtonPressed(8)) {
        m_drive.arcadeDrive(0, 0);
        return;
      }
    }
    //place hatch
  }
  
  public void collectBall() {
    double start = m_timer.get();
    String s = spi.readString();
    SmartDashboard.putString("value", s);
    pixyData(s);
    while(Math.abs(159-(ball[0])/2)>3) {
      //get x value from pixy
      double turn = 0;
      if((ball[0])<159) {
        turn = 0.5;
      }
      else {
        turn = -0.5;
      }
      m_drive.arcadeDrive(turn, 0);
      if(m_timer.get()-start>2.5|| m_joystick.getRawButtonPressed(8)) {
        return;
      }
    }
    //Fix This!!!
    while(ball[1]>100) {
      m_drive.arcadeDrive(0.5, 0);
      //setIntake(-1.0)
      if(m_timer.get()-start>4.5 || m_joystick.getRawButtonPressed(8)) {
        m_drive.arcadeDrive(0, 0);
        //setIntake(0)
        return;
      }
    }
  }

  public void turn90() {
    double calcAngle = startAngle;
    while(calcAngle > 90) {
      calcAngle-=90;
    }
    while(gyro.getAngle()-calcAngle<90) {
      SmartDashboard.putNumber("Angle", gyro.getAngle());
      SmartDashboard.putNumber("getAngle-calcAngle", gyro.getAngle() - calcAngle);
      m_drive.arcadeDrive(0, -0.5);
    }
    m_drive.arcadeDrive(0, 0);
  }
}