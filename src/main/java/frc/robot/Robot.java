// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;

  @Override
  public void robotInit() {
    m_robotContainer = new RobotContainer();
    //DataLogManager.start();
    //DataLog log = DataLogManager.getLog();
    //DriverStation.startDataLog(log);

    //boolean fileOnly = false;
    //boolean lazyLogging = false;
    //Monologue.setupMonologue(this, "Robot", fileOnly, lazyLogging);
    SmartDashboard.putNumber("distance P", 3);
    SmartDashboard.putNumber("yaw P", 3);

  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run(); 

    //m_robotContainer.drivetrain.updateFieldWidget();

    // setFileOnly is used to shut off NetworkTables broadcasting for most logging calls.
    // Basing this condition on the connected state of the FMS is a suggestion only.
    //Monologue.setFileOnly(DriverStation.isFMSAttached());
    // This method needs to be called periodically, or no logging annotations will process properly.
    //Monologue.updateAll();
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    m_robotContainer.configureAutoSettings();
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    m_robotContainer.configureTeleopSettings();
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}

  @Override
  public void simulationPeriodic() {}
}
