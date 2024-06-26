package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.lib.Constants.AutonomousOptions;
import frc.lib.CtreConfigs;
import frc.lib.config.DashboardConfig;
import frc.lib.config.RobotConfig;
import frc.robot.containers.RobotContainerTeleop;
import edu.wpi.first.wpilibj.TimedRobot;
public class Robot extends TimedRobot {
    public final CtreConfigs ctreConfigs = new CtreConfigs();
    public final DashboardConfig dashboardConfig = new DashboardConfig();
    public final RobotConfig robotConfig = new RobotConfig(ctreConfigs, dashboardConfig);
    private Command m_autonomousCommand1;
    private Command m_autonomousCommand2;
    private Command m_InitCommand;
    private RobotContainerTeleop mRobotContainer;

    private final SendableChooser<AutonomousOptions> positionChooser = new SendableChooser<>();

    /**
     * This function is run when the robot is first started up and should be used for any
     * initialization code.
     */
    @Override
    public void robotInit() {
        // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
        // autonomous chooser on the dashboard.
        positionChooser.setDefaultOption("TwoNoteCenter", AutonomousOptions.TWO_NOTE_CENTER);
        positionChooser.addOption("ShootNote", AutonomousOptions.SHOOT_NOTE);
        positionChooser.addOption("shootandmovebackright side", AutonomousOptions.RIGHTSPEAKERSIDESHOOTANDMOVEBACK);
        positionChooser.addOption("twoNoteCenterAutoWithBackup", AutonomousOptions.SHOOT_NOTE_MOVEBACK);
        positionChooser.addOption("ThreeNoteRight", AutonomousOptions.THREE_NOTES_RIGHT);
        positionChooser.addOption("ThreeNoteLeft", AutonomousOptions.THREE_NOTES_LEFT);
        positionChooser.addOption("FourNoteAuto", AutonomousOptions.FOUR_NOTES);

        SmartDashboard.putData("AutonomousSelection", positionChooser);
        SmartDashboard.putString("Version", "2");
        mRobotContainer = new RobotContainerTeleop(robotConfig);

    }

    /**
     * This function is called every robot packet, no matter the mode. Use this for items like
     * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
     *
     * <p>This runs after the mode-specific periodic functions, but before LiveWindow and
     * SmartDashboard integrated updating.
     */
    @Override
    public void robotPeriodic() {
        // Runs the Scheduler.  This is responsible for polling buttons, adding newly scheduled
        // commands, running already-scheduled commands, removing finished or interrupted commands,
        // and running subsystem periodic() methods.  This must be called from the robot's periodic
        // block in order for anything in the Command-based framework to work.
        CommandScheduler.getInstance().run();
        mRobotContainer.robotPeriodic();
    }

    /** This function is called once each time the robot enters Disabled mode. */
    @Override
    public void disabledInit() {}

    @Override
    public void disabledPeriodic() {}

    /** This autonomous runs the autonomous command selected by your {@link RobotContainerTeleop} class. */
    @Override
    public void autonomousInit() {
        AutonomousOptions sp = positionChooser.getSelected();
        m_autonomousCommand1 = mRobotContainer.getAutonomousCommand(sp);
        //m_InitCommand = mRobotContainer.Initialize();

        // schedule the autonomous command (example)
        if (m_autonomousCommand1 != null) {
            //m_InitCommand.schedule();
            m_autonomousCommand1.schedule();
            // m_autonomousCommand2.schedule();
        }
    }

    /** This function is called periodically during autonomous. */
    @Override
    public void autonomousPeriodic() {}

    @Override
    public void teleopInit() {
        // This makes sure that the autonomous stops running when
        // teleop starts running. If you want the autonomous to
        // continue until interrupted by another command, remove
        // this line or comment it out.
        if (m_autonomousCommand1 != null && m_autonomousCommand2 != null) {
            m_autonomousCommand1.cancel();
            m_autonomousCommand2.cancel();
        }
    }

    /** This function is called periodically during operator control. */
    @Override
    public void teleopPeriodic() {}

    @Override
    public void testInit() {
        // Cancels all running commands at the start of test mode.
        CommandScheduler.getInstance().cancelAll();
    }

    /** This function is called periodically during test mode. */
    @Override
    public void testPeriodic() {}
}
