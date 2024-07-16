package frc.robot.containers;

import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.Constants;
import frc.lib.Constants.AutonomousOptions;
import frc.lib.config.RobotConfig;
import frc.lib.config.krakenTalonConstants;
import frc.robot.classes.ColorSensorController;
import frc.robot.classes.Pigeon2Handler;
import frc.robot.classes.ToggleHandler;
import frc.robot.commands.Arm.AmpPosition;
import frc.robot.commands.Auto.RevAuto;
import frc.robot.commands.Auto.ShootAuto;
import frc.robot.commands.CPX.CpxSet;
import frc.robot.commands.CommandGroups.IntakeCommands.IntakeCommandGroup;
import frc.robot.commands.CommandGroups.IntakeCommands.IntakeNoteCommandGroup;
import frc.robot.commands.CommandGroups.IntakeCommands.SendBackCommandGroup;
import frc.robot.commands.CommandGroups.IntakeCommands.ShuffleNote;
import frc.robot.commands.CommandGroups.ShootCommands.PrepareShootCommandGroup;
import frc.robot.commands.Drive.AutoSwerve;
import frc.robot.commands.Drive.HybridSwerve;
import frc.robot.commands.Shooter.ShootAmp;

import frc.robot.commands.Indexer.FeedNote;
import frc.robot.commands.Intake.IntakeNote;
import frc.robot.commands.Intake.RejectNoteIntake;
import frc.robot.commands.Shooter.PassNote;
import frc.robot.commands.Shooter.ShootAmp;
import frc.robot.hybrid.BlendedControl;
import frc.robot.hybrid.HybridModes;
import frc.robot.hybrid.ControlVector;
import frc.robot.subsystems.*;

import java.sql.Driver;
import java.util.function.DoubleSupplier;

public class RobotContainerTeleop {
    /* Controllers */
    private final CommandXboxController pilot = new CommandXboxController(0);
    private final CommandXboxController copilot = new CommandXboxController(1);

    /* Instatiantion of pilot Triggers */

    private final int LeftYAxis = XboxController.Axis.kLeftY.value;
    private final int LeftXAxis = XboxController.Axis.kLeftX.value;
    private final int RightYAxis = XboxController.Axis.kRightY.value;
    private final int RightXAxis = XboxController.Axis.kRightX.value;

    private final int RightTriggerAxis = XboxController.Axis.kRightTrigger.value;
    private final int LeftTriggerAxis = XboxController.Axis.kLeftTrigger.value;

    private final Trigger pilotRightTrigger = pilot.rightTrigger();
    private final Trigger pilotLeftTrigger = pilot.leftTrigger();
    private final Trigger copilotRightTrigger = copilot.rightTrigger();
    private final Trigger copilotLeftTrigger = copilot.leftTrigger();

    private final Trigger pilotRightBumper = pilot.rightBumper();
    private final Trigger pilotLeftBumper = pilot.leftBumper();
    private final Trigger copilotRightBumper = copilot.rightBumper();
    private final Trigger copilotLeftBumper = copilot.leftBumper();

    private final Trigger pilotyButton = pilot.y();
    private final Trigger pilotaButton = pilot.a();
    private final Trigger pilotxButton = pilot.x();
    private final Trigger pilotbButton = pilot.b();

    private final Trigger copilotaButton = copilot.a();
    private final Trigger copilotxButton = copilot.x();
    private final Trigger copilotyButton = copilot.y();
    private final Trigger copilotbButton = copilot.b();


    private final Trigger pilotPOVup = pilot.povUp();
    private final Trigger pilotPOVleft = pilot.povLeft();
    private final Trigger pilotPOVright = pilot.povRight();
    private final Trigger pilotPOVdown = pilot.povDown();
    private final Trigger pilotPOVupright = pilot.povUpRight();
    private final Trigger pilotPOVupleft = pilot.povUpLeft();
    private final Trigger pilotPOVdownright = pilot.povDownRight();
    private final Trigger pilotPOVdownleft = pilot.povDownLeft();

    private final Trigger copilotPOVup = copilot.povUp();
    private final Trigger copilotPOVleft = copilot.povLeft();
    private final Trigger copilotPOVright = copilot.povRight();
    private final Trigger copilotPOVdown = copilot.povDown();

    /* Subsystems */
    private final Swerve SwerveSubsystem;
    private final Arm ArmSubsystem;
    private final Intake IntakeSubsystem;
    private final Shooter ShooterSubsystem;
    private final Climber ClimberSubsystem;
    private final Indexer IndexerSubsystem;
    private final Light LightSubsystem;
    private final Vision VisionSubsystem;
    private final CPX CPXSubsystem;

    /* State Machine */
    private final RobotStateMachine robotStateMachine;

    /* Util Classes */
    private final ColorSensorController colorSensorController;
    private final ToggleHandler shootAimOverideToggle;
    private final ToggleHandler intakeAimOverideToggle;
    private final ToggleHandler beamBreakToggle;
    private final ToggleHandler povControlToggle;
    
    //private final AutoCommands autoCommandsConstructor;

    /* Teleop Commands */
    private final IntakeCommandGroup intakeCommand;
    private final PrepareShootCommandGroup prepareShootCommand;
    private final PrepareShootCommandGroup secondprepareShootCommand;
    private final FeedNote feedNoteCommand;
    private final FeedNote feedNoteCopilot;
    private final SendBackCommandGroup manualFeedBackCommand;
    private final RejectNoteIntake rejectNoteIntakeCommand;
    private final CpxSet cpxOn;
    private final CpxSet cpxOff;
    private final ShootAmp shootAmp;
    private final ShuffleNote shuffleNote;
    private final PassNote passNote;
    private final PassNote passNote2;


    private double autoPOVAngle;
    private ControlVector AutoPOVInfluence;

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainerTeleop(RobotConfig robotConfig) {

        /* Util Classes */
        shootAimOverideToggle = new ToggleHandler();
        intakeAimOverideToggle = new ToggleHandler();
        beamBreakToggle = new ToggleHandler();
        povControlToggle = new ToggleHandler();
        Pigeon2 gyro = new Pigeon2(krakenTalonConstants.Swerve.pigeonID);
        colorSensorController = new ColorSensorController(Constants.colorSensorConfig, beamBreakToggle);
        AutoPOVInfluence = new ControlVector();

        DriverStation.Alliance alliance = DriverStation.Alliance.Blue;
        if (DriverStation.getAlliance().isPresent()) {
            alliance = DriverStation.getAlliance().get();
        }

        /* Subsystems */
        SwerveSubsystem = new Swerve(robotConfig.ctreConfigs, gyro);
        ArmSubsystem = new Arm(Constants.armConfig);
        IntakeSubsystem = new Intake(Constants.intakeConfig, colorSensorController);
        ShooterSubsystem = new Shooter(Constants.shooterConfig);
        IndexerSubsystem = new Indexer(Constants.indexerConfig);
        ClimberSubsystem = new Climber(Constants.climberConfig, robotConfig.dashboardConfig);
        LightSubsystem = new Light(Constants.lightConfig, colorSensorController);
        VisionSubsystem = new Vision(Constants.visionConfig, alliance);
        CPXSubsystem = new CPX(3); // TODO create CpxConfig

        /* State Machine */
        robotStateMachine = new RobotStateMachine();

        /* Teleop Commands */
        intakeCommand = new IntakeCommandGroup(IndexerSubsystem, IntakeSubsystem, ShooterSubsystem, LightSubsystem, pilot);
        prepareShootCommand = new PrepareShootCommandGroup(ArmSubsystem, IndexerSubsystem, IntakeSubsystem, ShooterSubsystem, pilot);
        feedNoteCommand = new FeedNote(IndexerSubsystem, pilot);
        manualFeedBackCommand = new SendBackCommandGroup(IndexerSubsystem, ShooterSubsystem);
        rejectNoteIntakeCommand = new RejectNoteIntake(IntakeSubsystem);
        cpxOn = new CpxSet(CPXSubsystem, true);
        cpxOff = new CpxSet(CPXSubsystem, false);
        shuffleNote = new ShuffleNote(IndexerSubsystem, ShooterSubsystem);
        secondprepareShootCommand = new PrepareShootCommandGroup(ArmSubsystem, IndexerSubsystem, IntakeSubsystem, ShooterSubsystem, pilot);
        shootAmp = new ShootAmp(ShooterSubsystem);
        passNote = new PassNote(ShooterSubsystem, copilot);
        feedNoteCopilot = new FeedNote(IndexerSubsystem, copilot);
        passNote2 = new PassNote(ShooterSubsystem, copilot);


        /* Command Constructor for Autos */
        //autoCommandsConstructor = new AutoCommands(SwerveSubsystem, ArmSubsystem, IndexerSubsystem, ShooterSubsystem, IntakeSubsystem, DriverStation.getAlliance().get());

        // Mode names in variables to prevent typos
        String modeDriverActive = "DriverControlActive";
        String modeDriverInactive = "DriverControlInactive";
        String modeIntakeAimInactive = "IntakeAimInactive";
        String modeIntakeAimActive = "IntakeAimActive";
        String modeShootAimInactive = "ShootAimInactive";
        String modeShootAimActive = "ShootAimActive";
        String modeShootDistanceInactive = "ShootDistanceInactive";
        String modeShootDistanceActive = "ShootDistanceActive";


        // Each mode describes an amount of influence that may be applied to each control
        HybridModes modes = new HybridModes();
        modes.addMode(modeDriverActive, ControlVector.fromFieldRelative(1.0, 1.0, 1.0).setSwerveRobotX(0.0).setSwerveRobotY(0.0));
        modes.addMode(modeDriverInactive, ControlVector.fromFieldRelative(1.0, 1.0, 1.0).setSwerveRobotX(0.0).setSwerveRobotY(0.0));
        modes.addMode(modeIntakeAimInactive, ControlVector.fromFieldRelative(0.0, 0.0, 0.0));
        modes.addMode(modeIntakeAimActive, ControlVector.fromFieldRelative(0.0, 0.0, 0.0));
        modes.addMode(modeShootAimInactive, ControlVector.fromFieldRelative(0.0, 0.0, 0.0));
        modes.addMode(modeShootAimActive, ControlVector.fromFieldRelative(0.0, 0.0, 0.0));
        // modes.addMode(modeShootDistanceInactive, ControlVector.fromRobotRelative(0, 0, 0));
        // modes.addMode(modeShootDistanceActive, ControlVector.fromRobotRelative(0,1, 0));


        // Each entry in the BlendedControl contributes some output to the Robot's movements
        BlendedControl blendedControl = new BlendedControl();
        blendedControl.addComponent(
                // Teleop Driver component
                () -> {
                    // double fieldX = MathUtil.applyDeadband(-pilot.getRawAxis(LeftXAxis), 0.1) * 4.5;
                    // double fieldY = MathUtil.applyDeadband(-pilot.getRawAxis(LeftYAxis), 0.1) * 4.5;
                    // double robotY = 0;
                    // double rot = MathUtil.applyDeadband(-pilot.getRawAxis(RightXAxis), 0.1) * 5;
                    // if (pilotLeftBumper.getAsBoolean()) {
                    //     robotY += 0.5;
                    // }
                    // return ControlVector.fromFieldRelative(fieldX, fieldY, rot).setSwerveRobotY(-robotY);

                    double X = MathUtil.applyDeadband(-pilot.getRawAxis(LeftXAxis), 0.1) * 4.5;
                    double Y = MathUtil.applyDeadband(-pilot.getRawAxis(LeftYAxis), 0.1) * 4.5;
                    double pilotrot = MathUtil.applyDeadband(-pilot.getRawAxis(RightXAxis), 0.1) * 6;
                    if (pilotLeftBumper.getAsBoolean()) {
                        return ControlVector.fromRobotRelative(-X, -Y, pilotrot);
                    }
                    return ControlVector.fromFieldRelative(X, Y, pilotrot);
                    
                },
                // TValue describes how much influence the Teleop Driver component has
                () -> {
                    double aimT = MathUtil.applyDeadband(pilot.getRawAxis(LeftTriggerAxis), 0.1);
                    ControlVector aimBlend = modes.interpolate(modeDriverActive, modeDriverInactive, aimT);

                    double armT = ArmSubsystem.percentRaised();
                    ControlVector armBlend = aimBlend.interpolate(ControlVector.fromFieldRelative(0.25,0.25,0.25), armT);
                    return armBlend;
                }
        );


        // Long range auto-aiming: Lifting the arm
        blendedControl.addComponent(
                // PValue
                () -> {
//                    setpoint += copilot.getRawAxis(LeftYAxis);
                    ArmSubsystem.setTargetAngle(Rotation2d.fromRotations(Constants.armConfig.intakeAngle));
                    if (pilotPOVup.getAsBoolean()) {
                        ArmSubsystem.setControlType(true);
                        ArmSubsystem.setTargetAngle(Rotation2d.fromRotations(Constants.armConfig.ampAngle));
                    }
                    // if (copilotLeftBumper.getAsBoolean()){
                    //     ArmSubsystem.setControlType(false);
                    //     ArmSubsystem.setTargetAngle(Rotation2d.fromRotations(VisionSubsystem.getArmAngleForShoot()));
                    // }
                    // if(pilotRightTrigger.getAsBoolean() && !copilotLeftTrigger.getAsBoolean()) {
                    //     ArmSubsystem.setControlType(false);
                    //     ArmSubsystem.setTargetAngle(Rotation2d.fromRotations(VisionSubsystem.getArmAngleForShoot()));
                    // }
                    // Arm power from PID to target angle
                    double armPower = ArmSubsystem.getArmPowerToTarget();
                    return new ControlVector().setArmPower(armPower);
                },
                // TValue
                () -> {
                    
                        return new ControlVector().setArmPower(1);
                    
                }
        );

        HybridSwerve hybridSwerve = new HybridSwerve(SwerveSubsystem, blendedControl);
        SwerveSubsystem.setDefaultCommand(hybridSwerve);

        ClimberSubsystem.setDefaultCommand(
                new InstantCommand(() -> ClimberSubsystem.joystickControl(MathUtil.applyDeadband(copilot.getRawAxis(RightYAxis), 0.1)), ClimberSubsystem)
        );

        ArmSubsystem.setDefaultCommand(new InstantCommand(() -> {
            ControlVector control = blendedControl.solve();
            ArmSubsystem.moveArm(control.armPower());
//             ArmSubsystem.moveArm(MathUtil.applyDeadband(copilot.getRawAxis(LeftYAxis), 0.1));
        }, ArmSubsystem));

        configureButtonBindings();
    }

    private void configureButtonBindings() {
        /* pilot Buttons */
        pilotLeftTrigger.onTrue(intakeCommand);
        pilotRightTrigger.whileTrue(passNote);
        pilotRightBumper.onTrue(new SequentialCommandGroup(feedNoteCommand.withTimeout(1), new InstantCommand(LightSubsystem::setWhite)));
        pilotaButton.whileTrue(rejectNoteIntakeCommand);
        pilotyButton.onTrue(new InstantCommand(SwerveSubsystem::zeroHeading));
        //pilotbButton.whileTrue(shootAmp);

        /* Copilot Buttons */
        pilotxButton.onTrue(manualFeedBackCommand.withTimeout(0.7));
        copilotPOVleft.onTrue(cpxOff);
        copilotPOVright.onTrue(cpxOn);
        copilotPOVup.onTrue(new InstantCommand(shootAimOverideToggle::toggle));
        copilotPOVdown.onTrue(new InstantCommand(intakeAimOverideToggle::toggle));
        copilotPOVright.onTrue(new InstantCommand(() -> beamBreakToggle.toggle()));
        copilotRightTrigger.whileTrue(secondprepareShootCommand);
        pilotbButton.onTrue(shuffleNote);
        //copilotLeftTrigger.whileTrue(shootAmp);
        copilotxButton.onTrue(new SequentialCommandGroup(feedNoteCopilot.withTimeout(1), new InstantCommand(LightSubsystem::setWhite)));
        copilotLeftBumper.whileTrue(passNote2);
    }
    public Command getAutonomousCommand(AutonomousOptions plan) { 

        return shootNote();
    }


    public void robotPeriodic() {
        VisionSubsystem.periodic();
    }

    public Command twoNoteCenterAuto() {
        return new SequentialCommandGroup(shootNote(),scoreCenterNote());
    }

    public Command threeNoteLeftAuto() {
        return new SequentialCommandGroup(shootNote(),scoreCenterNote(), scoreLeftNote());
    }

    public Command threeNoteRightAuto() {
        return new SequentialCommandGroup(shootNote(),scoreCenterNote(), scoreRightNote());
    }

    public Command fourNoteAuto() {
        return new SequentialCommandGroup(shootNote(),scoreCenterNote(), scoreLeftNote(), scoreRightNote());
    }

    public Command shootNote (){
        return new SequentialCommandGroup(
            new RevAuto(ShooterSubsystem).withTimeout(1),
            new ShootAuto(ShooterSubsystem, IndexerSubsystem).withTimeout(0.5)
        );
    }






    public Command scoreCenterNote() {
        Command backupAndIntake = new ParallelDeadlineGroup(
            backupToCenterNote(),
            sendNoteBack().withTimeout(0.5),
            new IntakeNoteCommandGroup(IntakeSubsystem, IndexerSubsystem).withTimeout(2)
        );

        Command returnToSpeakerAndShuffle = new ParallelDeadlineGroup(
            centerNoteReturnToSpeaker(),
            new IntakeNoteCommandGroup(IntakeSubsystem, IndexerSubsystem).withTimeout(2),
            sendNoteBack()
        );

        return new SequentialCommandGroup(
            backupAndIntake.withTimeout(2.125),
            returnToSpeakerAndShuffle.withTimeout(2.125),
            new ShuffleNote(IndexerSubsystem, ShooterSubsystem),
            shootNote()
        );
    }

    public Command waitCommand() {
        return new Command() {
            
        };
    }

    public Command taxi(){
        return new SequentialCommandGroup(
            shootNote(),
            waitCommand().withTimeout(3),
            new AutoSwerve(SwerveSubsystem, 0, -0.4, 0, false).withTimeout(15)
        );
    }

    public Command scoreLeftNote() {
        Command backupAndIntake = new ParallelDeadlineGroup(
            backupToLeftNote(),
            new IntakeNoteCommandGroup(IntakeSubsystem, IndexerSubsystem).withTimeout(2)
        );

        Command returnToSpeakerAndShuffle = new ParallelDeadlineGroup(
            leftNoteReturnToSpeaker(),
            new IntakeNoteCommandGroup(IntakeSubsystem, IndexerSubsystem).withTimeout(2),
            sendNoteBack()
        );

        return new SequentialCommandGroup(
            backupAndIntake.withTimeout(2.125),
            returnToSpeakerAndShuffle.withTimeout(2.125),
            new ShuffleNote(IndexerSubsystem, ShooterSubsystem).withTimeout(3),
            shootNote()
        );
    }

    public Command scoreRightNote() {
        Command backupAndIntake = new ParallelDeadlineGroup(
            backupToRightNote(),
            new IntakeNoteCommandGroup(IntakeSubsystem, IndexerSubsystem).withTimeout(2)
        );

        Command returnToSpeakerAndShuffle = new ParallelDeadlineGroup(
            rightNoteReturnToSpeaker(),
            new IntakeNoteCommandGroup(IntakeSubsystem, IndexerSubsystem).withTimeout(2),
            sendNoteBack()
        );

        return new SequentialCommandGroup(
            backupAndIntake.withTimeout(2.125),
            returnToSpeakerAndShuffle.withTimeout(2.125),
            new ShuffleNote(IndexerSubsystem, ShooterSubsystem).withTimeout(3),
            shootNote()
        );
    }

    public Command backupToCenterNote() {
        return new AutoSwerve(SwerveSubsystem, 0, -0.3, 0.1,false);
    }

    public Command centerNoteReturnToSpeaker() {
        return new AutoSwerve(SwerveSubsystem, 0, 0.3, 0,false);
    }

    public Command backupToLeftNote() {
        return new AutoSwerve(SwerveSubsystem, -0.23*1.5, -0.175*1.5, 0.1,false);
    }

    public Command leftNoteReturnToSpeaker() {
        return new AutoSwerve(SwerveSubsystem, 0.245*1.5, 0.19*1.5, 0.1,false);
    }

    public Command backupToRightNote() {
        return new AutoSwerve(SwerveSubsystem, 0.23*1.5, -0.175*1.5, -0.1,false);
    }

    public Command rightNoteReturnToSpeaker() {
        return new AutoSwerve(SwerveSubsystem, -0.245*1.5, 0.19*1.5, -0.1,false);
    }


    public Command sendNoteBack() {
        return new InstantCommand(() -> IndexerSubsystem.sendBack());
    }
}
