package frc.robot.containers;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.Constants;
import frc.lib.Constants.AutonomousOptions;
import frc.lib.config.RobotConfig;
import frc.lib.config.krakenTalonConstants;
import frc.robot.classes.ColorSensorController;
import frc.robot.classes.Pigeon2Handler;
import frc.robot.classes.ToggleHandler;
import frc.robot.commands.SendBackCommand;
import frc.robot.commands.WaitCommand;
import frc.robot.commands.Auto.RevAuto;
import frc.robot.commands.Auto.ShootAuto;
import frc.robot.commands.CPX.CpxSet;
import frc.robot.commands.CommandGroups.IntakeCommands.IntakeCommandGroup;
import frc.robot.commands.CommandGroups.IntakeCommands.IntakeNoteCommandGroup;
import frc.robot.commands.CommandGroups.IntakeCommands.SendBackCommandGroup;
import frc.robot.commands.CommandGroups.IntakeCommands.ShuffleNote;
import frc.robot.commands.CommandGroups.ShootCommands.PrepareShootCommandGroup;
import frc.robot.commands.Drive.HybridSwerve;

import frc.robot.commands.HybridAuto.AutoDrive;
import frc.robot.commands.HybridAuto.AutoVectors;
import frc.robot.commands.Indexer.FeedNote;
import frc.robot.commands.Indexer.IndexNote;
import frc.robot.commands.Intake.IntakeNote;
import frc.robot.commands.Intake.RejectNoteIntake;
import frc.robot.commands.Shooter.PassNote;
import frc.robot.hybrid.BlendedControl;
import frc.robot.hybrid.ControlVector;
import frc.robot.subsystems.*;

public class RobotContainer {
    public static final double CLOCKWISE = 1.0;
    public static final double COUNTER_CLOCKWISE = -1.0;

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
    private final Trigger copilotaButton = copilot.a();

    private final Trigger copilotPOVup = copilot.povUp();
    private final Trigger copilotPOVleft = copilot.povLeft();
    private final Trigger copilotPOVright = copilot.povRight();
    private final Trigger copilotPOVdown = copilot.povDown();

    /* Subsystems */
    private final Swerve SwerveSubsystem;
    private final Arm ArmSubsystem;
    private final Intake IntakeSubsystem;
    private final Shooter ShooterSubsystem;
    // private final Climber ClimberSubsystem;
    private final Indexer IndexerSubsystem;
    private final Light LightSubsystem;
    private final Vision VisionSubsystem;
    private final CPX CPXSubsystem;

    /* Util Classes */
    private final ColorSensorController colorSensorController;
    private final ToggleHandler shootAimOverideToggle;
    private final ToggleHandler intakeAimOverideToggle;
    private final ToggleHandler beamBreakToggle;
    
    /* Teleop Commands */
    private final IntakeCommandGroup intakeCommand;
    private final PrepareShootCommandGroup prepareShootCommand;
    private final PrepareShootCommandGroup secondprepareShootCommand;
    private final FeedNote feedNoteCommand;
    private final SendBackCommandGroup manualFeedBackCommand;
    private final RejectNoteIntake rejectNoteIntakeCommand;
    private final CpxSet cpxOn;
    private final CpxSet cpxOff;
    private final PassNote passNoteCommand;
    private final ShuffleNote shuffleNote;

    private ControlVector autoDriveControlVector;
    private ControlVector autoDriveInfluenceVector;
    private ControlVector autoIntakeAimInfluence;
    private ControlVector autoShootAimInfluence;

    // TODO Add ChassisSpeeds pValue and tValue from PathPlanner
    private ChassisSpeeds pathPlannerSpeed;
    private ChassisSpeeds pathPlannerInfluence;

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer(RobotConfig robotConfig) {

        /* Util Classes */
        shootAimOverideToggle = new ToggleHandler();
        intakeAimOverideToggle = new ToggleHandler();
        beamBreakToggle = new ToggleHandler();
        Pigeon2 gyro = new Pigeon2(krakenTalonConstants.Swerve.pigeonID);
        colorSensorController = new ColorSensorController(Constants.colorSensorConfig, beamBreakToggle);

        DriverStation.Alliance alliance = DriverStation.Alliance.Blue;
        if (DriverStation.getAlliance().isPresent()) {
            alliance = DriverStation.getAlliance().get();
        }

        pathPlannerSpeed = new ChassisSpeeds();
        pathPlannerInfluence = new ChassisSpeeds(1, 1, 0);

        /* Subsystems */
        SwerveSubsystem = new Swerve(robotConfig.ctreConfigs, gyro, (pathSpeeds) -> {
            // Each frame, assign the latest ChassisSpeeds from PathPlanner
            pathPlannerSpeed = pathSpeeds;
        });
        ArmSubsystem = new Arm(Constants.armConfig);
        IntakeSubsystem = new Intake(Constants.intakeConfig, colorSensorController);
        ShooterSubsystem = new Shooter(Constants.shooterConfig);
        IndexerSubsystem = new Indexer(Constants.indexerConfig);
        //ClimberSubsystem = new Climber(Constants.climberConfig, robotConfig.dashboardConfig);
        LightSubsystem = new Light(Constants.lightConfig, colorSensorController);
        VisionSubsystem = new Vision(Constants.visionConfig, alliance);
        CPXSubsystem = new CPX(3); // TODO create CpxConfig

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
        passNoteCommand = new PassNote(ShooterSubsystem);

        // Influence vectors for blended control
        ControlVector driverActive = ControlVector.fromFieldRelative(1.0, 1.0, 1.0).setSwerveRobotX(0.5).setSwerveRobotY(0.5);
        ControlVector driverInactive = ControlVector.fromFieldRelative(1.0, 0.5, 0.5).setSwerveRobotX(0.5).setSwerveRobotY(0.5);
        ControlVector intakeAimInactive = ControlVector.fromFieldRelative(0.0, 0.0, 0.0);
        ControlVector intakeAimActive = ControlVector.fromFieldRelative(0.0, 0.0, 1.0);
        ControlVector shootAimInactive = ControlVector.fromFieldRelative(0.0, 0.0, 0.0);
        ControlVector shootAimActive = ControlVector.fromFieldRelative(0.0, 0.0, 1.0);

        // A ControlVector that is updated by Auto Commands during auto mode
        autoDriveControlVector = new ControlVector();
        autoDriveInfluenceVector = new ControlVector();
        autoIntakeAimInfluence = new ControlVector();
        autoShootAimInfluence = new ControlVector();

        // Each entry in the BlendedControl contributes some output to the Robot's movements
        BlendedControl blendedControl = new BlendedControl();
        blendedControl.addControlVectorComponent(
                // Teleop Driver component
                () -> {
                    double X = MathUtil.applyDeadband(-pilot.getRawAxis(LeftXAxis), 0.1) * 4.5;
                    double Y = MathUtil.applyDeadband(-pilot.getRawAxis(LeftYAxis), 0.1) * 4.5;
                    double rot = MathUtil.applyDeadband(-pilot.getRawAxis(RightXAxis), 0.1) * 5;
                    if (pilotLeftBumper.getAsBoolean()) {
                        return ControlVector.fromRobotRelative(-X, -Y, rot);
                    }
                    return ControlVector.fromFieldRelative(X, Y, rot);
                },
                // TValue describes how much influence the Teleop Driver component has
                () -> {
                    if (DriverStation.isAutonomous()) {
                        // In auto mode, drive component has zero influence
                        return new ControlVector();
                    }

                    double aimT = MathUtil.applyDeadband(pilot.getRawAxis(LeftTriggerAxis), 0.1);
                    double armT = ArmSubsystem.percentRaised();
                    ControlVector blend = driverActive.interpolate(driverInactive, aimT)
                            .interpolate(ControlVector.fromFieldRelative(0.25,0.25,0.25), armT);

                    return blend;
                }

        );

        // Input from autonomous planner, only active in auto mode
        blendedControl.addControlVectorComponent(
                () -> {
                    return autoDriveControlVector;
                },
                () -> {
                    return autoDriveInfluenceVector;
                }
        );

        // Component for auto-aiming intake at note
        blendedControl.addControlVectorComponent(
                () -> ControlVector.fromFieldRelative(0, 0, VisionSubsystem.getNoteAimRotationPower()),
                () -> {
                    if (DriverStation.isAutonomous()) {
                        return autoIntakeAimInfluence;
                    }

                    double t = MathUtil.applyDeadband(pilot.getRawAxis(LeftTriggerAxis), 0.1);
                    if (intakeAimOverideToggle.get()) {
                        t=0;
                    }
                    return intakeAimInactive.interpolate(intakeAimActive, t);
                }
        );

        // Component for auto-aiming shooter at speaker
        blendedControl.addControlVectorComponent(
                () -> ControlVector.fromFieldRelative(0, 0, VisionSubsystem.getAngleToShootAngle()),
                () -> {
                    if (DriverStation.isAutonomous()) {
                        return autoShootAimInfluence;
                    }
                    double t = 0;
                    double t1 = MathUtil.applyDeadband(pilot.getRawAxis(RightTriggerAxis), 0.1);
                    double t2 = MathUtil.applyDeadband(copilot.getRawAxis(RightTriggerAxis), 0.1);
                    SmartDashboard.putBoolean("modeIntakeAimActive", shootAimOverideToggle.get());

                    if (shootAimOverideToggle.get()) {
                        t = 0;
                    } else {
                        t = (t1 > t2) ? t1 : t2;
                    }

                    return shootAimInactive.interpolate(shootAimActive, t);
                }
        );

        // Long range auto-aiming: Lifting the arm
        blendedControl.addControlVectorComponent(
                // PValue
                () -> {
                    ArmSubsystem.setTargetAngle(Rotation2d.fromRotations(Constants.armConfig.intakeAngle));
                    if (copilotLeftTrigger.getAsBoolean()) {
                        ArmSubsystem.setControlType(true);
                        ArmSubsystem.setTargetAngle(Rotation2d.fromRotations(Constants.armConfig.ampAngle));
                    }
                    if (copilotLeftBumper.getAsBoolean()){
                        ArmSubsystem.setControlType(false);
                        ArmSubsystem.setTargetAngle(Rotation2d.fromRotations(VisionSubsystem.getArmAngleForShoot()));
                    }
                    if(pilotRightTrigger.getAsBoolean() && !copilotLeftTrigger.getAsBoolean()) {
                        ArmSubsystem.setControlType(false);
                        ArmSubsystem.setTargetAngle(Rotation2d.fromRotations(VisionSubsystem.getArmAngleForShoot()));
                    }
                    // Arm power from PID to target angle
                    double armPower = ArmSubsystem.getArmPowerToTarget();
                    return new ControlVector().setArmPower(armPower);
                },
                // TValue
                () -> {
                        return new ControlVector().setArmPower(1);
                }
        );

        blendedControl.addChassisSpeedsComponent(
            () -> {
                // TODO return the latest ChassisSpeeds from PathPlanner
                SmartDashboard.putString("coordinates", String.format("x:%f,y:%f,z:%f", pathPlannerSpeed.vxMetersPerSecond, pathPlannerSpeed.vyMetersPerSecond, pathPlannerSpeed.omegaRadiansPerSecond));
                return pathPlannerSpeed;
            },
            () -> {
                // TODO return a ChassisSpeeds influence vector
                // This should have full influence (1, 1, 1) for auto mode but no influence (0, 0, 0) for teleop mode
                if(!DriverStation.isAutonomous()){
                    return new ChassisSpeeds();
                }
                return pathPlannerInfluence;
            }
        );

        HybridSwerve hybridSwerve = new HybridSwerve(SwerveSubsystem, blendedControl);
        SwerveSubsystem.setDefaultCommand(hybridSwerve);

        // TODO: Re-enable climber
        // ClimberSubsystem.setDefaultCommand(
        //         new InstantCommand(() -> ClimberSubsystem.joystickControl(MathUtil.applyDeadband(copilot.getRawAxis(RightYAxis), 0.1)), ClimberSubsystem)
        // );

        ArmSubsystem.setDefaultCommand(new InstantCommand(() -> {
            ControlVector control = blendedControl.solveControlVectorInputs();
            ArmSubsystem.moveArm(control.armPower());
        }, ArmSubsystem));

        configureButtonBindings();
    }

    private void configureButtonBindings() {
        /* pilot Buttons */
        pilotLeftTrigger.onTrue(intakeCommand);
        pilotRightTrigger.whileTrue(prepareShootCommand);
        pilotRightBumper.onTrue(new SequentialCommandGroup(feedNoteCommand.withTimeout(1), new InstantCommand(LightSubsystem::setWhite)));
        pilotaButton.whileTrue(rejectNoteIntakeCommand);
        pilotyButton.onTrue(new InstantCommand(SwerveSubsystem::zeroHeading));

        /* Copilot Buttons */
        copilotRightBumper.onTrue(manualFeedBackCommand.withTimeout(0.7));
        copilotPOVleft.onTrue(cpxOff);
        copilotPOVright.onTrue(cpxOn);
        copilotPOVup.onTrue(new InstantCommand(shootAimOverideToggle::toggle));
        copilotPOVdown.onTrue(new InstantCommand(intakeAimOverideToggle::toggle));
        copilotPOVright.onTrue(new InstantCommand(() -> beamBreakToggle.toggle()));
        copilotRightTrigger.whileTrue(secondprepareShootCommand);
        copilotaButton.onTrue(shuffleNote);
    }

    public Command getAutonomousCommand(AutonomousOptions plan) {
        switch (plan) {
            case SHOOT_NOTE:
                return shoot();
            case TWO_NOTE_CENTER:
                return twoNoteCenterAuto();
            case THREE_NOTES_RIGHT:
                return threeNoteRightAuto();
            case THREE_NOTES_LEFT:
                return threeNoteLeftAuto();
            case FOUR_NOTES:
                return fourNoteAuto();
            case SEEK_NOTE:
                return seekNote();
            case SEEK_PICKUP_NOTE:
                return seekPickupNote();
            case SEEK_SPEAKER:
                return seekSpeaker();
            case DEBUG_DRIVE:
                return debugDrive();
            case SHUFFLE:
                return shuffle();
            case SHOOT:
                return shoot();
            case PICKUP:
                return pickup();
            case FOLLOWPATH:
                PathPlannerPath path = PathPlannerPath.fromPathFile("left-note-center");
                // Create a path following command using AutoBuilder. This will also trigger event markers.
                return AutoBuilder.followPath(path);

            default:
                return shoot();
        }
    }

    public void robotPeriodic() {
        VisionSubsystem.periodic();
    }

    public Command twoNoteCenterAuto() {
        return new SequentialCommandGroup(
            shoot(),
            scoreCenterNote()
        );
    }

    public Command threeNoteLeftAuto() {
        return new SequentialCommandGroup(
            shoot(),
            scoreCenterNote(),
            scoreLeftNote()
        );
    }

    public Command threeNoteRightAuto() {
        return new SequentialCommandGroup(
            shoot(),
            scoreCenterNote(),
            scoreRightNote()
        );
    }

    public Command fourNoteAuto() {
        return new SequentialCommandGroup(
            shoot(),
            scoreCenterNote(),
            moveForward().withTimeout(0.125),
            scoreLeftNote(),
            rotate(-0.9, .35),
            moveForward().withTimeout(0.25),
            scoreRightNote()
        );
    }

    /**
     * @param rotation the power to rotate before approaching a note
     * @param rotationTime the time the robot needs to rotate
     * @param approachTime the time for which to approach the note before aiming
     */
    public Command score(double rotation, double rotationTime, double approachTime) {
        return new SequentialCommandGroup(
            rotate(rotation, rotationTime),

            // Move forward without aiming to get close to the note
            new ParallelDeadlineGroup(
                moveForward().withTimeout(approachTime),
                intake(0.5),
                index().withTimeout(1)
            ),
            
            // Move forward while aiming at the note now that it's in camera range
            seekPickupNote().withTimeout(2), //was 2.125 without moveForward

            sendNoteBack().withTimeout(2),

            // After picking up the note, continue intaking it while returning to speaker
            // Return to speaker uses speaker aiming
            rotate(-rotation, .5),
            // drive(new ControlVector().setSwerveRotation(-rotation).setSwerveFieldX(-1)).withTimeout(0.5),
            sendNoteBack().withTimeout(.5),

            new ParallelDeadlineGroup(
                seekSpeaker().withTimeout(2.125),
                //pickup().withTimeout(2) // NOTE: Maybe reduce timeout
                sendNoteBack().withTimeout(1)
            ).withTimeout(2.125),

            //sendNoteBack().withTimeout(1),

            // Once returned to speaker, shoot the note
            shoot()
        );
    }

    // Score the center note from center position at the subwoofer
    public Command scoreCenterNote() {
        return score(0, 0, 1);
    }

    // Score the left note from center position at the subwoofer
    public Command scoreLeftNote() {
        return score(CLOCKWISE * 0.5, 1, 1);
    }

    // Score the right note from center position at the subwoofer
    public Command scoreRightNote() {
        return score(COUNTER_CLOCKWISE * 0.5, 1 ,1);
    }

    // Rotate from center position toward left note
    public Command rotateToLeftNote() {
        return rotate(COUNTER_CLOCKWISE * 0.5, .8);
    }

    // Rotate from center position toward right note
    public Command rotateToRightNote() {
        return rotate(CLOCKWISE * 0.5, .8);
    }

    public Command rotate(double power, double time) {
        return drive(new ControlVector().setSwerveRotation(power)).withTimeout(time);
    }

    // Drive aiming at a note while running intake
    public Command seekPickupNote() {
        return new ParallelDeadlineGroup(
            seekNote().withTimeout(2),
            index().withTimeout(1),
            intake(0.5).withTimeout(2)
        );
    }

    // Drive aiming at a note
    public Command seekNote() {
        return driveAuto(new ControlVector().setSwerveRobotY(-1), 0.7, 0);
    }

    // Drive aiming at the speaker
    public Command seekSpeaker() {
        return driveAuto(new ControlVector().setSwerveRobotY(1), 0, .95);
    }

    // Drive intake-forward full speed
    public Command moveForward() {
        return drive(new ControlVector().setSwerveRobotY(-1));
    }

    public Command debugDrive() {
        return drive(new ControlVector().setSwerveRobotY(0.5));
    }

    // Drive according to the given driveControl vector FOREVER
    // This REQUIRES a timeout if you want to stop
    public Command drive(ControlVector driveControl) {
        return driveAuto(driveControl, 0, 0);
    }

    /**
     * @param drive ControlVector for this drive command
     * @param noteAim influence to give to note aiming rotation
     * @param shootAim influence to give to shoot aiming rotation
     */
    public Command driveAuto(ControlVector drive, double noteAim, double shootAim) {
        return new SequentialCommandGroup(
            new AutoDrive(
                new AutoVectors(autoDriveControlVector, autoDriveInfluenceVector, autoIntakeAimInfluence, autoShootAimInfluence),
                new AutoVectors(
                    drive,
                    new ControlVector(1, 1, 1, 1, 1, 0), // Drive influence
                    new ControlVector().setSwerveRotation(noteAim), // Note aim influence
                    new ControlVector().setSwerveRotation(shootAim) // Shoot aim influence
                )
            ),
            new WaitCommand()
        );
    }

    // Shuffle the note, running it up and down in the indexer to un-squish a note
    public Command shuffle() {
        return new ShuffleNote(IndexerSubsystem, ShooterSubsystem);
    }

    // Rev the shooter without feeding the note into the shooter
    public Command rev() {
        return new RevAuto(ShooterSubsystem);
    }

    public Command shoot() {
        return new SequentialCommandGroup(
            rev().withTimeout(1),
            new ShootAuto(ShooterSubsystem, IndexerSubsystem).withTimeout(1)
        );
    }

    public Command pickup() {
        return new IntakeNoteCommandGroup(IntakeSubsystem, IndexerSubsystem);
    }

    public Command intake(double speed){
        return new IntakeNote(IntakeSubsystem, speed);
    }

    public Command index(){
        return new IndexNote(IndexerSubsystem);
    }

    public Command sendNoteBack() {
        return new SendBackCommand(IndexerSubsystem);
    }
}
