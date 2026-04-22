package frc.robot.commands;

import static frc.robot.commands.AutonomousBuilderConstants.k24ScoringTimer;
import static frc.robot.commands.AutonomousBuilderConstants.k60ScoringTimer;
import static frc.robot.commands.AutonomousBuilderConstants.k8ScoringTimer;
import static frc.robot.commands.AutonomousBuilderConstants.kAutoBufferTime;
import static frc.robot.commands.AutonomousBuilderConstants.kDepotIdleCooldown;
import static frc.robot.commands.AutonomousBuilderConstants.kLeaveSpeedsFwdBlue;
import static frc.robot.commands.AutonomousBuilderConstants.kLeaveTimer;
import static frc.robot.commands.AutonomousBuilderConstants.kOutpostIdleCooldown;
import static frc.robot.commands.AutonomousBuilderConstants.kPassBumpSpeedsFwdBlue;
import static frc.robot.commands.AutonomousBuilderConstants.kPassBumpSpeedsRevBlue;
import static frc.robot.commands.AutonomousBuilderConstants.kPassTrenchSpeedsFwdBlue;
import static frc.robot.commands.AutonomousBuilderConstants.kPassTrenchSpeedsRevBlue;
import static frc.robot.commands.AutonomousBuilderConstants.kPassTrenchTimer;
import static frc.robot.commands.AutonomousBuilderConstants.kScoringPoseLeftBumpBlue;
import static frc.robot.commands.AutonomousBuilderConstants.kScoringPoseLeftTrenchBlue;
import static frc.robot.commands.AutonomousBuilderConstants.kScoringPoseRightBumpBlue;
import static frc.robot.commands.AutonomousBuilderConstants.kScoringPoseRightTrenchBlue;
import static frc.robot.commands.AutonomousBuilderConstants.kSimPassBumpTimer;
import static frc.robot.constants.FieldConstants.kRobotPoseLeftBumpBlue;
import static frc.robot.constants.FieldConstants.kRobotPoseLeftTrenchBlue;
import static frc.robot.constants.FieldConstants.kRobotPoseRightBumpBlue;
import static frc.robot.constants.FieldConstants.kRobotPoseRightTrenchBlue;
import static frc.robot.constants.FieldConstants.path;

import com.bobcats.lib.auto.CustomRoutineBuilder;
import com.bobcats.lib.utils.AllianceUtil;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.FileVersionException;
import com.pathplanner.lib.util.FlippingUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.commands.drive.DriveCommands;
import frc.robot.commands.drive.PassBumpCommand;
import frc.robot.constants.FieldConstants;
import frc.robot.subsystems.swerve.SwerveConstants.AutoConstants;
import java.io.IOException;
import java.util.List;
import java.util.Set;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;
import org.json.simple.parser.ParseException;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * A class for building custom autonomous routines based on dashboard selections.
 *
 * <p>
 * This version supports both bump and trench neutral routes, and allows a second neutral cycle
 * as a full autonomous objective.
 */
public class AutonomousBuilder implements CustomRoutineBuilder {

	// Enums //

	/** An enum specifying the autonomous routine type. */
	public enum AutoType {
		kCustom, kPathPlanner;
	}

	/** An enum specifying the starting point of the robot. */
	public enum AutoStartingPoint {
		kMiddle, kRightBump, kLeftBump, kRightTrench, kLeftTrench;
	}

	/** An enum specifying an autonomous routine objective. */
	public enum AutoObjective {
		kNone, kLeave, kScoreDepot, kScoreOutpost, kScoreNeutralLoopOver, kScoreNeutralSweepOver, kScoreNeutralCleanUp;
	}

	/** An enum specifying whether to override the crossing side for neutral scoring. */
	public enum AutoCrossingSideOverride {
		kNone, kForceRight, kForceLeft;
	}

	/** An enum specifying what route to take to cross into neutral. */
	public enum AutoNeutralRoute {
		kBump, kTrench;
	}

	/** The scoring mode, i.e. after what objectives to score. */
	public enum ScoringMode {
		kNone, kObjective1, kObjective2, kObjective1And2;
	}

	// Dashboard choosers
	private LoggedDashboardChooser<AutoType> m_autoTypeChooser;
	private LoggedDashboardChooser<Command> m_ppAutoRoutine;

	private LoggedDashboardChooser<AutoStartingPoint> m_startingPointChooser;
	private LoggedDashboardChooser<AutoObjective> m_objective1Chooser;
	private LoggedDashboardChooser<AutoObjective> m_objective2Chooser;
	private LoggedDashboardChooser<AutoNeutralRoute> m_neutralRouteObj1Chooser;
	private LoggedDashboardChooser<AutoNeutralRoute> m_neutralRouteObj2Chooser;
	private LoggedDashboardChooser<AutoCrossingSideOverride> m_forceNeutralCrossSideOverride;
	private LoggedDashboardChooser<Double> m_adaptiveNeutralTimeBudgetChooser;
	private LoggedDashboardChooser<Double> m_objective1EndCooldown;
	private LoggedDashboardChooser<Boolean> m_parkChooser;
	private LoggedDashboardChooser<Boolean> m_passThroughTowerObj2;
	private LoggedDashboardChooser<Boolean> m_passThroughTowerObj1;
	private LoggedDashboardChooser<ScoringMode> m_scoringModeChooser;
	private LoggedDashboardChooser<Boolean> m_scoreFirst8;

	// Incompatibility warnings
	private Alert m_doubleDepotAlert, m_doubleOutpostAlert, m_redundantCrossOverride, m_redundantTowerCrossObj2,
			m_redundantTowerCrossObj1, m_redundantPathPlannerPath, m_noPathSelected, m_invalidLeaveObjective,
			m_unableToObtainPath, m_objective2before1;

	// Pre-pathed trajectories (bump)
	private PathPlannerPath m_rightLoopOverBump, m_leftLoopOverBump, m_rightSweepOverBump, m_leftSweepOverBump,
			m_leftCleanUpOverBump, m_rightCleanUpOverBump;
	// Pre-pathed trajectories (trench)
	private PathPlannerPath m_rightLoopOverTrench, m_leftLoopOverTrench, m_rightSweepOverTrench, m_leftSweepOverTrench,
			m_leftCleanUpOverTrench, m_rightCleanUpOverTrench;
	// Tower crossing trajectories
	private PathPlannerPath m_towerRL, m_towerLR;

	/** Constructs a new AutonomousSequenceBuilder. */
	public AutonomousBuilder() {
		// Initialize choosers
		m_autoTypeChooser = new LoggedDashboardChooser<>("Autonomous Type");
		m_autoTypeChooser.addDefaultOption("Custom Builder", AutoType.kCustom);
		m_autoTypeChooser.addOption("PathPlanner Preset", AutoType.kPathPlanner);

		m_ppAutoRoutine = new LoggedDashboardChooser<>("Autonomous PathPlanner Routines",
				AutoBuilder.buildAutoChooser());

		m_startingPointChooser = new LoggedDashboardChooser<>("Autonomous Starting Point");
		m_startingPointChooser.addDefaultOption("Middle", AutoStartingPoint.kMiddle);
		m_startingPointChooser.addOption("Right Bump", AutoStartingPoint.kRightBump);
		m_startingPointChooser.addOption("Left Bump", AutoStartingPoint.kLeftBump);
		m_startingPointChooser.addOption("Right Trench", AutoStartingPoint.kRightTrench);
		m_startingPointChooser.addOption("Left Trench", AutoStartingPoint.kLeftTrench);

		m_objective1Chooser = new LoggedDashboardChooser<>("Autonomous Objective #1");
		m_objective1Chooser.addDefaultOption("None", AutoObjective.kNone);
		m_objective1Chooser.addOption("Leave", AutoObjective.kLeave);
		m_objective1Chooser.addOption("Score Depot (Primary)", AutoObjective.kScoreDepot);
		m_objective1Chooser.addOption("Score Outpost (Primary)", AutoObjective.kScoreOutpost);
		m_objective1Chooser.addOption("Score Neutral (Primary - Loop Over)", AutoObjective.kScoreNeutralLoopOver);
		m_objective1Chooser.addOption("Score Neutral (Primary - Sweep Over)", AutoObjective.kScoreNeutralSweepOver);
		m_objective1Chooser.addOption("Score Neutral (Primary - Clean Up)", AutoObjective.kScoreNeutralCleanUp);

		m_objective1EndCooldown = new LoggedDashboardChooser<>("Autonomous Objective #1 to #2 Cooldown");
		m_objective1EndCooldown.addDefaultOption("0 seconds", 0.0);
		m_objective1EndCooldown.addOption("1 seconds", 1.0);
		m_objective1EndCooldown.addOption("2 seconds", 2.0);
		m_objective1EndCooldown.addOption("3 seconds", 3.0);
		m_objective1EndCooldown.addOption("4 seconds", 4.0);
		m_objective1EndCooldown.addOption("5 seconds", 5.0);

		m_objective2Chooser = new LoggedDashboardChooser<>("Autonomous Objective #2");
		m_objective2Chooser.addDefaultOption("None", AutoObjective.kNone);
		m_objective2Chooser.addOption("Score Depot (Secondary)", AutoObjective.kScoreDepot);
		m_objective2Chooser.addOption("Score Outpost (Secondary)", AutoObjective.kScoreOutpost);
		m_objective2Chooser.addOption("Score Neutral (Secondary - Loop Over)", AutoObjective.kScoreNeutralLoopOver);
		m_objective2Chooser.addOption("Score Neutral (Secondary - Sweep Over)", AutoObjective.kScoreNeutralSweepOver);
		m_objective2Chooser.addOption("Score Neutral (Secondary - Clean Up)", AutoObjective.kScoreNeutralCleanUp);

		m_neutralRouteObj1Chooser = new LoggedDashboardChooser<>("Autonomous Neutral Route #1");
		m_neutralRouteObj1Chooser.addDefaultOption("Trench", AutoNeutralRoute.kTrench);
		m_neutralRouteObj1Chooser.addOption("Bump", AutoNeutralRoute.kBump);

		m_neutralRouteObj2Chooser = new LoggedDashboardChooser<>("Autonomous Neutral Route #2");
		m_neutralRouteObj2Chooser.addDefaultOption("Trench", AutoNeutralRoute.kTrench);
		m_neutralRouteObj2Chooser.addOption("Bump", AutoNeutralRoute.kBump);

		m_forceNeutralCrossSideOverride = new LoggedDashboardChooser<>("Autonomous Override Cross Side");
		m_forceNeutralCrossSideOverride.addDefaultOption("No Override", AutoCrossingSideOverride.kNone);
		m_forceNeutralCrossSideOverride.addOption("Force Right", AutoCrossingSideOverride.kForceRight);
		m_forceNeutralCrossSideOverride.addOption("Force Left", AutoCrossingSideOverride.kForceLeft);

		m_adaptiveNeutralTimeBudgetChooser = new LoggedDashboardChooser<>("Autonomous Adaptive Neutral Time Budget");
		m_adaptiveNeutralTimeBudgetChooser.addDefaultOption("5 seconds", 5.0);
		m_adaptiveNeutralTimeBudgetChooser.addOption("4 seconds", 4.0);
		m_adaptiveNeutralTimeBudgetChooser.addOption("3 seconds", 3.0);

		m_parkChooser = new LoggedDashboardChooser<>("Autonomous Parking");
		m_parkChooser.addDefaultOption("No", false);
		m_parkChooser.addOption("Yes", true);

		m_passThroughTowerObj1 = new LoggedDashboardChooser<>("Autonomous Pass Through Tower Objective #1");
		m_passThroughTowerObj1.addDefaultOption("No", false);
		m_passThroughTowerObj1.addOption("Yes", true);

		m_passThroughTowerObj2 = new LoggedDashboardChooser<>("Autonomous Pass Through Tower Objective #2");
		m_passThroughTowerObj2.addDefaultOption("No", false);
		m_passThroughTowerObj2.addOption("Yes", true);

		m_scoringModeChooser = new LoggedDashboardChooser<>("Autonomous Scoring");
		m_scoringModeChooser.addDefaultOption("After Objective #1 and #2", ScoringMode.kObjective1And2);
		m_scoringModeChooser.addOption("After Objective #1", ScoringMode.kObjective1);
		m_scoringModeChooser.addOption("After Objective #2", ScoringMode.kObjective2);
		m_scoringModeChooser.addOption("None", ScoringMode.kNone);

		m_scoreFirst8 = new LoggedDashboardChooser<>("Autonomous Score First 8 Fuel");
		m_scoreFirst8.addDefaultOption("No", false);
		m_scoreFirst8.addOption("Yes", true);

		// Load individual paths
		m_unableToObtainPath = new Alert("Unable to obtain an autonomous builder path! Reboot the robot.",
				AlertType.kError);
		try {
			m_rightLoopOverBump = PathPlannerPath.fromPathFile(path("RightLoopOverEnd"));
			m_leftLoopOverBump = PathPlannerPath.fromPathFile(path("LeftLoopOverEnd"));
			m_rightSweepOverBump = PathPlannerPath.fromPathFile(path("RightSweepOverEnd"));
			m_leftSweepOverBump = PathPlannerPath.fromPathFile(path("LeftSweepOverEnd"));
			m_rightCleanUpOverBump = PathPlannerPath.fromPathFile(path("RightCleanUpOverEnd"));
			m_leftCleanUpOverBump = PathPlannerPath.fromPathFile(path("LeftCleanUpOverEnd"));

			m_rightLoopOverTrench = PathPlannerPath.fromPathFile(path("RightLoopOverEndTrench"));
			m_leftLoopOverTrench = PathPlannerPath.fromPathFile(path("LeftLoopOverEndTrench"));
			m_rightSweepOverTrench = PathPlannerPath.fromPathFile(path("RightSweepOverEndTrench"));
			m_leftSweepOverTrench = PathPlannerPath.fromPathFile(path("LeftSweepOverEndTrench"));
			m_rightCleanUpOverTrench = PathPlannerPath.fromPathFile(path("RightCleanUpOverEndTrench"));
			m_leftCleanUpOverTrench = PathPlannerPath.fromPathFile(path("LeftCleanUpOverEndTrench"));

			m_towerRL = PathPlannerPath.fromPathFile(path("CrossTowerRL"));
			m_towerLR = PathPlannerPath.fromPathFile(path("CrossTowerLR"));
		} catch (FileVersionException | IOException | ParseException e) {
			m_unableToObtainPath.set(true);
			DriverStation.reportWarning("WARNING: AutonomousSequenceBuilder::new, unable to obtain a PathPlanner path",
					false);
			e.printStackTrace();
		}

		// Initialize alerts
		m_doubleDepotAlert = new Alert("Incompatible autonomous! Primary and secondary objectives both set to depot.",
				AlertType.kWarning);
		m_redundantCrossOverride = new Alert(
				"Redundant cross override given, but no neutral scoring objective selected.", AlertType.kWarning);
		m_redundantTowerCrossObj1 = new Alert("Redundant Objective #1 tower cross given with invalid objectives.",
				AlertType.kWarning);
		m_redundantTowerCrossObj2 = new Alert("Redundant Objective #2 tower cross given with invalid objectives.",
				AlertType.kWarning);
		m_redundantPathPlannerPath = new Alert(
				"Redundant PathPlanner path was selected even though the custom builder is being used.",
				AlertType.kWarning);
		m_noPathSelected = new Alert("PathPlanner mode active, but no PathPlanner path selected!", AlertType.kWarning);
		m_invalidLeaveObjective = new Alert(
				"Redundant leave objective is set as primary when a secondary or climb objective is present. Set primary to none instead.",
				AlertType.kWarning);
		m_doubleOutpostAlert = new Alert(
				"Incompatible autonomous! Primary and secondary objectives both set to outpost.", AlertType.kWarning);
		m_objective2before1 = new Alert("Objective #2 set before Objective #1, possible issues during autonomous!",
				AlertType.kWarning);
	}

	/**
	 * Updates the NetworkTable alerts to show possible problems with the autonomous sequence.
	 */
	public void updateNTAlerts() {
		var obj1 = m_objective1Chooser.get();
		var obj2 = m_objective2Chooser.get();
		var autoType = m_autoTypeChooser.get();

		// Both objectives set to Depot
		m_doubleDepotAlert.set(obj1 == AutoObjective.kScoreDepot && obj2 == AutoObjective.kScoreDepot);
		// Both objectives set to Outpost
		m_doubleOutpostAlert.set(obj1 == AutoObjective.kScoreOutpost && obj2 == AutoObjective.kScoreOutpost);

		// A cross override provided with no neutral objective in either cycle
		m_redundantCrossOverride.set(!isNeutralObjective(obj1) && !isNeutralObjective(obj2)
				&& m_forceNeutralCrossSideOverride.get() != AutoCrossingSideOverride.kNone);

		// Cross to go from the depot to the outpost or vice versa
		boolean objective2CrossBetweenDepotOutpost = (obj1 == AutoObjective.kScoreDepot
				&& obj2 == AutoObjective.kScoreOutpost)
				|| (obj1 == AutoObjective.kScoreOutpost && obj2 == AutoObjective.kScoreDepot);

		// For objective #2, tower-cross is relevant only when it is actually used for a cross-field
		// objective. Neutral objective #2 always uses the neutral route helpers, not tower-cross
		// logic.
		boolean objective2CrossFromRightNeutralToDepot = isNeutralObjective(obj1)
				&& (m_objective1Chooser.get() == AutoObjective.kScoreNeutralSweepOver ? !isCrossRightObjective2()
						: isCrossRightObjective2())
				&& obj2 == AutoObjective.kScoreDepot;
		boolean objective2CrossFromLeftNeutralToOutpost = isNeutralObjective(obj1)
				&& (m_objective1Chooser.get() == AutoObjective.kScoreNeutralSweepOver ? isCrossRightObjective2()
						: !isCrossRightObjective2())
				&& obj2 == AutoObjective.kScoreOutpost;

		m_redundantTowerCrossObj2.set(m_passThroughTowerObj2.get() && !(objective2CrossBetweenDepotOutpost
				|| objective2CrossFromRightNeutralToDepot || objective2CrossFromLeftNeutralToOutpost));

		// In the beginning, crossing from the right starting point to the depot (left)
		boolean objective1CrossRightToLeftValid = isRightSideStartingPoint(m_startingPointChooser.get())
				&& (obj1 == AutoObjective.kScoreDepot || (isNeutralObjective(obj1) && !isCrossRightObjective1()));
		// In the beginning, crossing from the left starting point to the outpost (right)
		boolean objective1CrossLeftToRightValid = isLeftSideStartingPoint(m_startingPointChooser.get())
				&& (obj1 == AutoObjective.kScoreOutpost || (isNeutralObjective(obj1) && isCrossRightObjective1()));
		m_redundantTowerCrossObj1.set(
				m_passThroughTowerObj1.get() && !(objective1CrossRightToLeftValid || objective1CrossLeftToRightValid));

		// PathPlanner path selected for the custom builder
		m_redundantPathPlannerPath.set(
				autoType == AutoType.kCustom && !m_ppAutoRoutine.getSendableChooser().getSelected().equals("None"));
		// No path selected for PathPlanner
		m_noPathSelected.set(
				autoType == AutoType.kPathPlanner && m_ppAutoRoutine.getSendableChooser().getSelected().equals("None"));
		// Redundant leave objective, wastes time
		m_invalidLeaveObjective
				.set(obj1 == AutoObjective.kLeave && (obj2 != AutoObjective.kNone || m_scoreFirst8.get()));

		// Objective 2 is set before objective 1 is set
		m_objective2before1.set(
				m_objective2Chooser.get() != AutoObjective.kNone && m_objective1Chooser.get() == AutoObjective.kNone);
	}

	@Override
	public Command getRoutine() {
		var robotContainer = RobotContainer.getInstance();
		var swerve = robotContainer.swerve;

		var obj1 = m_objective1Chooser.get();
		var obj2 = m_objective2Chooser.get();
		var autoType = m_autoTypeChooser.get();

		// Return early if using a PP routine
		if (autoType == AutoType.kPathPlanner) {
			return m_ppAutoRoutine.get().asProxy().withName(m_ppAutoRoutine.get().getName());
		}

		// Resetting the odometry / starting pose
		Command resetOdomCommand = Commands.runOnce(() -> swerve.resetOdometry(getStartingPoseFlipped()));

		// Main objective command
		Command objective1Command = switch (obj1) {
			case kLeave -> Commands.run(() -> swerve.runSpeeds(getFlippedSpeeds(kLeaveSpeedsFwdBlue), true), swerve)
					.withTimeout(kLeaveTimer)
					.andThen(Commands.runOnce(swerve::stop));
			case kScoreDepot -> depotTrajectory(true, m_passThroughTowerObj1::get);
			case kScoreOutpost -> outpostTrajectory(false, m_passThroughTowerObj1::get);
			case kScoreNeutralLoopOver -> buildNeutralCycle(true, m_neutralRouteObj1Chooser.get(),
					isCrossRightObjective1(), neutralLoopPathForObj1(),
					!isLeftSideStartingPoint(m_startingPointChooser.get()), m_passThroughTowerObj1::get);
			case kScoreNeutralSweepOver -> buildNeutralCycle(true, m_neutralRouteObj1Chooser.get(),
					isCrossRightObjective1(), neutralSweepPathForObj1(),
					!isLeftSideStartingPoint(m_startingPointChooser.get()), m_passThroughTowerObj1::get);
			case kScoreNeutralCleanUp -> buildNeutralCycle(true, m_neutralRouteObj1Chooser.get(),
					isCrossRightObjective1(), neutralCleanUpPathForObj1(),
					!isLeftSideStartingPoint(m_startingPointChooser.get()), m_passThroughTowerObj1::get);
			case kNone -> Commands.none();
			default -> Commands.none();
		};

		// Secondary objective command
		Command objective2Command = switch (obj2) {
			case kScoreDepot -> depotTrajectory(true, () -> m_passThroughTowerObj2.get()
					&& (obj1 == AutoObjective.kScoreOutpost || isNeutralObjective(obj1)));
			case kScoreOutpost -> outpostTrajectory(false, () -> m_passThroughTowerObj2.get()
					&& (obj1 == AutoObjective.kScoreDepot || isNeutralObjective(obj1)));
			case kScoreNeutralLoopOver -> buildNeutralCycle(false, m_neutralRouteObj2Chooser.get(),
					isCrossRightObjective2(), neutralLoopPathForObj2(), !isCrossRightObjective2(),
					m_passThroughTowerObj2::get);
			case kScoreNeutralSweepOver -> buildNeutralCycle(false, m_neutralRouteObj2Chooser.get(),
					isCrossRightObjective2(), neutralSweepPathForObj2(), !isCrossRightObjective2(),
					m_passThroughTowerObj2::get);
			case kScoreNeutralCleanUp -> buildNeutralCycle(false, m_neutralRouteObj2Chooser.get(),
					isCrossRightObjective2(), neutralCleanUpPathForObj2(), !isCrossRightObjective2(),
					m_passThroughTowerObj2::get);
			case kNone -> Commands.none();
			default -> Commands.none();
		};

		// Parking command
		Supplier<Command> parking = () -> robotContainer.getAutoManager()
				.getPathfindCommand(path("AutoPark"), AutoConstants.kPathfindSkipTolerance)
				.onlyIf(m_parkChooser::get);

		// First scoring can happen even before objective #1 if the user wants that behavior.
		Command firstScoreIfRequested = Commands.either(
				parking.get()
						.andThen(new DeferredCommand(() -> scoreFuel(99999, getPrescorePose(), true),
								Set.of(RobotContainer.getInstance().swerve)).onlyIf(m_scoreFirst8::get)),
				new DeferredCommand(() -> scoreFuel(k8ScoringTimer, getPrescorePose(), true),
						Set.of(RobotContainer.getInstance().swerve)).onlyIf(m_scoreFirst8::get),
				() -> m_objective1Chooser.get() == AutoObjective.kNone
						&& m_objective2Chooser.get() == AutoObjective.kNone && m_parkChooser.get());

		return resetOdomCommand.andThen(Commands.waitSeconds(kAutoBufferTime))
				.andThen(firstScoreIfRequested)
				.andThen(objective1Command)
				.andThen(scoreAfterObjective(
						m_objective2Chooser.get() != AutoObjective.kNone ? getObjective1ScoreTime() : 99999, true)
								.onlyIf(() -> m_objective1Chooser.get() != AutoObjective.kNone))
				.andThen(Commands.waitSeconds(m_objective1EndCooldown.get()))
				.andThen(objective2Command)
				.andThen(parking.get()
						.onlyIf(() -> (m_objective1Chooser.get() != AutoObjective.kNone
								|| m_objective2Chooser.get() != AutoObjective.kNone) && m_parkChooser.get()))
				.andThen(scoreAfterObjective(99999, false)
						.onlyIf(() -> m_objective2Chooser.get() != AutoObjective.kNone))
				.withName("Custom Auto Routine (Starting: " + m_startingPointChooser.get() + " / Objective #1: " + obj1
						+ " / Objective #2: " + obj2 + " / Score: " + m_scoringModeChooser.get() + " / Route #1: "
						+ m_neutralRouteObj1Chooser.get() + " / Route #2: " + m_neutralRouteObj2Chooser.get()
						+ " / Pass Tower Objective #1: " + m_passThroughTowerObj1.get() + " / Pass Tower Objective #2: "
						+ m_passThroughTowerObj2.get() + " / Park: " + m_parkChooser.get() + ")");
	}

	/** Returns the appropriately flipped starting pose. */
	private Pose2d getStartingPoseFlipped() {
		return AllianceUtil.flipWithAlliance(switch (m_startingPointChooser.get()) {
			case kMiddle -> FieldConstants.kRobotPoseMiddleBlue;
			case kRightBump -> FieldConstants.kRobotPoseRightBumpBlue;
			case kLeftBump -> FieldConstants.kRobotPoseLeftBumpBlue;
			case kRightTrench -> FieldConstants.kRobotPoseRightTrenchBlue;
			case kLeftTrench -> FieldConstants.kRobotPoseLeftTrenchBlue;
			default -> FieldConstants.kRobotPoseMiddleBlue;
		});
	}

	/** Returns the appropriately flipped chassis speeds. */
	private ChassisSpeeds getFlippedSpeeds(ChassisSpeeds speeds) {
		return AllianceUtil.isRedAlliance() ? FlippingUtil.flipFieldSpeeds(speeds) : speeds;
	}

	/**
	 * Returns whether the right side should be used if going to the neutral zone for Objective #1.
	 */
	private boolean isCrossRightObjective1() {
		if (m_forceNeutralCrossSideOverride.get() != AutoCrossingSideOverride.kNone) {
			return m_forceNeutralCrossSideOverride.get() == AutoCrossingSideOverride.kForceRight;
		}

		return !isLeftSideStartingPoint(m_startingPointChooser.get());
	}

	/** Returns the command to pass the bump. */
	private Command passBump(boolean isToNeutral) {
		if (!Robot.isSimulation()) {
			return new PassBumpCommand(RobotContainer.getInstance().swerve,
					isToNeutral ? kPassBumpSpeedsFwdBlue : kPassBumpSpeedsRevBlue);
		}

		return Commands
				.run(() -> RobotContainer.getInstance().swerve
						.runSpeeds(isToNeutral ? getFlippedSpeeds(new ChassisSpeeds(kPassBumpSpeedsFwdBlue, 0, 0))
								: getFlippedSpeeds(new ChassisSpeeds(kPassBumpSpeedsRevBlue, 0, 0)), true))
				.withTimeout(kSimPassBumpTimer);
	}

	/** Returns the command to cross the trench. */
	private Command crossTrench(boolean isToNeutral) {
		if (!Robot.isSimulation()) {
			return Commands
					.run(() -> RobotContainer.getInstance().swerve
							.runSpeeds(
									isToNeutral ? getFlippedSpeeds(new ChassisSpeeds(kPassTrenchSpeedsFwdBlue, 0, 0))
											: getFlippedSpeeds(new ChassisSpeeds(kPassTrenchSpeedsRevBlue, 0, 0)),
									true))
					.withTimeout(kPassTrenchTimer);
		}

		return Commands
				.run(() -> RobotContainer.getInstance().swerve
						.runSpeeds(isToNeutral ? getFlippedSpeeds(new ChassisSpeeds(kPassTrenchSpeedsFwdBlue, 0, 0))
								: getFlippedSpeeds(new ChassisSpeeds(kPassTrenchSpeedsRevBlue, 0, 0)), true))
				.withTimeout(kPassTrenchTimer);
	}

	/** Crosses to the neutral zone using the selected route. */
	private Command crossNeutralBarrier(AutoNeutralRoute route, boolean isToNeutral) {
		return route == AutoNeutralRoute.kTrench ? crossTrench(isToNeutral) : passBump(isToNeutral);
	}

	/** Returns the pose used to align for crossing into neutral for the selected route. */
	private Pose2d alignNeutralBarrierPose(AutoNeutralRoute route, boolean isRight) {
		return route == AutoNeutralRoute.kTrench ? (isRight ? kRobotPoseRightTrenchBlue : kRobotPoseLeftTrenchBlue)
				: (isRight ? kRobotPoseRightBumpBlue : kRobotPoseLeftBumpBlue);
	}

	/** Returns the command to intake from the depot. */
	private Command depotTrajectory(boolean crossRightToLeft, BooleanSupplier towerCrossCondition) {
		return crossTower(crossRightToLeft).onlyIf(towerCrossCondition)
				.andThen(RobotContainer.getInstance().superstructure.startIntaking())
				.andThen(Commands.either(
						RobotContainer.getInstance()
								.getAutoManager()
								.getPathfindCommand(path("TowerThenDepot"), AutoConstants.kPathfindSkipTolerance),
						RobotContainer.getInstance()
								.getAutoManager()
								.getPathfindCommand(path("Depot"), AutoConstants.kPathfindSkipTolerance),
						() -> crossRightToLeft && towerCrossCondition.getAsBoolean()))
				.andThen(Commands.waitSeconds(kDepotIdleCooldown))
				.andThen(RobotContainer.getInstance().superstructure.stopIntake());
	}

	/** Returns the command to intake from the outpost. */
	private Command outpostTrajectory(boolean crossRightToLeft, BooleanSupplier towerCrossCondition) {
		return crossTower(crossRightToLeft).onlyIf(towerCrossCondition)
				.andThen(RobotContainer.getInstance()
						.getAutoManager()
						.getPathfindCommand(path("Outpost"), AutoConstants.kPathfindSkipTolerance))
				.andThen(Commands.waitSeconds(kOutpostIdleCooldown));
	}

	/** Returns the command to pass through the tower. */
	private Command crossTower(boolean isRightToLeft) {
		return pathfindThenFollow(isRightToLeft ? m_towerRL : m_towerLR);
	}

	/** Returns the command to follow a path. */
	private Command pathfindThenFollow(PathPlannerPath path) {
		return Commands.either(
				// Just try to follow if negligibly close to the starting point
				AutoBuilder.followPath(path),
				// Pathfind first, then follow if too far away
				AutoBuilder.pathfindThenFollowPath(path, AutoConstants.kPathConstraints),
				() -> RobotContainer.getInstance().swerve.getFilteredPose()
						.getTranslation()
						.getDistance(path.getStartingHolonomicPose()
								.orElse(Pose2d.kZero)
								.getTranslation()) <= AutoConstants.kPathfindSkipTolerance);
	}

	/** Pathfinds to the given pose if far enough. */
	private Command pathfindToPose(Pose2d pose) {
		return Commands.either(
				// Do nothing if close enough
				Commands.none(),
				// Pathfind if far
				AutoBuilder.pathfindToPose(pose, AutoConstants.kPathConstraints),
				() -> RobotContainer.getInstance().swerve.getFilteredPose()
						.getTranslation()
						.getDistance(pose.getTranslation()) <= AutoConstants.kPathfindSkipTolerance);
	}

	/** Score after an objective has ended. */
	private Command scoreAfterObjective(double scoreTime, boolean isObjective1) {
		return new DeferredCommand(() -> scoreFuel(scoreTime, getOptimalScoringPose(isObjective1), false), Set.of())
				.onlyIf(() -> m_scoringModeChooser.get() == ScoringMode.kObjective1And2
						|| (isObjective1 && m_scoringModeChooser.get() == ScoringMode.kObjective1)
						|| (!isObjective1 && m_scoringModeChooser.get() == ScoringMode.kObjective2));
	}

	/** Returns the optimal pose to score the first 8 fuel. */
	private Pose2d getPrescorePose() {
		boolean isTrench = isPrimaryNeutralObjective() && m_neutralRouteObj1Chooser.get() == AutoNeutralRoute.kTrench;
		Pose2d right = AllianceUtil
				.flipWithAlliance(isTrench ? kScoringPoseRightTrenchBlue : kScoringPoseRightBumpBlue);
		Pose2d left = AllianceUtil.flipWithAlliance(isTrench ? kScoringPoseLeftTrenchBlue : kScoringPoseLeftBumpBlue);

		if (isPrimaryNeutralObjective()) { return isCrossRightObjective1() ? right : left; }

		if (m_objective1Chooser.get() == AutoObjective.kNone && m_parkChooser.get()) { return null; }
		if (m_objective1Chooser.get() == AutoObjective.kScoreDepot && !m_passThroughTowerObj1.get()) {
			return left;
		} else if (m_objective1Chooser.get() == AutoObjective.kScoreDepot && m_passThroughTowerObj1.get()) {
			return right;
		}
		if (m_objective1Chooser.get() == AutoObjective.kScoreOutpost && !m_passThroughTowerObj1.get()) {
			return right;
		} else if (m_objective1Chooser.get() == AutoObjective.kScoreOutpost && m_passThroughTowerObj1.get()) {
			return left;
		}

		return getNearestScoringPose(isTrench);
	}

	/** Returns the command to start scoring. */
	private Command scoreFuel(double scoreTime, Pose2d scoringPose, boolean isFirst8) {
		return Commands.runOnce(RobotContainer.getInstance()::initAutonScoreFuelCommand)
				.alongWith(Commands.runOnce(() -> RobotContainer.getInstance().superstructure.autoHaltShots = true))
				.andThen(new DeferredCommand(
						() -> AutoBuilder.pathfindToPose(scoringPose, AutoConstants.kPathConstraints), Set.of())
								.onlyIf(() -> scoringPose != null))
				.andThen(Commands.runOnce(() -> RobotContainer.getInstance().superstructure.autoHaltShots = false))
				// Rotate to setpoint
				.andThen(Commands
						.run(() -> RobotContainer.getInstance().swerve.runSpeeds(0, 0,
								DriveCommands.LatestShootingAngularVelocity, true))
						.until(() -> RobotContainer.getInstance().swerve.ArbitraryPIDAngular.atSetpoint()
								&& RobotContainer.getInstance().rollers.isNearSetpoint()))
				.andThen(Commands.print("Reached setpoint"))
				// Wait and make sure to stay at the setpoint
				.andThen(Commands.race(Commands.waitSeconds(scoreTime),
						Commands.run(() -> RobotContainer.getInstance().swerve.runSpeeds(0, 0,
								DriveCommands.LatestShootingAngularVelocity, true))))
				.andThen(Commands.print("Seconds waited: " + scoreTime))
				.finallyDo(() -> {
					DriveCommands.LatestShootingAngularVelocity = 0;
					RobotContainer.getInstance().cancelAutonScoreFuelCommand();
					RobotContainer.getInstance().superstructure.setObjectiveOriented(true);
				});
	}

	/** Returns the closest scoring pose for the hub. */
	private Pose2d getNearestScoringPose(boolean isTrench) {
		return RobotContainer.getInstance().swerve.getFilteredPose()
				.nearest(isTrench
						? List.of(AllianceUtil.flipWithAlliance(kScoringPoseRightTrenchBlue),
								AllianceUtil.flipWithAlliance(kScoringPoseLeftTrenchBlue))
						: List.of(AllianceUtil.flipWithAlliance(kScoringPoseRightBumpBlue),
								AllianceUtil.flipWithAlliance(kScoringPoseLeftBumpBlue)));
	}

	/** Returns the optimal objective scoring pose for the hub. */
	private Pose2d getOptimalScoringPose(boolean isObjective1) {
		boolean isTrench = isObjective1
				? isNeutralObjective(m_objective1Chooser.get())
						&& m_neutralRouteObj1Chooser.get() == AutoNeutralRoute.kTrench
				: isNeutralObjective(m_objective2Chooser.get())
						&& m_neutralRouteObj2Chooser.get() == AutoNeutralRoute.kTrench;

		return getNearestScoringPose(isTrench);
	}

	/** Returns the scoring time for objective 1. */
	private double getObjective1ScoreTime() {
		if (getObjective1Fuel() == 24) { return k24ScoringTimer; }
		if (getObjective1Fuel() == 60) { return k60ScoringTimer; }
		return k8ScoringTimer;
	}

	/** Returns the approximate fuel scored for objective 1. */
	private double getObjective1Fuel() {
		if (m_objective1Chooser.get() == AutoObjective.kScoreDepot
				|| m_objective1Chooser.get() == AutoObjective.kScoreOutpost) {
			return 24;
		}
		if (isPrimaryNeutralObjective()) { return 60; }

		return 0;
	}

	/** Returns whether the given objective is in the neutral zone. */
	private boolean isNeutralObjective(AutoObjective objective) {
		return objective == AutoObjective.kScoreNeutralCleanUp || objective == AutoObjective.kScoreNeutralLoopOver
				|| objective == AutoObjective.kScoreNeutralSweepOver;
	}

	/** Returns whether the routine's starting point is on the right. */
	private boolean isRightSideStartingPoint(AutoStartingPoint startingPoint) {
		return startingPoint == AutoStartingPoint.kRightBump || startingPoint == AutoStartingPoint.kRightTrench;
	}

	/** Returns whether the routine's starting point is on the left. */
	private boolean isLeftSideStartingPoint(AutoStartingPoint startingPoint) {
		return startingPoint == AutoStartingPoint.kLeftBump || startingPoint == AutoStartingPoint.kLeftTrench;
	}

	/** Returns the path for the loop-over neutral route for objective #1. */
	private PathPlannerPath neutralLoopPathForObj1() {
		return isCrossRightObjective1() ? rightLoopOverTrenchOrBump(true) : leftLoopOverTrenchOrBump(true);
	}

	/** Returns the path for the sweep-over neutral route for objective #1. */
	private PathPlannerPath neutralSweepPathForObj1() {
		return isCrossRightObjective1() ? rightSweepOverTrenchOrBump(true) : leftSweepOverTrenchOrBump(true);
	}

	/** Returns the path for the sweep-over neutral route for objective #1. */
	private PathPlannerPath neutralCleanUpPathForObj1() {
		return isCrossRightObjective1() ? rightCleanUpOverTrenchOrBump(true) : leftCleanUpOverTrenchOrBump(true);
	}

	/** Returns the path for the loop-over neutral route for objective #2. */
	private PathPlannerPath neutralLoopPathForObj2() {
		return isCrossRightObjective2() ? rightLoopOverTrenchOrBump(false) : leftLoopOverTrenchOrBump(false);
	}

	/** Returns the path for the sweep-over neutral route for objective #2. */
	private PathPlannerPath neutralSweepPathForObj2() {
		return isCrossRightObjective2() ? rightSweepOverTrenchOrBump(false) : leftSweepOverTrenchOrBump(false);
	}

	/** Returns the path for the clean-up neutral route for objective #1. */
	private PathPlannerPath neutralCleanUpPathForObj2() {
		return isCrossRightObjective2() ? rightCleanUpOverTrenchOrBump(false) : leftCleanUpOverTrenchOrBump(false);
	}

	/**
	 * Returns the bump/trench neutral loop path for the right side.
	 */
	private PathPlannerPath rightLoopOverTrenchOrBump(boolean isObj1) {
		return (isObj1 ? m_neutralRouteObj1Chooser.get() : m_neutralRouteObj2Chooser.get()) == AutoNeutralRoute.kTrench
				? m_rightLoopOverTrench : m_rightLoopOverBump;
	}

	/**
	 * Returns the bump/trench neutral loop path for the left side.
	 */
	private PathPlannerPath leftLoopOverTrenchOrBump(boolean isObj1) {
		return (isObj1 ? m_neutralRouteObj1Chooser.get() : m_neutralRouteObj2Chooser.get()) == AutoNeutralRoute.kTrench
				? m_leftLoopOverTrench : m_leftLoopOverBump;
	}

	/**
	 * Returns the bump/trench neutral sweep path for the right side.
	 */
	private PathPlannerPath rightSweepOverTrenchOrBump(boolean isObj1) {
		return (isObj1 ? m_neutralRouteObj1Chooser.get() : m_neutralRouteObj2Chooser.get()) == AutoNeutralRoute.kTrench
				? m_rightSweepOverTrench : m_rightSweepOverBump;
	}

	/**
	 * Returns the bump/trench neutral clean-up path for the right side.
	 */
	private PathPlannerPath rightCleanUpOverTrenchOrBump(boolean isObj1) {
		return (isObj1 ? m_neutralRouteObj1Chooser.get() : m_neutralRouteObj2Chooser.get()) == AutoNeutralRoute.kTrench
				? m_rightCleanUpOverTrench : m_rightCleanUpOverBump;
	}

	/**
	 * Returns the bump/trench neutral sweep path for the left side.
	 */
	private PathPlannerPath leftSweepOverTrenchOrBump(boolean isObj1) {
		return (isObj1 ? m_neutralRouteObj1Chooser.get() : m_neutralRouteObj2Chooser.get()) == AutoNeutralRoute.kTrench
				? m_leftSweepOverTrench : m_leftSweepOverBump;
	}

	/**
	 * Returns the bump/trench neutral clean-up path for the left side.
	 */
	private PathPlannerPath leftCleanUpOverTrenchOrBump(boolean isObj1) {
		return (isObj1 ? m_neutralRouteObj1Chooser.get() : m_neutralRouteObj2Chooser.get()) == AutoNeutralRoute.kTrench
				? m_leftCleanUpOverTrench : m_leftCleanUpOverBump;
	}

	/**
	 * Builds a neutral cycle for either objective #1 or objective #2.
	 */
	private Command buildNeutralCycle(boolean isObjective1, AutoNeutralRoute route, boolean alignRight,
			PathPlannerPath selectedPath, boolean crossTowerRightToleft, BooleanSupplier towerCrossCondition) {

		// TODO ???
		// boolean shouldCrossTower = useObjectiveOneTowerCrossCondition
		// ? (isObjective1 ? (isRightSideStartingPoint(m_startingPointChooser.get()) && !alignRight)
		// : towerCrossCondition.getAsBoolean())
		// : towerCrossCondition.getAsBoolean();
		boolean shouldCrossTower = towerCrossCondition.getAsBoolean();

		return crossTower(crossTowerRightToleft).onlyIf(() -> shouldCrossTower)
				.andThen(pathfindToPose(alignNeutralBarrierPose(route, alignRight)))
				.andThen(Commands.runOnce(() -> RobotContainer.getInstance().cancelAutonScoreFuelCommand()))
				.andThen(crossNeutralBarrier(route, true))
				.andThen(pathfindThenFollow(selectedPath))
				.andThen(Commands.runOnce(() -> RobotContainer.getInstance().superstructure.stopIntake()))
				.andThen(crossNeutralBarrier(route, false))
				.andThen(() -> RobotContainer.getInstance().swerve.stop());
	}

	/** Returns whether objective #1 is a neutral objective. */
	private boolean isPrimaryNeutralObjective() {
		return isNeutralObjective(m_objective1Chooser.get());
	}

	/** Whether to cross the right side for objective #2. */
	private boolean isCrossRightObjective2() {
		if (!isNeutralObjective(m_objective2Chooser.get())) return false;
		if (m_forceNeutralCrossSideOverride.get() != AutoCrossingSideOverride.kNone)
			return m_forceNeutralCrossSideOverride.get() == AutoCrossingSideOverride.kForceRight;

		if (isPrimaryNeutralObjective() && m_objective1Chooser.get() != AutoObjective.kScoreNeutralSweepOver) {
			return isCrossRightObjective1();
		} else if (isPrimaryNeutralObjective() && m_objective1Chooser.get() == AutoObjective.kScoreNeutralSweepOver) {
			return !isCrossRightObjective1();
		} else if (m_objective1Chooser.get() == AutoObjective.kScoreDepot) {
			return false;
		} else if (m_objective1Chooser.get() == AutoObjective.kScoreOutpost) {
			return true;
		} else {
			return false;
		}
	}
}
