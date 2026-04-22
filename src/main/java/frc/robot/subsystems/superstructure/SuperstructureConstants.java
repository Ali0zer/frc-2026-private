package frc.robot.subsystems.superstructure;

import com.bobcats.lib.container.Vector3;
import com.bobcats.lib.control.shooter.ShooterCalculator;
import com.bobcats.lib.control.shooter.ShooterCalculator.ShooterParameters;
import com.bobcats.lib.control.shooter.data.ShooterDescriptor;
import com.bobcats.lib.control.shooter.data.ShooterProjectileType;
import com.bobcats.lib.math.BilinearInterpolator2D;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.util.Color;

/** The class storing the constants required for the Superstructure. */
public class SuperstructureConstants {
	// Chassis state constants
	public static final double kSwerveIdleVelocityThreshold = 0.3;
	public static final double kChassisStateSwitchDebounce = 0.2;

	public static final double kShooterStateStaleTimeThreshold = 0.2;

	// Simulation related data
	public static final int kFuelCapacity = 60;
	public static final double kIntakeArmMaxHorizontalExtension = 0.2;

	// Corral alignment
	public static final String kCorralAlignPath = "CorralAlign";
	public static final double kCorralOuttakeTimerLimit = 2.0;

	// Dashboard status indicators
	public static final Color kShooterReadyFlashColor = new Color(208, 0, 245);
	public static final Color kShooterNotReadyFlashColor = new Color(245, 57, 0);
	public static final Color kShooterFlashOffColor = new Color();

	public static final Color kPrefireFlashColor = new Color(208, 0, 245);
	public static final Color kPrefireFlashColorSecondary = new Color(0, 181, 30);
	public static final Color kPrefireFlashOffColor = new Color();

	public static final double kIntakeRollerVelocityThreshold = 5;
	public static final Color kIntakeFlashRollingColor = new Color(0, 181, 30);
	public static final Color kIntakeFlashOffColor = new Color();
	public static final double kFlashPeriod = 0.1; // seconds

	// Shot calculator data
	// Soft-limit of the maximum allowed shot distance
	public static final double kMaxAllowedShotDistance = 8.0; // 5.63;
	public static final double kMinAllowedShotDistance = 1.17;

	public static final double kRollerRadiusMeters = Units.inchesToMeters(2);
	public static final double kBarrelLengthMeters = 0;
	public static final double kAverageShotTime = 1.0 / 8.0;

	public static final double kIntakeOscillateStartDelay = 1.0;
	public static final double kMaxOscillationTime = 3.0;

	// Shooting chassis yaw error tolerance for angular PID
	public static final double kShotYawMaxError = Math.toRadians(3.5);

	// TODO
	public static final double kStaticShotAngle = 56;
	public static final Transform3d kHoodCentralPivotRobotRelative = new Transform3d(0.122173567, 0.00185111465,
			0.536099189, Rotation3d.kZero);
	public static final double kHoodCalibrationYaw = 180;

	// About a step size of 1.6in
	public static final int kPassTrajOptSteps = 200;

	public static final ShooterParameters kPresetShootingParametersBlue = new ShooterParameters(true,
			Rotation2d.fromDegrees(80.5), Rotation2d.fromDegrees(180 + 80.5), 65, 2300, 0, Translation3d.kZero,
			Vector3.kZero, 0);

	public static final int kPreloadedFuelAmount = 8;

	// Note: make sure to pick high horizontal velocities in order to minimize time
	// and allow for shooting on the move while going backwards

	public static final double[] kDistanceMap = new double[] { 1.17, 2.14, 3.10, 3.84, 5.12, 5.63, 7.0, 8.0 };
	public static final double[] kHeightDiffMap = new double[] { 3, 3 };
	public static final double[][] kAngleMap = ShooterCalculator.createSingleHeightParameterMap(new double[8]);
	public static final double[][] kRPMMap = ShooterCalculator.createSingleHeightParameterMap(
			new double[] { 185.0 * 30.0 / Math.PI, 180.0 * 30.0 / Math.PI, 220.0 * 30.0 / Math.PI,
					245.0 * 30.0 / Math.PI, 265.0 * 30.0 / Math.PI, 270.0 * 30.0 / Math.PI, 2700.0, 2770.0 });

	public static final double[] kDistanceMapTOF = new double[] { 1.38, 1.88, 3.15, 4.55, 5.68 };
	public static final double[][] kTimeOfFlightMap = ShooterCalculator
			.createSingleHeightParameterMap(new double[] { 1.1, 1.15, 0.7, 1.25, 1.29 });

	// Extra flight time fudge factors to allow shooting earlier to time more accurately
	public static final InterpolatingDoubleTreeMap kShootingFlightTimeFudgeFactors = new InterpolatingDoubleTreeMap();
	public static final double kPrefireFlashTimeBuffer = 2.5;

	static {
		kShootingFlightTimeFudgeFactors.put(0.5, 0.10);
		kShootingFlightTimeFudgeFactors.put(1.0, 0.20);
		kShootingFlightTimeFudgeFactors.put(1.5, 0.30);
		kShootingFlightTimeFudgeFactors.put(2.0, 0.35);
		kShootingFlightTimeFudgeFactors.put(3.0, 0.40);
		kShootingFlightTimeFudgeFactors.put(4.5, 0.45);
	}

	// Shot phase delay
	public static final double kShotDelay = 0;

	// Bilinear interpolators
	public static final BilinearInterpolator2D kAngleInterpolator = new BilinearInterpolator2D(kDistanceMap,
			kHeightDiffMap, kAngleMap);
	public static final BilinearInterpolator2D kRPMInterpolator = new BilinearInterpolator2D(kDistanceMap,
			kHeightDiffMap, kRPMMap);
	public static final BilinearInterpolator2D kTimeOfFlightInterpolator = new BilinearInterpolator2D(kDistanceMapTOF,
			kHeightDiffMap, kTimeOfFlightMap);

	public static final ShooterDescriptor kShooterDescriptor = new ShooterDescriptor.Builder(kRollerRadiusMeters,
			kHoodCentralPivotRobotRelative).barrelLength(kBarrelLengthMeters)
					.dataSet(kAngleInterpolator, kRPMInterpolator, kTimeOfFlightInterpolator)
					.setDistanceLimits(kMinAllowedShotDistance, kMaxAllowedShotDistance)
					.shotDelay(kShotDelay)
					.build();

	// Shooting velocity limits
	public static final boolean kLimitVelocityWhenShootingTeleop = true;
	public static final double kLimitK0Hub = 0.90;
	public static final double kLimitK1Hub = 0.60;

	public static final double kLimitK0Pass = 0.86;
	public static final double kLimitK1Pass = 0.60;

	// Hub shooting arc
	public static final double kHubShootRadius = 2.65;
	public static final double kHubRadialKp = 5.0;
	public static final double kHubVelocityAmpl = 1.5;
	public static final double kArcPointAllowedDistanceError = 0.06;

	// TODO Obtain e coefficient via calibration
	public static final ShooterProjectileType kFuelProjectile = new ShooterProjectileType(0.000485513987, 0.0050949066,
			0.215456376, 0.7);
}
