package frc.robot.subsystems.shooter.rollers.io;

import static frc.robot.subsystems.shooter.rollers.RollerConstants.kD;
import static frc.robot.subsystems.shooter.rollers.RollerConstants.kFollower1OpposesMain;
import static frc.robot.subsystems.shooter.rollers.RollerConstants.kFollower2OpposesMain;
import static frc.robot.subsystems.shooter.rollers.RollerConstants.kFollower3OpposesMain;
import static frc.robot.subsystems.shooter.rollers.RollerConstants.kFollowerMotor1ID;
import static frc.robot.subsystems.shooter.rollers.RollerConstants.kFollowerMotor2ID;
import static frc.robot.subsystems.shooter.rollers.RollerConstants.kFollowerMotor3ID;
import static frc.robot.subsystems.shooter.rollers.RollerConstants.kGearboxReduction;
import static frc.robot.subsystems.shooter.rollers.RollerConstants.kIsFOC;
import static frc.robot.subsystems.shooter.rollers.RollerConstants.kMainMotorID;
import static frc.robot.subsystems.shooter.rollers.RollerConstants.kMotorCANBus;
import static frc.robot.subsystems.shooter.rollers.RollerConstants.kMotorInverted;
import static frc.robot.subsystems.shooter.rollers.RollerConstants.kMotorStatorLimitAmps;
import static frc.robot.subsystems.shooter.rollers.RollerConstants.kMotorSupplyLimitAmps;
import static frc.robot.subsystems.shooter.rollers.RollerConstants.kP;
import static frc.robot.subsystems.shooter.rollers.RollerConstants.kS;
import static frc.robot.subsystems.shooter.rollers.RollerConstants.kV;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonUtils;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;

/**
 * The IO hardware implementation for roller hardware interacions with the TalonFX.
 */
public class RollerIOKraken implements RollerIO {
	private TalonFX m_mainTalon;
	private TalonFX m_followerTalon1;
	private TalonFX m_followerTalon2;
	private TalonFX m_followerTalon3;

	private Debouncer m_mainMotorConnectedDebouncer = new Debouncer(0.5, DebounceType.kFalling);
	private Debouncer m_follower1MotorConnectedDebouncer = new Debouncer(0.5, DebounceType.kFalling);
	private Debouncer m_follower2MotorConnectedDebouncer = new Debouncer(0.5, DebounceType.kFalling);
	private Debouncer m_follower3MotorConnectedDebouncer = new Debouncer(0.5, DebounceType.kFalling);

	// Control objects
	private VelocityVoltage m_rpmOut = new VelocityVoltage(0).withEnableFOC(kIsFOC).withSlot(0);
	private VoltageOut m_voltsOut = new VoltageOut(0).withEnableFOC(kIsFOC);

	private StatusSignal<Voltage> m_voltsSignal;
	private StatusSignal<AngularVelocity> m_rpmSignal;
	private StatusSignal<Angle> m_positionSignal;
	private StatusSignal<Current> m_supplySignal, m_statorSignal;
	private StatusSignal<Temperature> m_tempSignal;

	private StatusSignal<Current> m_supplySignalFollower1, m_supplySignalFollower2, m_supplySignalFollower3;
	private StatusSignal<Temperature> m_tempSignalFollower1, m_tempSignalFollower2, m_tempSignalFollower3;

	/**
	 * Constructs a new RollerIOKraken.
	 */
	public RollerIOKraken() {
		m_mainTalon = new TalonFX(kMainMotorID, new CANBus(kMotorCANBus));
		m_followerTalon1 = new TalonFX(kFollowerMotor1ID, new CANBus(kMotorCANBus));
		m_followerTalon1 = new TalonFX(kFollowerMotor2ID, new CANBus(kMotorCANBus));
		m_followerTalon1 = new TalonFX(kFollowerMotor3ID, new CANBus(kMotorCANBus));

		// Configure motors
		TalonFXConfiguration motorConfig = new TalonFXConfiguration();
		motorConfig.CurrentLimits.StatorCurrentLimit = kMotorStatorLimitAmps;
		motorConfig.CurrentLimits.SupplyCurrentLimit = kMotorSupplyLimitAmps;
		motorConfig.MotorOutput.DutyCycleNeutralDeadband = 0.03;
		motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
		motorConfig.MotorOutput.Inverted = kMotorInverted ? InvertedValue.Clockwise_Positive
				: InvertedValue.CounterClockwise_Positive;
		motorConfig.Feedback.SensorToMechanismRatio = kGearboxReduction;
		// Convert relevant PID units from RPM to RPS-frame while maintaining output
		motorConfig.Slot0.kP = kP * 60.0;
		motorConfig.Slot0.kD = kD * 60.0;
		motorConfig.Slot0.kS = kS;
		motorConfig.Slot0.kV = kV * 60.0;

		TalonUtils.retryUntilOk(() -> m_mainTalon.getConfigurator().apply(motorConfig), 3,
				"Applying configuration to main roller motor");

		TalonUtils.retryUntilOk(() -> m_followerTalon1.getConfigurator().apply(motorConfig), 3,
				"Applying configuration to follower roller motor #1");

		TalonUtils.retryUntilOk(() -> m_followerTalon1.getConfigurator().apply(motorConfig), 3,
				"Applying configuration to follower roller motor #2");

		TalonUtils.retryUntilOk(() -> m_followerTalon1.getConfigurator().apply(motorConfig), 3,
				"Applying configuration to follower roller motor #3");

		m_followerTalon1.setControl(new Follower(kMainMotorID,
				kFollower1OpposesMain ? MotorAlignmentValue.Opposed : MotorAlignmentValue.Aligned));
		m_followerTalon2.setControl(new Follower(kMainMotorID,
				kFollower2OpposesMain ? MotorAlignmentValue.Opposed : MotorAlignmentValue.Aligned));
		m_followerTalon3.setControl(new Follower(kMainMotorID,
				kFollower3OpposesMain ? MotorAlignmentValue.Opposed : MotorAlignmentValue.Aligned));

		// Status signals
		m_voltsSignal = m_mainTalon.getMotorVoltage();
		m_rpmSignal = m_mainTalon.getVelocity();
		m_positionSignal = m_mainTalon.getPosition();
		m_supplySignal = m_mainTalon.getSupplyCurrent();
		m_statorSignal = m_mainTalon.getStatorCurrent();
		m_tempSignal = m_mainTalon.getDeviceTemp();

		m_supplySignalFollower1 = m_followerTalon1.getSupplyCurrent();
		m_tempSignalFollower1 = m_followerTalon1.getDeviceTemp();
		m_supplySignalFollower2 = m_followerTalon2.getSupplyCurrent();
		m_tempSignalFollower2 = m_followerTalon2.getDeviceTemp();
		m_supplySignalFollower3 = m_followerTalon3.getSupplyCurrent();
		m_tempSignalFollower3 = m_followerTalon3.getDeviceTemp();

		BaseStatusSignal.setUpdateFrequencyForAll(100, m_voltsSignal, m_rpmSignal, m_positionSignal, m_supplySignal,
				m_statorSignal, m_tempSignal, m_supplySignalFollower1, m_supplySignalFollower2, m_supplySignalFollower3,
				m_tempSignalFollower1, m_tempSignalFollower2, m_tempSignalFollower3);

		ParentDevice.optimizeBusUtilizationForAll(m_mainTalon, m_followerTalon1, m_followerTalon2, m_followerTalon3);
	}

	@Override
	public void updateInputs(RollerIOInputs inputs) {
		inputs.mainMotorConnected = m_mainMotorConnectedDebouncer.calculate(
				BaseStatusSignal.refreshAll(m_voltsSignal, m_rpmSignal, m_supplySignal, m_statorSignal, m_tempSignal)
						.isOK());
		inputs.appliedVoltageMain = m_voltsSignal.getValueAsDouble();
		inputs.rpmMain = m_rpmSignal.getValueAsDouble() * 60.0;
		inputs.positionRevsMain = m_positionSignal.getValueAsDouble();
		inputs.supplyCurrentAmpsMain = Math.abs(m_supplySignal.getValueAsDouble());
		inputs.statorCurrentAmpsMain = Math.abs(m_statorSignal.getValueAsDouble());
		inputs.temperatureCelsiusMain = m_tempSignal.getValueAsDouble();

		inputs.follower1MotorConnected = m_follower1MotorConnectedDebouncer
				.calculate(BaseStatusSignal.refreshAll(m_supplySignalFollower1, m_tempSignalFollower1).isOK());
		inputs.supplyCurrentAmpsFollower1 = Math.abs(m_supplySignalFollower1.getValueAsDouble());
		inputs.temperatureCelsiusFollower1 = m_tempSignalFollower1.getValueAsDouble();

		inputs.follower2MotorConnected = m_follower2MotorConnectedDebouncer
				.calculate(BaseStatusSignal.refreshAll(m_supplySignalFollower2, m_tempSignalFollower2).isOK());
		inputs.supplyCurrentAmpsFollower2 = Math.abs(m_supplySignalFollower2.getValueAsDouble());
		inputs.temperatureCelsiusFollower2 = m_tempSignalFollower2.getValueAsDouble();

		inputs.follower3MotorConnected = m_follower3MotorConnectedDebouncer
				.calculate(BaseStatusSignal.refreshAll(m_supplySignalFollower3, m_tempSignalFollower3).isOK());
		inputs.supplyCurrentAmpsFollower3 = Math.abs(m_supplySignalFollower3.getValueAsDouble());
		inputs.temperatureCelsiusFollower3 = m_tempSignalFollower3.getValueAsDouble();
	}

	@Override
	public void runVolts(double volts) {
		m_mainTalon.setControl(m_voltsOut.withOutput(volts));
	}

	@Override
	public void runVelocity(double rpm) {
		m_mainTalon.setControl(m_rpmOut.withVelocity(rpm / 60.0));
	}

	@Override
	public void zeroEncoders() {
		m_mainTalon.setPosition(0);
	}

	@Override
	public void stop() {
		m_mainTalon.stopMotor();
	}
}
