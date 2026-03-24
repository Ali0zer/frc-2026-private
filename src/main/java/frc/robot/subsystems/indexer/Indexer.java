package frc.robot.subsystems.indexer;

import static frc.robot.subsystems.indexer.IndexerConstants.kForwardVoltage;
import static frc.robot.subsystems.indexer.IndexerConstants.kGearboxReduction;
import static frc.robot.subsystems.indexer.IndexerConstants.kReverseVoltage;

import com.bobcats.lib.subsystem.rollers.GenericRollerSubsystem;
import com.bobcats.lib.subsystem.rollers.GenericRollerSubsystem.GenericRollerVoltage;

import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.subsystems.indexer.Indexer.IndexerGoal;
import frc.robot.subsystems.indexer.io.IndexerIO;

/**
 * The main indexer subsystem to carry fuel into the shooter mechanism from the hopper.
 */
public class Indexer extends GenericRollerSubsystem<IndexerGoal> {
	private IndexerGoal m_currentGoal = new IndexerGoal(0.0);

	/** The setpoint for the indexer motor voltage. */
	public static class IndexerGoal implements GenericRollerVoltage {
		private final double m_voltage;

		/**
		 * Constructs a new IndexerGoal.
		 *
		 * @param voltage The voltage to set the indexer motor to.
		 */
		public IndexerGoal(double voltage) {
			m_voltage = voltage;
		}

		@Override
		public double voltage() {
			return m_voltage;
		}
	}

	/**
	 * Constructs a new Indexer.
	 *
	 * @param io The IO implementation to use.
	 */
	public Indexer(IndexerIO io) {
		super("Indexer", io);
		setName("Indexer");
	}

	@Override
	public void periodic() {
		if (DriverStation.isDisabled()) m_currentGoal = new IndexerGoal(0.0);
		super.periodic();
	}

	/**
	 * Starts pushing balls into the feeder. Can be stopped via {@link #stop()} method.
	 */
	public void runForward() {
		m_currentGoal = new IndexerGoal(kForwardVoltage);
	}

	/**
	 * Stops feeding balls into the shooter.
	 */
	public void stop() {
		m_currentGoal = new IndexerGoal(0.0);
	}

	/**
	 * Moves any balls in the hopper back towards the intake to unfeed.
	 */
	public void runReverse() {
		m_currentGoal = new IndexerGoal(kReverseVoltage);
	}

	public void setVoltage(double volts) { m_currentGoal = new IndexerGoal(volts); }

	/**
	 * Returns the angular velocity of the indexer rollers.
	 *
	 * @return The angular velocity of the indexer rollers, in RPM.
	 */
	public double getVelocity() { return m_inputs.rpm / kGearboxReduction; }

	@Override
	public IndexerGoal getVoltageSetpoint() { return m_currentGoal; }
}
