package frc.robot.subsystems.indexer.io;

import static frc.robot.subsystems.indexer.IndexerConstants.kGearbox;
import static frc.robot.subsystems.indexer.IndexerConstants.kGearboxReduction;
import static frc.robot.subsystems.indexer.IndexerConstants.kMomentOfInertia;
import static frc.robot.subsystems.indexer.IndexerConstants.kMotorStatorLimitAmps;

import com.bobcats.lib.subsystem.rollers.io.GenericRollerIOSim;

/**
 * The IO sim implementation for indexer hardware interacions without any hardware.
 */
public class IndexerIOSim extends GenericRollerIOSim implements IndexerIO {
	/**
	 * Constructs a new IndexerIOSim.
	 */
	public IndexerIOSim() {
		super(kGearbox, kMomentOfInertia, kGearboxReduction, 1, Double.POSITIVE_INFINITY, kMotorStatorLimitAmps, false);
	}
}
