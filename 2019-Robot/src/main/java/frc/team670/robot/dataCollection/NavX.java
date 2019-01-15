package frc.team670.robot.dataCollection;

import com.kauailabs.navx.AHRSProtocol.AHRSUpdateBase;
import com.kauailabs.navx.frc.AHRS;
import com.kauailabs.navx.frc.ITimestampedDataSubscriber;

import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.SerialPort;
import frc.team254.lib.util.math.Rotation2d;

/**
 * Driver for a NavX board. Basically a wrapper for the AHRS class. Much of this was taken from 254's code release.
 */
public class NavX {
    protected class Callback implements ITimestampedDataSubscriber {
        @Override
        public void timestampedDataReceived(long system_timestamp, long sensor_timestamp, AHRSUpdateBase update,
                Object context) {
            synchronized (NavX.this) {
                // This handles the fact that the sensor is inverted from our coordinate conventions.
                if (mLastSensorTimestampMs != kInvalidTimestamp && mLastSensorTimestampMs < sensor_timestamp) {
                    mYawRateDegreesPerSecond = -1 * 1000.0 * (-mYawDegrees - update.yaw) // Multiply by -1 because our system is opposite of Bellarmine's
                            / (double) (sensor_timestamp - mLastSensorTimestampMs);
                }
                mLastSensorTimestampMs = sensor_timestamp;
                mYawDegrees = update.yaw; // This used to be multiplied by -1 to flip it, but our coord system is the opposite of Bellarmine's
            }
        }
    }

    protected AHRS mAHRS;

    protected Rotation2d mAngleAdjustment = Rotation2d.identity();
    protected double mYawDegrees;
    protected double mYawRateDegreesPerSecond;
    protected final long kInvalidTimestamp = -1;
    protected long mLastSensorTimestampMs;
    private double offSet;

    public NavX(SPI.Port spi_port_id) {
        mAHRS = new AHRS(spi_port_id, (byte) 200);
        resetState();
        mAHRS.registerCallback(new Callback(), null);
    }

    public NavX(SerialPort.Port port) {
        mAHRS = new AHRS(port);
        resetState();
        mAHRS.registerCallback(new Callback(), null);
    }

    /**
     * Resets and recalibrates the NavX (yaw will go back to zero and offset cleared). Call this right at the beginning of the match.
     */
    public synchronized void reset() {
        mAHRS.reset();
        resetState();
        offSet = 0;
    }

    /**
     * Zeroes the yaw for getYawDouble()
     */
    public synchronized void zeroYaw() {
        setOffSetAngle();
        resetState();
    }

    private void resetState() {
        mLastSensorTimestampMs = kInvalidTimestamp;
        mYawDegrees = 0.0;
        mYawRateDegreesPerSecond = 0.0;
    }

    // public synchronized void setAngleAdjustment(Rotation2d adjustment) {
    //     mAngleAdjustment = adjustment;
    // }

    /**
     * The Field Centric Yaw
     */
    private synchronized double getRawYawDegrees() {
        return mYawDegrees;
    }

    /**
     * Gets the yaw with offset taken into account. Offset sets the zero of the gyro to the point where zeroYaw() was last called.
     */
    public synchronized double getYawDouble() {
        double rtrnAngle = getRawYawDegrees() - offSet;
        while (rtrnAngle > 180) { 
            rtrnAngle = rtrnAngle - 360; // returns the same angle but in range [-180, 180]
        }
        while (rtrnAngle < -180) {
            rtrnAngle = rtrnAngle + 360; 
        }
        return rtrnAngle;
    }

    /**
     * The rate of change of the NavX angle in degrees per second.
     */
    public synchronized double getYawRateDegreesPerSec() {
        return mYawRateDegreesPerSecond;
    }

    /**
     * The rate of change of the NavX angle in radians per second.
     */
    public synchronized double getYawRateRadiansPerSec() {
        return 180.0 / Math.PI * getYawRateDegreesPerSec();
    }

    public synchronized double getRawAccelX() {
        return mAHRS.getRawAccelX();
    }

    private synchronized void setOffSetAngle() {
        offSet = getRawYawDegrees();
    }

    /**
     * Gets the NavX as a PIDSource that responds to the NavX being zeroed.
     */
    public synchronized ZeroableNavX_PIDSource getZeroableNavXPIDSource() {
        return new ZeroableNavX_PIDSource();
    }

    /**
     * Gets the NavX object itself so be careful with it and don't reset it. This will be field centric.
     */
    public synchronized AHRS getFieldCentricNavXPIDSource () {
        return mAHRS;
    }

    /**
     * Gets the field centric yaw (0 degrees is forward for the robot from its starting position),
     * @return The Yaw (-180, 180) with -180 and 180 being directly backwards.
     */
    public synchronized double getYawFieldCentric() {
        return getRawYawDegrees();
    }
    public class ZeroableNavX_PIDSource implements PIDSource{

        private PIDSourceType type;

        public ZeroableNavX_PIDSource() {
            type = PIDSourceType.kDisplacement;
        }

        @Override
        public void setPIDSourceType(PIDSourceType pidSource) {
            type = pidSource;
        }

        @Override
        public PIDSourceType getPIDSourceType() {
            return type;
        }

        @Override
        public double pidGet() {
            return getYawDouble();
        }

    }

}
