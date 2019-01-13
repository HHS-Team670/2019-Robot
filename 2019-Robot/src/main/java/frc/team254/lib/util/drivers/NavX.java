package frc.team254.lib.util.drivers;

import com.kauailabs.navx.AHRSProtocol.AHRSUpdateBase;
import com.kauailabs.navx.frc.AHRS;
import com.kauailabs.navx.frc.ITimestampedDataSubscriber;

import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.SerialPort;
import frc.team254.lib.util.math.Rotation2d;

/**
 * yes Driver for a NavX board. Basically a wrapper for the {@link AHRS} class
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

    public synchronized void reset() {
        mAHRS.reset();
        resetState();
    }

    public synchronized void zeroYaw() {
        setOffSetAngle();
        resetState();
    }

    private void resetState() {
        mLastSensorTimestampMs = kInvalidTimestamp;
        mYawDegrees = 0.0;
        mYawRateDegreesPerSecond = 0.0;
    }

    public synchronized void setAngleAdjustment(Rotation2d adjustment) {
        mAngleAdjustment = adjustment;
    }

    protected synchronized double getRawYawDegrees() {
        return mYawDegrees;
    }

    public double getYawDouble() {
        double rtrnAngle = getRawYawDegrees() - offSet;
        while (rtrnAngle > 180) { 
            rtrnAngle = rtrnAngle - 360; // returns the same angle but in range [-180, 180]
        }
        while (rtrnAngle < -180) {
            rtrnAngle = rtrnAngle + 360; 
        }
        return rtrnAngle;
    }

    public double getYawRateDegreesPerSec() {
        return mYawRateDegreesPerSecond;
    }

    public double getYawRateRadiansPerSec() {
        return 180.0 / Math.PI * getYawRateDegreesPerSec();
    }

    public double getRawAccelX() {
        return mAHRS.getRawAccelX();
    }

    private void setOffSetAngle() {
        offSet = getRawYawDegrees();
    }

    public double getYawFieldCentric() {
        return getRawYawDegrees();
    }


}
