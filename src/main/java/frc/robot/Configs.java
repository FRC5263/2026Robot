package frc.robot;

import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

public class Configs {
    public final class SwerveConfigs{
        private static final SparkMaxConfig driveConfig = new SparkMaxConfig();
        private static final SparkMaxConfig angleConfig = new SparkMaxConfig();

        public SwerveConfigs(){
        }

        public void configure(){
            driveConfig.idleMode(IdleMode.kBrake);
            angleConfig.idleMode(IdleMode.kBrake);

            driveConfig.smartCurrentLimit(75);
            angleConfig.smartCurrentLimit(75);
        }

        public static SparkMaxConfig getDriveConfig(){
            return driveConfig;
        }

        public static SparkMaxConfig getAngleConfig(){
            return angleConfig;
        }
    }

    public static final class otherMotors{
        private static final SparkMaxConfig shooterConfig = new SparkMaxConfig();
        
        public otherMotors(){
            configure();
        }

        public void configure(){}

        public SparkMaxConfig getShooter(){
            return shooterConfig;
        }
    }
}
