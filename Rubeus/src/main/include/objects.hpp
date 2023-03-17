// All the global objects


SwerveModule frontLeftSwerve (
	new SparkMotor(FRONT_LEFT_SPEED),
	new SparkMotor(FRONT_LEFT_DIREC), 
	FRONT_LEFT_CANCODER,
	1, 
	-1024 + FRONT_LEFT_OFFSET
);

SwerveModule frontRightSwerve (
	new SparkMotor(FRONT_RIGHT_SPEED),
	new SparkMotor(FRONT_RIGHT_DIREC),
	FRONT_RIGHT_CANCODER,
	2, 
	1024 + FRONT_RIGHT_OFFSET
);

SwerveModule mainSwerve (
	new SparkMotor(BACK_LEFT_SPEED),
	new SparkMotor(BACK_LEFT_DIREC),
	BACK_LEFT_CANCODER,
	4, 
	1024 + BACK_LEFT_OFFSET
);

SwerveModule backRightSwerve (
	new SparkMotor(BACK_RIGHT_SPEED),
	new SparkMotor(BACK_RIGHT_DIREC),
	BACK_RIGHT_CANCODER,
	3, 
	-1024 + BACK_RIGHT_OFFSET
);

Arm <1, 0, 5, 1, 3> arm {
	new TalonSRXMotor(ARM_SHOULDER),
	new SparkMotor(ARM_ELBOW),
	new SparkMotor(ARM_HAND)
};

Controls <5, 4, 3> controls;

frc::DoubleSolenoid armSol { frc::PneumaticsModuleType::CTREPCM, 0, 1 };

AHRS navx {frc::SPI::Port::kMXP}; // Well, obviously, the navx

Odometry <&mainSwerve> odometry("limelight"); /* This is what we call misusing templates and doing a bad job of it */

frc::Compressor compressor {frc::PneumaticsModuleType::CTREPCM};