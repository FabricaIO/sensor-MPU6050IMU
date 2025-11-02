#include "MPU6050IMU.h"

/// @brief Creates an MPU6050 sensor
/// @param Name The device name
/// @param I2C_bus The I2C bus attached to the sensor
/// @param I2CAddress The I2C address of the sensor
MPU6050IMU::MPU6050IMU(String Name, TwoWire* I2C_bus, int I2CAddress, String ConfigFile) : MPU6050_sensor(I2CAddress, I2C_bus), Sensor(Name) {
	i2c_bus = I2C_bus;
	config_path = "/settings/sen/" + ConfigFile;
}

/// @brief Creates an MPU6050 sensor
/// @param Name The device name
/// @param sda SDA pin to use for I2C bus
/// @param scl SCL pin to use for I2C bus
/// @param I2C_bus The I2C bus attached to the sensor
/// @param I2CAddress The I2C address of the sensor
MPU6050IMU::MPU6050IMU(String Name, int sda, int scl, TwoWire* I2C_bus, int I2CAddress, String ConfigFile) : MPU6050_sensor(I2CAddress, I2C_bus), Sensor(Name) {
	i2c_bus = I2C_bus;
	scl_pin = scl;
	sda_pin = sda;
	config_path = "/settings/sen/" + ConfigFile;
}

/// @brief Starts a gas sensor
/// @return True on success
bool MPU6050IMU::begin() {
	Description.parameterQuantity = 15;
	Description.type = "Motion Sensor";
	Description.parameters = {"accX", "accY", "accZ", "temp", "gyroX", "gyroY", "gyroZ",
		"angleAccX", "angleAccY", "angleGyroX", "angleGyroY", "angleGyroZ",
		"angleX", "angleY", "angleZ"};
	Description.units = {"g", "g", "g", "C", "deg/s", "deg/s", "deg/s", "deg", "deg", "deg", 
		"deg", "deg", "deg", "deg", "deg"};
	values.resize(Description.parameterQuantity);
	// Start I2C bus if not started
	if (scl_pin > -1 && sda_pin > -1) {
		if (!i2c_bus->begin(sda_pin, scl_pin)) {
			return false;
		}
	} else {
		if (!i2c_bus->begin()) {
			return false;
		}
	}
	bool result = false;
	// Start the sensor
	if(MPU6050_sensor.begin()) {
		// Create settings directory if necessary
		if (!checkConfig(config_path)){
			result = saveConfig(config_path, getConfig());
		} else {
			// Load settings
			result = setConfig(Storage::readFile(config_path), false);
		}
		if (mpu_config.autoCalibrate) {
			calibrateGyro();
		};
		MPU6050_sensor.resetAngles();
	}
	return result;
}

/// @brief Gets the current config
/// @return A JSON string of the config
String MPU6050IMU::getConfig() {
	// Allocate the JSON document
	JsonDocument doc;
	// Assign current values
	doc["Name"] = Description.name;
	doc["autoCalibrate"] = mpu_config.autoCalibrate;
	doc["calibrateNow"] = false;
	doc["angelReset"] = mpu_config.angleReset;
	doc["accelRange"]["current"] = mpu_config.accelRange;
	doc["accelRange"]["options"][0] = "2g";
	doc["accelRange"]["options"][1] = "4g";
	doc["accelRange"]["options"][2] = "8g";
	doc["accelRange"]["options"][3] = "16g";
	doc["gyroRange"]["current"] = mpu_config.gyroRange;
	doc["gyroRange"]["options"][0] = "250 deg/s";
	doc["gyroRange"]["options"][1] = "500 deg/s";
	doc["gyroRange"]["options"][2] = "1000 deg/s";
	doc["gyroRange"]["options"][3] = "2000 deg/s";

	// Create string to hold output
	String output;
	// Serialize to string
	serializeJson(doc, output);
	return output;
}

/// @brief Sets the configuration for this device
/// @param config A JSON string of the configuration settings
/// @param save If the configuration should be saved to a file
/// @return True on success
bool MPU6050IMU::setConfig(String config, bool save) {
	// Allocate the JSON document
	JsonDocument doc;
	// Deserialize file contents
	DeserializationError error = deserializeJson(doc, config);
	// Test if parsing succeeds.
	if (error) {
		Serial.print(F("Deserialization failed: "));
		Serial.println(error.f_str());
		return false;
	}
	// Assign loaded values
	Description.name = doc["Name"].as<String>();
	mpu_config.autoCalibrate = doc["autoCalibrate"].as<bool>();
	mpu_config.angleReset = doc["angelReset"].as<bool>();
	mpu_config.accelRange = doc["accelRange"]["current"].as<String>();
	mpu_config.gyroRange = doc["gyroRange"]["current"].as<String>();

	MPU6050_sensor.setAccelerometerRange(accelRanges[mpu_config.accelRange]);
	MPU6050_sensor.setGyroRange(gyroRanges[mpu_config.gyroRange]);
	
	// Calibrate gyro if requested
	if (doc["calibrateNow"].as<bool>()) {
		calibrateGyro();
	}

	// Save file if necessary
	if (save) {
		return saveConfig(config_path, config);
	}
	return true;
}

/// @brief Take a measurement and stores it in an internal variable
/// @return True on success
bool MPU6050IMU::takeMeasurement() {
	MPU6050_sensor.update();
	values[0] = MPU6050_sensor.getAccX();
	values[1] = MPU6050_sensor.getAccY();
	values[2] = MPU6050_sensor.getAccZ();
	values[3] = MPU6050_sensor.getTemperature();
	values[4] = MPU6050_sensor.getGyroX();
	values[5] = MPU6050_sensor.getGyroY();
	values[6] = MPU6050_sensor.getGyroZ();
	values[7] = MPU6050_sensor.getAccAngleX();
	values[8] = MPU6050_sensor.getAccAngleY();
	values[9] = MPU6050_sensor.getGyroAngleX();
	values[10] = MPU6050_sensor.getGyroAngleY();
	values[11] = MPU6050_sensor.getGyroAngleZ();
	values[12] = MPU6050_sensor.getAngleX();
	values[13] = MPU6050_sensor.getAngleY();
	values[14] = MPU6050_sensor.getAngleZ();
	if (mpu_config.angleReset) {
		MPU6050_sensor.resetAngles();
	}
	return true;
}

/// @brief Runs the gyroscope calibration procedure
void MPU6050IMU::calibrateGyro() {
	Logger.println("Calibrating gyro, don't move the sensor!");
	MPU6050_sensor.calcGyroOffsets(250, 0);
	Logger.println("Calibrating finished");
	Logger.println("Offsets:");
	Logger.print("X: ");
	Logger.println(MPU6050_sensor.getGyroXoffset());
	Logger.print("Y: ");
	Logger.println(MPU6050_sensor.getGyroYoffset());
	Logger.print("Z: ");
	Logger.println(MPU6050_sensor.getGyroZoffset());
}