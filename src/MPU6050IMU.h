/*
* This file and associated .cpp file are licensed under the GPLv3 License Copyright (c) 2025 Sam Groveman
* Contributors: Sam Groveman
* 
* External libraries needed:
* MPU6050Add: https://github.com/ShVerni/MPU6050Add
* 
*/

#pragma once
#include <Arduino.h>
#include <MPU6050Add.h>
#include <Wire.h>
#include <Sensor.h>
#include <map>

/// @brief Class for interfacing with an MPU6050
class MPU6050IMU: public Sensor {
	public:        
		MPU6050IMU(String Name, TwoWire* I2C_bus = &Wire, int I2CAddress = 0x68, String ConfigFile = "MPU6050IMU.json");
		MPU6050IMU(String Name, int sda, int scl, TwoWire* I2C_bus = &Wire, int I2CAddress = 0x68, String ConfigFile = "MPU6050IMU.json");
		bool begin();
		String getConfig();
		bool setConfig(String config, bool save);
		bool takeMeasurement();

	protected:
		/// @brief Stores configuration for MPU
		struct {
			/// @brief Automatically calibrate the gyroscope on start up
			bool autoCalibrate = true;

			/// @brief Calibrate the gyroscope immediately 
			bool calibrateNow = false;

			/// @brief Reset the angle calculations on each measurement
			bool angleReset = false;
			
			/// @brief Stores current accelerometer range
			String accelRange = "2g";

			/// @brief Stores current gyroscope range
			String gyroRange = "250 deg/s";
		} mpu_config;

		/// @brief Maps configuration schemes to accelerometer ranges
		std::map<String, MPU6050Add::accel_range> accelRanges = {{"2g", MPU6050Add::accel_range::MPU6050_RANGE_2_G}, 
			{"4g", MPU6050Add::accel_range::MPU6050_RANGE_4_G}, {"8g", MPU6050Add::accel_range::MPU6050_RANGE_8_G}, 
			{"16g", MPU6050Add::accel_range::MPU6050_RANGE_16_G}};

		/// @brief Maps configuration schemes to gyroscope ranges
		std::map<String, MPU6050Add::gyro_range > gyroRanges = {{"250 deg/s", MPU6050Add::gyro_range::MPU6050_RANGE_250_DEG}, 
			{"500 deg/s", MPU6050Add::gyro_range::MPU6050_RANGE_500_DEG}, {"1000 deg/s", MPU6050Add::gyro_range::MPU6050_RANGE_1000_DEG}, 
			{"2000 deg/s", MPU6050Add::gyro_range::MPU6050_RANGE_2000_DEG}};
	
		/// @brief I2C bus in use
		TwoWire* i2c_bus;

		/// @brief SCL pin in use
		int scl_pin = -1;

		/// @brief SDA pin in use
		int sda_pin = -1;

		/// @brief Stores location of config file
		String config_path;

		/// @brief Gas sensor object
		MPU6050Add MPU6050_sensor;

		void calibrateGyro();
};
