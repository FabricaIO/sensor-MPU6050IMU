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
		} mpu_config;
	
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
