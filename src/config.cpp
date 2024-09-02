#include "config.h"
#include <Arduino.h>
#include <FS.h>
#include <LittleFS.h>

config_t config;

void print_config() {
  Serial.print("config version: "); Serial.println(config.version);
  Serial.print("1 EIShaper_enabled: "); Serial.println(config.EIShaper_enabled);
  Serial.print("2 EIShaper_freq: "); Serial.println(config.EIShaper_freq);
  Serial.print("3 EIShaper_V: "); Serial.println(config.EIShaper_V);
  Serial.print("4 EIShaper_ctrl_freq: "); Serial.println(config.EIShaper_ctrl_freq);
  Serial.print("5 joint_vel_max: "); Serial.println(config.joint_vel_max);
  Serial.print("6 joint_acc: "); Serial.println(config.joint_acc);
  Serial.print("7 joint_backlash_compensate_feedforward: "); Serial.println(config.joint_backlash_compensate_feedforward);
  Serial.print("8 non_joint_vel_max: "); Serial.println(config.non_joint_vel_max);
  Serial.print("9 non_joint_acc: "); Serial.println(config.non_joint_acc);
}

void write_config() {
  File file = LittleFS.open("/config.txt", FILE_WRITE);
  if (!file) {
    Serial.println("Failed to open file for writing");
    return;
  }

  if (file.write((uint8_t *)&config, sizeof config) == 0) {
    Serial.println("Write failed");
    file.close();
    return;
  }

  file.close();
  
  Serial.println("Config written");
  print_config();
}

void read_config() {
  File file = LittleFS.open("/config.txt");
  if (!file) {
    Serial.println("Failed to open file for reading");

    Serial.println("Initing config");
    write_config();
    return;
  }

  if (file.isDirectory()) {
    Serial.println("Is dir?");
    file.close();
    return;
  }

  config_t temp_config;
  size_t read_size = file.read((uint8_t *)&temp_config, sizeof temp_config);
  if (read_size != (sizeof temp_config)) {
    Serial.println("Wrong size.");
    file.close();

    Serial.println("Initing config");
    write_config();
    return;
  }

  if (temp_config.version != config.version) {
    Serial.println("Wrong version");
    file.close();

    Serial.println("Initing config");
    write_config();
    return;
  }

  memcpy((uint8_t *)&config, (uint8_t *)&temp_config, sizeof config);

  file.close();

  Serial.println("Config read");
  print_config();
}

void configInit() {
  if (!LittleFS.begin(true)) {
    Serial.println("LittleFS Mount Failed");
    return;
  }

  read_config();
}