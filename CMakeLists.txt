idf_component_register(SRCS
  "I2CBus.cpp"
  "Peripheral.cpp"
  "Max3010x.cpp"
  INCLUDE_DIRS "include"
  REQUIRES "driver" esp_timer
)
