idf_component_register(SRCS "TXStream/TXStream.cpp"
  INCLUDE_DIRS "TXStream"
  REQUIRES esp32-camera esp_wifi esp_netif ReedSolomon StreamProtocol esp_timer
)