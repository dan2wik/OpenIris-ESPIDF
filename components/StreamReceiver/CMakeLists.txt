idf_component_register(SRCS "StreamReceiver/StreamReceiver.cpp"
  INCLUDE_DIRS "StreamReceiver"
  REQUIRES esp_wifi esp_netif esp_timer ReedSolomon StreamProtocol
)
