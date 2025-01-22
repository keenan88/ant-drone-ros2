#ifndef MICRO_ROS_TRANSPORT_H_
#define MICRO_ROS_TRANSPORT_H_

#include <Arduino.h>
#include <ETH.h>
#include <micro_ros_platformio.h>
#include <rmw_microros/rmw_microros.h>
#include <uxr/client/profile/transport/custom/custom_transport.h>
#include <uxr/client/util/time.h>

struct micro_ros_agent_locator {
  IPAddress address;
  int port;
};

void EthernetEventCallback(WiFiEvent_t event);

static inline void set_microros_ethernet_transports(IPAddress client_ip,
                                                    IPAddress netmask,
                                                    IPAddress agent_ip,
                                                    uint16_t agent_port) {
  static struct micro_ros_agent_locator locator;

  WiFi.onEvent(EthernetEventCallback);
  ETH.begin();
  ETH.config(client_ip, IPAddress(0, 0, 0, 0), netmask);
  delay(1000);

  locator.address = agent_ip;
  locator.port = agent_port;

  rmw_uros_set_custom_transport(
    false, &locator, 
    platformio_transport_open, 
    platformio_transport_close,
    platformio_transport_write, 
    platformio_transport_read
  );
}

#endif  // MICRO_ROS_TRANSPORT_H_