actor2 NUK - 192.168.60.100

actor2 NUK DHCP - 192.168.60.101
    auto-assign range
                  192.168.60.150 - 192.168.60.200

velodyne lidar - 192.168.60.50
    (default ip - 192.168.1.201)

mako gige camera - 192.168.0.60

Piksi GPS

GPS 0:
    altitude
    front
    uart 0: GPS 1
        recieve position and bias from GPS 1
    uart 1: box port
    192.168.60.30

GPS 1:
    reference
    rear
    uart 0: GPS 0
        sends position and bias to GPS 0
    uart 1: radio
    192.168.60.31


tp link access point

  account
    username: actor2
    password: robofest2
    
  ssid: actor2
  pass: robofest2
  
  (default ip - 192.168.0.254)
