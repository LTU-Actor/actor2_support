# Access Point Setup Instructions

These instructions provide a guide through the configuration of the _TP Link EAP 255_ and the dependent software required to use this device as a Wireless Access Point, which will manage the main computer of the vehicle, along with the peripheral sensor devices.  

## TP Link EAP 225 setup instructions
The TP Link __EAP__ (Extensible Authentication Protocol) access point is configured like most routers.

Requirements:
* windows

### Accessing configuration tool

1. Connect the access point through the powered adapter that is included.
    * connection from physical access point to EAP port
    * connection from windows machine to LAN port

2. Connect to the wireless network provided by the access point (i.e. TP_Link_xxxx)

3. Set a static ip with the wireless connection:
    * _192.168.0.[2-250]_

4. Open browser and navigate to default access point IP: _192.168.0.254_

With these steps, hopefully, the TP Link login screen will appear.
The default credentials are _admin_ for both username and password.

### Configuring Access Point for Actor2

The access point EAP relies on a DHCP server to assign and manage IPs in the network. This DHCP server partitions a range of IPs (192.168.1.[150-200]) for the peripheral sensors. The rest of the IPs in the networks will be auto-assigned in the remaining space (192.168.1.[201+]).

Requirements:
* actor2 (NUK - linux[ubuntu])

#### DHCP Server

Following the instructions, provided by Ubuntu:
https://ubuntu.com/server/docs/network-dhcp

##### Installing server

Install the DHCP server package to _actor2_.

```
sudo apt install isc-dhcp-server
```

##### Configuring server

The default configuration file is located at: `/etc/dhcp/dhcpd.conf`.

Recommended configuration:
```
# minimal sample /etc/dhcp/dhcpd.conf
default-lease-time 600;
max-lease-time 7200;

subnet 192.168.1.0 netmask 255.255.255.0 {
 range 192.168.1.150 192.168.1.200;
 option routers 192.168.1.254;
 # option domain-name-servers 192.168.1.1, 192.168.1.2;
 # option domain-name "mydomain.example";
}

```

`default-lease-time 600;` will set the device lease with the network to _600 seconds_ or _10 minutes_.

`max-lease-time 7200;` sets a maximum length for a device lease with the network to _7200 seconds_ or _120 minutes_ or _2 hours_.

`subnet 192.168.1.0 netmask 255.255.255.0 {}` defines a subnetwork within which the DHCP server can work. This subset will exist at the location _192.168.1.[0-255]_.
    `range 192.168.1.150 192.168.1.200;` sets the range of IP addresses available within the subnetwork. Here the range is _192.168.1.[150-200]_.

    The rest of these options are specfic overridable parameters for subnets.
    * `option routers 192.168.1.254;` sets a specific address for the router. In our case, this is the address of the EAP, _192.168.0.254_.
    * `option domain-name-servers 192.168.1.1, 192.168.1.2;` is not needed, as we are continuing to use numeric IP address format.
    * `option domain-name "mydomain.example";` is not needed, as we are continuing to use numeric IP address format.

## Configuring Passthrough IP for Actor2

Passthrough IP will allow most traffic to passthrough to the internet. This allows devices to connect to the access point and access the internet.
Some IP addresses will not passthrough to the internet, however. These are the IP addresses within the DHCP subnet (i.e. the NUK computer and all of its peripheral sensors).

Requirements:
* actor2 (NUK - linux[ubuntu])

### IP Tables

IP Tables is a linux tool to manage and filter IP packets. It can be used for Firewalls.
Here we will use it to allow packets from the ethernet card to passthrough to the internet on the wireless card.

Information provided by: https://linux.die.net/man/8/iptables

#### MASQUERADE
* POSTROUTING
// TODO
