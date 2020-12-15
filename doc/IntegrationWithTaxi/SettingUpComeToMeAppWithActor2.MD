# Instructions for setting up Android device to send location to Actor2

## Install the ComeToMe app

The ComeToMe android application is available through invitation from Google Firebase.

The Google Account already signed into the K10 tablet (actor2.ltu@gmail.com) is an account owner, and can be used to initiate an email request.

Once the invitation is sent, follow the instructions in the email to download and install the ComeToMe application on your device.

### Link to Firebase App Distribution
Link:
https://console.firebase.google.com/u/3/project/cometome-f87ca/appdistribution/app/android:com.ltu.actor.comeToMe/releases

## Install the OpenVPN app

Navigate to this link, or search 'OpenVPN' in the Google PlayStore to install.

https://play.google.com/store/apps/details?id=net.openvpn.openvpn&hl=en_US&gl=US

## Download the actor2 general VPN profile

This vpn profile ('PleuneNET-1.0-general') is used by any device wanting to connect to the Actor2 VPN.

This profile is included in the directory with these instructions ([link](PleuneNET-1.0-general.ovpn)).

Please, save this profile to an accessible place on the device.

# Running ComeToMe

## Set up OpenVPN

1. Launch OpenVPN
2. The landing screen should have an option to enter an url or select a file -- select the Tab for 'File'
3. Select the 'PleuneNET-1.0-general.ovpn' file from your device's file directory
4. Press 'Import'
5. Press 'Add' on the top right corner

Now, the profile should be preferred by OpenVPN.

6. Toggle the profile ON
7. (Good Practice: Toggle this profile OFF, when not communicating with ACTor2)

When toggled ON, there should be an activity graph showing network traffic over the vpn.

## Launch ComeToMe

1. Launch ComeToMe, titled: "Press to send location to LTU ACTor".
2. Select 'Allow', when prompted "Allow ACTor-ComeToMe to access this device's location?"
3. Find a map centered on Lat, Long (0.0, 0.0) 
4. Press the Google map current location button on the top right of the screen, to enable location tracking on the map (crosshairs with circle in center)
5. Find the map recenter to the device location
6. Press 'ComeToMe' button, to send device location to ACTor2 over VPN
7. Find message, "Sending Location to: http://192.168.99.5:8642"
8. Once connection is complete and ACTor2 receves location, message will display "Vehicle Received Location!"