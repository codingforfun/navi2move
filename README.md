# conn2m -- Talk with your navi2move

This is a copy of conn2m.py initially created by Markus Demleitner (http://katze.tfiu.de/projects/conn2m/)


## What is conn2m.py?

conn2m.py is a program intended to exchange data with an O-Synce navi2move GPS device.  
It's written in Python3 and was developed solely by serial protocol reverse engineering.

conn2m.py was originally intended to enable me as user of a free operating
system to interact with my device and to save me from the subjection
to the piece of proprietary software shipped with it, respectively.
It then evolved into a program substituting the complete set of device-related
functionality provided by the manufacturer's software, including the remote
setting of some behaviour also configurable through the user interface on
the device itself.

There is, however, no route planning functionality.  Use your favourite route
planning program instead, save the route as gpx file and upload it using
conn2m.py's send-route mode.


## Features

conn2m.py features include:

* downloading of recorded tracks from the device
* downloading of a previously uploaded route
* uploading of a local route gpx file to the device
* downloading of recorded waypoints (POIs)
* uploading of a local waypoint (POI) gpx file
* setting of general device configuration like power saving etc.
* setting of track recording configuration
* inclusion of bitmap data for some special characters like the german umlauts in route files which are not included when using the manufacturer's software. It's in principle possible to include any character with a 2-byte long utf-16 representation. If you have problems with any characters, let me know.


## License

Original license text as of http://katze.tfiu.de/projects/conn2m/:

    This program is distributed under the
    [GNU General Public License version 2](http://www.gnu.org/licenses/gpl-2.0.html),
     or, at your option, any later version.


## Disclaimer

This program comes as-is, with absolutely no warranty.  If you download this program, you agree that you use it at your own risk.

Although a lot of testing has been done and my device did never complain,
it can't be excluded that using this software may cause permanent damage
to your GPS device or even render it unusable.


## Dependencies

The program should run with python &gt;= 3.0.
Additional python dependencies are the following:

* python3-serial
* python3-crcmod
* python3-gdal


## Usage

For information on how to use conn2m.py, say
```
python3 conn2m.py --help
```








