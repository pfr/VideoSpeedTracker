# VideoSpeedTracker
The Video Speed Tracker (VST) is an open source, technically sound, vehicle speed measuring system
that can track bidirectional traffic, one lane in each direction (e.g. a typical residential street). 
Beyond a typical home computer, VST requires video from an HD camera, many of which can be purchased
for less than $100 (e.g. the Foscam Fi9103EP, power over Ethernet, outdoor camera.  If you buy a POE camera,
be sure to buy something like the TP-LINK TL-PoE150S - about $18 - to power it.).

Win7telx64 executables have been added on 15 Feb 2016.  See the section named "Installing executables" in the
User Manual.

As of 31 March 2016, watch the use of the speed tracker in support of documenting Charlottesville, Virginia's current struggles with urban vehicular assault in its established residential neighborhoods at http://eyetach.github.io/CharlottesvilleSpeeds/.  

Video example:

[![Video example](http://img.youtube.com/vi/1HVkKysDvGA/0.jpg)](http://www.youtube.com/watch?v=1HVkKysDvGA "Video Speed Tracker on a 25 MPH Residential Street")

# Set-up Checklist
1. You really want 100 feet or more of open view (a maple tree trunk in the foreground is OK) of
the street. Your speed measuring zone should be at least one second wide for a vehicle going
the speed limit. For a speed limit of 25 MPH, that’s about 37 feet. My speed measuring zone is
1-1/3 seconds wide, which translates to about 50 feet for a vehicle going 25 MPH. I’ve been
able to track vehicles up to 71 MPH (Blue line on the front bumper the whole way!).

2. You need an HD (1280 x 720) video input stream for the stretch of street you want to analyze.
That requires a camera. I’m using the Foscam FI9803EP outdoor, HD, power over ethernet
camera. It’s about $90 online. I have no particular loyalty to Foscam, but I can say I’ve had
about a half dozen of their cameras and only one (an FI9821) has developed an issue (when it
pans, the video signal cuts out). That’s a decent record for an inexpensive cam that does HD
and 30FPS. My 9803EP has been working about four weeks, including through the infamous
Middle Atlantic January 2016 Snowzilla.

3. Aim the camera well. Aim the camera so that the street passes left to right
about half way up the overall image. This should minimize the effects of lens distortion. A
vehicle moving a constant velocity cross the lens moves at different pixel rates, frame to frame,
as it proceeds from an edge towards the center and back towards the far edge. My tracker is
what’s known as a “piecewise, linear least squares” tracker. The piecewise part (throwing out
the oldest data) is what allows it to adapt to changing pixel rates as a vehicle moves through
the scene. Setting up as you see in Figure 2 of the included overview
(in docs/Video Speed Tracker Overview and User Manual.md) helps the tracker produce high quality results.

4. If you’re recompiling, you’ll need OpenCV 2.4.11 installed to have a successful compile. If
you’re executing a provided executable the required OpenCV 2.4.11 x64/vc12 runtime libraries (dll’s in
Windows) are provided in /bin.

5. Set critical one time setup values in VST.cfg. These values are described in the appendix of the included pdf under docs.

6. Create a location (the “prefixPath”) for five directories used by VST (and the final highlights
video processor):

  1. IPCam – You put directories for (typically) daily collections of .avi video files here.

  2. HiLites – VST outputs highlights videos into this directory

    1.  forPosting – Subdirectory for output of final highlights video processor

  3. Stats – VST puts csv files with tracked vehicle data here

  4. Trace – VST puts debug files here.

All of the files VST creates are given names derived from the input video file name (or the video file
directory if you answer “*” when asked for which file in a directory full of video files to process.) For
the IPCam directory, I create subdirectories with syntax yyyymmdd (e.g. 20160205) in which I place
that day’s video files.

## Additional Documentation

- A much more detailed version of this readme is available
[**here.**](docs/Video Speed Tracker Overview and User Manual.md)

## Original Author and Development Lead
- Paul Reynolds (reynolds@virginia.edu) www.cs.virginia.edu/~pfr

## License

- BSD 3-Clause
