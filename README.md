# VideoSpeedTracker
The Video Speed Tracker (VST) is an open source, technically sound, vehicle speed measuring system
that can track bidirectional traffic, one lane in each direction (e.g. a typical residential street). 
Beyond a typical home computer, VST requires video from an HD camera, many of which can be purchased
for less than $100 (e.g. theFoscam Fi9103EP, power over Ethernet, outdoor camera). You need to have
OpenCV 2.4.11 installed.

Win7telx64 executables have been added on 15 Feb 2016.  See the section named "Installing executables" in the
User Manual.

Video example at  https://www.youtube.com/watch?v=1HVkKysDvGA

# Set-up Checklist
1) You really want 100 feet or more of open view (a maple tree trunk in the foreground is OK) of
the street. Your speed measuring zone should be at least one second wide for a vehicle going
the speed limit. For a speed limit of 25 MPH, that’s about 37 feet. My speed measuring zone is
1-1/3 seconds wide, which translates to about 50 feet for a vehicle going 25 MPH. I’ve been
able to track vehicles up to 71 MPH (Blue line on the front bumper the whole way!).

2) You need an HD (1280 x 720) video input stream for the stretch of street you want to analyze.
That requires a camera. I’m using the Foscam FI9803EP outdoor, HD, power over ethernet
camera. It’s about $90 online. I have no particular loyalty to Foscam, but I can say I’ve had
about a half dozen of their cameras and only one (an FI9821) has developed an issue (when it
pans, the video signal cuts out). That’s a decent record for an inexpensive cam that does HD
and 30FPS. My 9803EP has been working about four weeks, including through the infamous
Middle Atlantic January 2016 Snowzilla.

3) Aim the camera well. Aim the camera so that the street passes left to right
about half way up the overall image. This should minimize the effects of lens distortion. A
vehicle moving a constant velocity cross the lens moves at different pixel rates, frame to frame,
as it proceeds from an edge towards the center and back towards the far edge. My tracker is
what’s known as a “piecewise, linear least squares” tracker. The piecewise part (throwing out
the oldest data) is what allows it to adapt to changing pixel rates as a vehicle moves through
the scene. Setting up as you see in Figure 2 of the included overview
(in Docs/Overview and User Manual_v2.pdf) helps the tracker produce high quality results.

4) If you’re recompiling, you’ll need OpenCV 2.4.11 installed to have a successful compile. If
you’re executing a provided executable you’ll still need OpenCV 2.4.11 runtime libraries (dll’s in
Windows) before you can execute successfully.

5) Set critical one time setup values in VST.cfg. These values are described in the appendix of the included pdf under docs.

6) Create a location (the “prefixPath”) for five directories used by VST (and the final highlights
video processor):

a. IPCam -- You put directories for (typically) daily collections of .avi video files here.

b. HiLites -- VST outputs highlights videos into this directory

- forPosting -- Subdirectory for output of final highlights video processor

c. Stats – VST puts csv files with tracked vehicle data here

d. Trace – VST puts debug files here.

All of the files VST creates are given names derived from the input video file name (or the video file
directory if you answer “*” when asked for which file in a directory full of video files to process.) For
the IPCam directory, I create subdirectories with syntax yyyymmdd (e.g. 20160205) in which I place
that day’s video files.

A much more detailed version of this readme is available in "Docs/Overview and User Manual_v2.pdf".

## Original Author and Development Lead
- Paul Reynolds (reynolds@virginia.edu) www.cs.virginia.edu/~pfr

## License

- BSD 3-Clause
