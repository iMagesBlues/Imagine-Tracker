# Imagine-Tracker
Open Source AR Tracker for Unity

<p>
 <img src="https://user-images.githubusercontent.com/22974556/59549981-e36e4880-8f97-11e9-9444-6cb077f8d37d.gif">
</p>

To run in Unity Standalone-Mac:
- 1.) Make sure your imagetarget is under Assets/Imagetargets/<your_imagetarget_name>.jpg
- 2.) In scene.unity. Drag image to ARCamera and Imagetarget gameobjects
- 3.) Hit Play

To setup in XCode:
- 1.) Install opencv3 (preferably using homebrew)
- 2.) Open imagineAR.xcodeproj
- 3.) In build settings, change library search paths, header search paths and other linker flags with your opencv3 version:
(eg. /usr/local/Cellar/opencv@3/3.4.5_2/include)

To test the plugin in Xcode:
- 1.) In XCode Project View. Right click "imagineAR>Products>imagineAR Plugin Tests" then "Show In Finder". Paste your image target here.
- 2.) In main.cpp rename "panda.jpg" with <your_imagetarget_name>.jpg.
- 3.) Select "imagineAR plugin test" as build target.
- 4.) Hit play

To build the plugin:
- 1.) Select "RenderingPlugin" as build target.
- 2.) Hit play. This will automatically save the .bundle file to "UnityProject/Assets/imagineAR/Plugins". But you will need to restart unity to reload the new build

Known-issues and limitations:
- 1.) Standalone-Win, Mobile-iOS and Mobile-Android will be supported very soon in the next revisions.
- 2.) Only works on a single imagetarget for now.
- 3.) Can't quit Unity after running the plugin. You need to Force-quit for now.. ugh!
- 4.) Displaying the Webcam image slows down the tracker. Needs an efficient method to convert openCV Mat to gl texture


Shout-out to fellow devs (especially to opencv devs and plugin creators) - 
Code is still in very early stages of development. And developer has very limited opencv and plugin creation knowledge.
Please help the community by sharing your improvements :) Any help from you will be very invaluable! :) 
It's about time we move forward and democratize AR development together ;)




 
