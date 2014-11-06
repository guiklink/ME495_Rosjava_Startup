##Getting a ROS Android Workspace##

Start by getting [Android Studio](https://developer.android.com/sdk/installing/studio.html) into your computer.
Then get the ROS Android repositories in your computer and start your workspace.

```
> mkdir -p ~/android
> wstool init -j4 ~/android/src https://raw.github.com/rosjava/rosjava/hydro/android_core.rosinstall  
> source /opt/ros/indigo/setup.bash
> source ~/rosjava/devel/setup.bash # source into the setup.bash file of your ROSJava a workspace.  
> cd ~/android
> export ANDROID_HOME="~/Android_Devel/adt-bundle-linux-x86_64-20140702/sdk" # create an env variable that points to your sdk 
> catkin_make  
```
 
During your first ```catkin_make``` lots of **errors** can be raised from Android packages not found. If that is the case, the easiest way to download it is using your Android Studio. 

In order to do that, open your Android Studio and click on ![](https://github.com/guiklink/ME495_Rosjava_Startup/blob/master/SDK_Manager_Logo.png) on the upper right of your screen.

In addition, in some cases (specially if you are using a 64 bit computer) you might have to add missing C++ libraries. This happens because Android relies in a bunch of 32 bit libraries to execute.If that is the case this [link](http://askubuntu.com/questions/454253/how-to-run-32-bit-app-in-ubuntu-64-bit) shows you how to get the missing packages working in your 64 bit machine.

### Linking your Android Studio with your workspace and executing your first demos###

The following [tutorial](http://wiki.ros.org/ApplicationsPlatform/Clients/Android/Tutorials/Getting%20Started) is very strait forward in how to get your first apps executing.

Unfortunately, the tutorial fail to remark that as all ROS applications you need to add your **dependencies**. For Android applications the dependencies should go inside the **build.gradle** file inside the package.

Here is an example of how this files looks like:
```/*
 * Copyright (C) 2011 Google Inc.
 *
 * Licensed under the Apache License, Version 2.0 (the "License"); you may not
 * use this file except in compliance with the License. You may obtain a copy of
 * the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
 * License for the specific language governing permissions and limitations under
 * the License.
 */

dependencies {
  compile 'org.ros.rosjava_core:rosjava:[0.1,0.2)'
  compile 'org.ros.rosjava_messages:sensor_msgs:[1.10,1.11)'
  compile project(':android_gingerbread_mr1')
}

apply plugin: 'android-library'

android {
    compileSdkVersion 10
}
```

Here is how the dependencies are added:    
```compile 'org.ros.rosjava_core:rosjava:[0.1,0.2)' ```
```compile 'org.ros.rosjava_messages:sensor_msgs:[1.10,1.11)' ```   
```compile project(':android_gingerbread_mr1')```   

The dependency ```compile project(':android_gingerbread_mr1')``` is particular import since it will be the one that provides most of the Android features.

###IMPORTANT###
After creating your first packages you might face some *manifest merger* conflicts when you try to build them using ```catkin_make```. Most of the time you will be given commands in your **Terminal** to be add in your *manifest.xml* to fix the problems. Although, it will be times that duplicate files will give you package name conflicts. For this situations executing this command ```find ~/.gradle/caches/ -iname "*.jar" -exec zip -d '{}' 'META-INF/LICENSE*' \;``` will clean your caches and make it possible for you to carry on.
 



