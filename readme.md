##TeamCode Module

Welcome!

This module, TeamCode, is the place where our team has wrote the code for our robot controller app.

##Class Names

Note that some class names are not named in the best conventions possible, and therefore, it is highly recommended to look at the Op Mode name(right before class declaration) inside of the java file so that you can get a better understanding of what the class does. Also, reading our comments will ensure that you understand how our program runs.

##ADVANCED Multi-Team App management: Cloning the TeamCode Module

In case you import our files and need TeamCode Module help for multiple teams:

In some situations, you have multiple teams in your club and you want them to all share a common code organization, with each being able to see the others code but each having their own team module with their own code that they maintain themselves.

In this situation, you might wish to clone the TeamCode module, once for each of these teams. Each of the clones would then appear along side each other in the Android Studio module list, together with the FtcRobotController module (and the original TeamCode module).

Selective Team phones can then be programmed by selecting the desired Module from the pulldown list prior to clicking to the green Run arrow.

Warning: This is not for the inexperienced Software developer. You will need to be comfortable with File manipulations and managing Android Studio Modules. These changes are performed OUTSIDE of Android Studios, so close Android Studios before you do this.

Also.. Make a full project backup before you start this :)

To clone TeamCode, do the following:

1) Note: Some names start with "Team" and others start with "team". This is intentional.

2) Using your operating system file management tools, copy the whole "TeamCode" folder to a sibling folder with a corresponding new name, eg: "Team0417".

3) In the new Team0417 folder, delete the TeamCode.iml file.

4) the new Team0417 folder, rename the "src/main/java/org/firstinspires/ftc/teamcode" folder to a matching name with a lowercase 'team' eg: "team0417".

5) In the new Team0417/src/main folder, edit the "AndroidManifest.xml" file, change the line that contains package="org.firstinspires.ftc.teamcode" to be package="org.firstinspires.ftc.team0417"

6) Add: include ':Team0417' to the "/settings.gradle" file.

7) Open up Android Studios and clean out any old files by using the menu to "Build/Clean Project"
