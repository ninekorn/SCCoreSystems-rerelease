OCULUS RIFT CV1 only.

# SCCoreSystems-rerelease

you have a 60 days trial usuing the Ab3d.DXEngine nuggets. what i didn't code, i provide the link to the reference of stackoverflow and elsewhere on programming forums. no functionnal virtual reality keyboard yet though but sccsVD4ED and sccsVD4VE should have a working mouse for the virtual desktop. i have put up to date the references and nuggets. 

the username is "9" and the password is "std" for standard. 

List of programs:

sccsv10 - 1st version

sccsv11 - 2nd version

sc_core_systems - 3rd version. console dotnet directx virtual reality.

sccsVD4ED - 3rd version, using wpf. for elite dangerous - the steamVR overlay doesn't work properly yet unfortunately so i cannot have my virtual desktop screen sent to the steamVR overlay c# scripts. So this program doesn't even work with elite dangerous yet.

sccsVD4VE - 3rd version. using wpf. for void expanse.

Jitter - the original physics engine.

sc_message_object - i created this class because it was the only way i had found to have a stable multithreaded application where everything worked and that this single object can be passed to all threads inside of the application whether you have threads or background workers or tasks.

For the moment, i have chosen an MIT license for the rest of my repository that you can find here: https://github.com/ninekorn/SCCoreSystems-rerelease
PLEASE NOTE IT CURRENTLY ONLY WORKS WITH AN OCULUS RIFT CV1.  

This is a re-release of my repository posted on github 6 months ago for my c# Virtual Reality Virtual Desktop program, and i have left my original version private on github.
It is far from the new unity3d physics engine where one can load up to over 100000++ physics rigidbodies in one scene and all physics ready. But i liked being independant from unity3D but i am still using it from time to time as i've purchased nice assets in there.

I can do a console virtual reality version without using the ab3d.DXEngine.OculusWrap and using the ab3d.oculus wrap only dll which in turn isn't making you having to use the 60 days trial library. but i don't have the time to do that version yet and i see no reasons why i wouldn't use the libraries of the ab3d.dxengine because they are awesome. But i would consider coding that software if only people would request it.

But the quality of using the library ab3d.DXEngine.OculusWrap on top of the ab3d.OculusWrap is better and a lot of work was done already by Andrej Benedik towards having a good quality virtual reality solution for the oculus rift cv1 provided by his libraries. Using the oculus rift cv1 was my only option to make my own programs whenever i purchased the oculus rift cv1 on the 5th of August 2017. that's why currently my programs are only available for the oculus rift cv1 and that i did not invest in a newer oculus headset. i do not have an oculus rift s yet though, but c++ wrappers for the oculus rift cv1 for C# where made available with c++ entry points like this:

[DllImport(DllOvrDll, EntryPoint="ovr_SubmitFrame", SetLastError=false, CallingConvention=CallingConvention.Cdecl)]
private static extern Result ovr_SubmitFrame(ovrSession session, Int64 frameIndex, IntPtr viewScaleDesc, IntPtr layerPtrList, uint layerCount);

so if it was done for the oculus rift cv1, it might probably be done also for the other other oculus rift series headsets as wrappers in the future who knows, and thats when i might invest on an oculus rift s headset. On another note, i almost have all the pieces for a virtual reality linux ubuntu headset like this:
https://www.hackster.io/.../relativ-build-your-own-vr...
or this:
https://www.relativty.com/
and if i can make my c# programs for windows 10 work on ubuntu dotnet, it would become something even greater and it would be without the oculus headset. but some people at http://www.openhmd.net/index.php/devices/ almost have it all figured out for the oculus headsets compatibility in ubuntu.
thank you for reading me 🙂
steve chassé

--------------------------------------------------------------------------

I have learned to create my own direct3D c# programs and the main help from being able to get out of unity3d and be independant from unity3D was to code my own programs in C#. It was a sad choice, but back in unity 2017, adding the framework in order to make a virtual desktop from sharpdx was a tough thing to do and i couldn't make it happen. i then chose to learn by myself.

So for developping my softwares, i took this as a reference:
https://github.com/Dan6040/SharpDX-Rastertek-Tutorials
which is based on:
https://www.rastertek.com/tutindex.html
Also, I have implemented in my own way the inverse kinematics using the page here for the upper part of the virtual reality 1st person "controller":
https://mathworld.wolfram.com/Circle-CircleIntersection.html
Website for the jitter physics engine:
https://code.google.com/archive/p/jitterphysics/
Website for the windows input simulator:
https://archive.codeplex.com/?p=inputsimulator
Website for the Ab3d.OculusWrap
https://github.com/ab4d/Ab3d.OculusWrapiso
