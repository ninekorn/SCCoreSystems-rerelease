2021-jan-02 - 03h56am
i updated my console solution sc_core_systems program to only use the ab3d.OculusWrap DLL only, instead of using both the ab3d.OculusWrap and the ab3d.DXEngine.OculusWrap. The reason is people have to build the solutions found here https://github.com/ab4d/Ab3d.OculusWrap in order to build the ab3d.DXEngine.OculusWrap from source as it activates a nugget ab3d.dxengine. So if anyone wants the ab3d.dxengine.oculusWrap graphics, follow the steps i posted in my prior post for today.

regards,
steve chassé

2021-jan-02-
i wanted this to have the 60 days trial using the ab3d.dxengine.OculusWrap, right out of the box, for people to use the ab3d.dxengine.oculusWrap. I modified my repos so you won't be able to make them work out of the box anymore.

You will have to go clone the repository here https://github.com/ab4d/Ab3d.OculusWrap and build the dlls separately for yourselves. If the ab3d.dxengine.oculusWrap would be provided in the future with a nugget you won't have to do those steps. 

1. Clone the github repository here: https://github.com/ab4d/Ab3d.OculusWrap
2. Build the ab3d.OculusWrap solution first with the FrameWork 4.5 or 4.7.2 whatever.
3. Then build the solution ab3d.DXEngine.OculusWrap.
4. use both the ab3d.OculusWrap.dll and the ab3d.DXEngine.OculusWrap as references for my projects sc_core_systems and SCCoreSystems and the solution sccsv10 and this one sccsv11 and that one too sccsVD4VE. Those DLLs will make the projects work. after inserting those references, rebuild your projects and this should take care of restoring the nugget packages for the other dlls to load.

thank you for reading me,
steve chassé

2020-dec-25-
OCULUS RIFT CV1 only - NOT WORKING YET FOR ELITE DANGEROUS AS I HAVE A HARD TIME MAKING THE STEAMVR OVERLAY WORK - as per advertised here https://forums.frontier.co.uk/threads/virtual-desktop-program-with-embedded-physics-engine-at-the-press-of-a-button-coming-in-2020.542577/#post-8514927 . currently the keyboard is not working and the microsoft windows voice recognition stopped working. I have coded this program and used as reference 
many sources from the internet and normally i try to reference it all when i program.

# SCCoreSystems-rerelease

you have a 60 days trial usuing the Ab3d.DXEngine nuggets. what i didn't code, i provide the link to the reference of stackoverflow and elsewhere on programming forums. no functionnal virtual reality keyboard yet though but sccsVD4ED and sccsVD4VE should have a working mouse for the virtual desktop. i have put up to date the references and nuggets. 

the username is "9" and the password is "std" for standard. 

List of programs:

sccsv10 - 1st version
<img src="https://i.ibb.co/LJP0C2W/sccsv10.png" alt="sccsv10" border="0">

sccsv11 - 2nd version
<img src="https://i.ibb.co/c66WLyn/sccsv11.png" alt="sccsv11" border="0">

sc_core_systems - 3rd version. console dotnet directx virtual reality. currently it is not updated with the few modifications i made in sccsVD4ED.

sccsVD4ED - 3rd version, using wpf. for elite dangerous - the steamVR overlay doesn't work properly yet unfortunately so i cannot have my virtual desktop screen sent to the steamVR overlay c# scripts. So this program doesn't even work with elite dangerous yet.
<img src="https://i.ibb.co/chZnvY2/sccs-VD4-ED.png" alt="sccs-VD4-ED" border="0">

sccsVD4VE - 3rd version. using wpf. for void expanse. not updated yet with sccsVD4ED tiny changes.

Jitter - the original physics engine.

sc_message_object - i created this class because it was the only way i had found to have a stable multithreaded application where everything worked and that this single object can be passed to all threads inside of the application whether you have threads or background workers or tasks.

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
and if i can make my c# programs for windows 10 work on ubuntu dotnet, it would become something even greater and it would be without the oculus headset. but some people at http://www.openhmd.net/index.php/devices/ provide a ubuntu solution.

thank you for reading me
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
