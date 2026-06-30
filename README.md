### About this branch:
This is the visualization sector of the SITL simulation environment that accepts the interceptor states, and generate target dynamics, while capturing camera to send it to YOLO models do Detection and Track. 

Goes in this branch: 
Interceptor FDM states 

Goes out the branch: 
camera sensor output,
dynamics and trajectory of target drones, 
bounding boxes from the YOLO


### Demo: 
NA 


### Requirments before doing the setup:
1) Python 3.12 and C languages
2) Visual Studio 2022
3) VSCode --> for runing this repo
4) Have a Windows OS and a Linux OS (if no native Linux, try WSL or VMWare) --> you need to clone this repo in both of these OSs and open it in VSCode
5) Epic Games Launcher 
6) check the "kernel/__init__.py" file to make sure the PCs' addresses are compatible with your workstations/OSs



### Setup on Windows OS: 

## Unreal Engine + Project Airsim
1) Installed Unreal Engine 5.7 from Epic Games Launcher in (C:\Program Files\Epic Games): https://dev.epicgames.com/documentation/unreal-engine/install-unreal-engine 
 
2) Install Cesium following this link up to step 4: https://cesium.com/learn/unreal/unreal-quickstart/ 
    The location of cesium folder that needs to be downloaded should be here: C:\Program Files\Epic Games\UE_5.7\Engine\Plugins\Marketplace
    
3) Add 3d google map world: https://cesium.com/learn/unreal/unreal-photorealistic-3d-tiles/

4) Clone modified project airsim repo on C drive (optimal performance): 

    once cloned, open Blocks.sln with Visual Studio and set the Solution Configuration to [Development Editor] [Win64] 
    Then do Ctril + F5 -- this will build and launch Unreal

5) Once Unreal opened, go to Content Browser > Blocks > C++ classes > Public and then drag Actor_SetWorldOrigin to the viewport. 
            
6) Press play (the green button)

## optional (if you want to drive the interceptor around)
7) Go to the branch named and clone that in Ubuntu Environment: physic-source

8) Make sure to run that branch's scheduler before runing the visual-source branch

#[for more info go to the physic-branch]


## Open this branch in VSCode  

9) create a venv and activate: 
python -m venv .venv
.\.venv\Scripts\Activate.ps1

10) Other dependecies: 
python -m pip install --upgrade pip 
pip install -e client/python/projectairsim
pip install -r requirements_win.txt

11) run the scheduler to use the repo: 
py -m kernel.scheduler  



### Other notes
# Project Airsim
- The official stable release (v0.1.1) is distributed as a prebuilt Unreal environment.
- There is no separate "stable source branch", but the prebuilt package is built from commit 9fe8d3a.
- This project uses the SAME source commit (v0.1.1 / 9fe8d3a), assuming it is stable since it was used to create the official prebuilt release.
- Instead of using the prebuilt binaries, we clone the source and build locally to allow customization.
- The only customized part is the Blocks folder: C:\ProjectAirSim\unreal\Blocks

# ProjectAirSim\unreal\Blocks
- The ProjectAirSim repository is based on v0.1.1 / commit 9fe8d3a. However, the Unreal Blocks folder was originally taken from an untracked main-branch state and was customized before the repository was reset/checked out to v0.1.1.
- Because of this, the exact original commit for unreal/Blocks is unknown. The current Blocks folder should be treated as a custom local version, not a clean v0.1.1 copy.