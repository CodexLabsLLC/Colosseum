# Build Colosseum on Linux

The current recommended and tested environment is **Ubuntu 24.04 LTS**. Theoretically, you can build on other distros as well, but we haven't tested it.

We've two options - you can either build inside docker containers or your host machine.

## Docker

Please see instructions [here](docker_ubuntu.md)

## Host machine

### Pre-build Setup

#### Build Unreal Engine

- Make sure you are [registered with Epic Games](https://docs.unrealengine.com/en-US/SharingAndReleasing/Linux/BeginnerLinuxDeveloper/SettingUpAnUnrealWorkflow/index.html). This is required to get source code access for Unreal Engine.

- Clone Unreal in your favorite folder and build it (this may take a while!). **Note**: We only support Unreal >= 5.0 at present. We recommend using 5.4 or 5.5.

```bash
# go to the folder where you clone GitHub projects
git clone -b 5.4 git@github.com:EpicGames/UnrealEngine.git
cd UnrealEngine
./Setup.sh
./GenerateProjectFiles.sh
make
```

### Build Colosseum

- Clone Colosseum and build it:

```bash
# go to the folder where you clone GitHub projects
git clone https://github.com/CodexLabsLLC/Colosseum.git
cd Colosseum
```

By default Colosseum uses clang 18 to build for compatibility with UE 5.4+. The setup script will install the right version of cmake, llvm, and eigen.

```bash
./setup.sh
./build.sh
# use ./build.sh --debug to build in debug mode
```

### Build Unreal Environment

Finally, you will need an Unreal project that hosts the environment for your vehicles. Colosseum comes with a built-in "Blocks Environment" which you can use, or you can create your own. Please see [setting up Unreal Environment](unreal_proj.md) if you'd like to setup your own environment.

## How to Use Colosseum

Once Colosseum is setup:

- Go to `UnrealEngine` installation folder and start Unreal by running `./Engine/Binaries/Linux/UnrealEditor`.
- When Unreal Engine prompts for opening or creating project, select Browse and choose `Colosseum/Unreal/Environments/BlocksV2` (or your [custom](unreal_custenv.md) Unreal project).
- Alternatively, the project file can be passed as a commandline argument. For Blocks: `./Engine/Binaries/Linux/UnrealEditor <Colosseum_path>/Unreal/Environments/Blocks/BlocksV2.uproject`
- If you get prompts to convert project, look for More Options or Convert-In-Place option. If you get prompted to build, choose Yes. If you get prompted to disable Colosseum plugin, choose No.
- After Unreal Editor loads, press Play button.

See [Using APIs](apis.md) and [settings.json](settings.md) for various options available for Colosseum usage.

!!! tip
Go to 'Edit->Editor Preferences', in the 'Search' box type 'CPU' and ensure that the 'Use Less CPU when in Background' is unchecked.

### [Optional] Setup Remote Control (Multirotor Only)

A remote control is required if you want to fly manually. See the [remote control setup](remote_control.md) for more details.

Alternatively, you can use [APIs](apis.md) for programmatic control or use the so-called [Computer Vision mode](image_apis.md) to move around using the keyboard.
