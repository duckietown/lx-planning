<p align="center">
<a href="https://duckietown.com"><img src="./assets/images/dtlogo.png" alt="Logo and watermark of Duckietown, a robotics company" width="50%"></a>
</p>

# **Learning Experience (LX): Planning**

Find the most up-to-date instructions on [how to run LXs on the Duckietown manual](https://docs.duckietown.com/ente/duckietown-manual/60-learning-experiences/lx-general-procedure.html). 

# About these activities

In this learning experience, you will build a robot planner designed to have a Duckiebot drive in a cluttered environment. We will represent the environment by a set of obstacles (in our simplified case they will be circular and rectangular). The first task will be to figure out how to tell if a robot configuration (pose) collides with an obstacle. From there you can build a planner based on the randomly exploring random trees (RRT) algorithm to find a path from a start configuration to a goal configuration without colliding with anything. 

This learning experience is provided by the Duckietown team and can be run on Duckiebots. Visit us at the 
[Duckietown Website](https://www.duckietown.com) for more learning materials, documentation, and demos.

For guided setup instructions, lecture content, and more related to this LX, see our [Self-Driving Cars with Duckietown](https://duckietown.com/self-driving-cars-with-duckietown-mooc/).

**(If not already done) Clone this repository**

The recommended way to use this repository is to make a fork and then clone that fork. 

This can be done through the GitHub web interface. However, you are also free to simply clone this repository and get started. 

Example instructions to fork a repository and configure to pull from upstream can be found in the [duckietown-lx repository README](https://github.com/duckietown/duckietown-lx/blob/mooc2022/README.md).

# Instructions

**NOTE:** All commands below are intended to be executed from the root directory of this exercise (i.e., the directory containing this README).


## 1. Make sure your LX is up-to-date

Update your LX definition and instructions,

    git remote add upstream git@github.com:duckietown/lx-planning 
    git pull upstream ente


- 💻 This is an `ente` learning experience (note the branch name). Make sure your Duckietown Shell is set to an `ente` profile 

- (and not, e.g., a `daffy` one). You can check your current distribution with

    dts profile list

  To switch to an ente profile, follow the [Duckietown Manual DTS installation instructions](https://docs.duckietown.com/ente/duckietown-manual/10-setup/02-software/duckietown-shell-dts-installation.html#dt-account-switch-profile).


- 💻 Always make sure your Duckietown Shell is updated to the latest version. See [installation instructions](https://github.com/duckietown/duckietown-shell)

- 💻 Update the shell commands: `dts update`

- 💻 Update your laptop/desktop: `dts desktop update`

- 🚙 Update your Duckiebot: `dts duckiebot update ROBOTNAME` (where `ROBOTNAME` is the name of your Duckiebot - real or virtual.)

**Note**: if your virtual robot hangs indefinitely when you try to update it, you can try to restart it with:

    dts duckiebot virtual restart ROBOTNAME


## 3. Work on the exercise

### Launch the code editor

#### SSL certificate

If you have not done so already, set up your local SSL certificate needed to run the learning experience editor with:

    sudo apt install libnss3-tools
    dts setup mkcert

Open the code editor by running the following command,

```
dts code editor [--bind 0.0.0.0]
```

Where the `--bind` flag can be used if using a Duckietown Workspace and the browser is not automatically opening this document. Wait for a URL to appear on the terminal, then click on it or copy-paste it in the address bar of your browser to access the code editor. The first thing you will see in the code editor is this same document, you can continue there.

**NOTE**: if you are running Duckietown inside a [Duckietown Workspace](https://docs.duckietown.com/ente/duckietown-manual/10-setup/00-computer/setup-duckietown-workspace.html), make sure to [install the certificate for your host machine as well](https://docs.duckietown.com/ente/duckietown-manual/10-setup/00-computer/setup-duckietown-workspace.html#running-dts-code-editor). 


### Walkthrough of notebooks

**NOTE**: You should be reading this from inside the code editor in your browser.

Inside the code editor, use the navigator sidebar on the left-hand side to navigate to the `notebooks` directory and open the first notebook.

Follow the instructions on the notebook and work through the notebooks in sequence.


### Testing with the Duckiematrix

To test your code in the Duckiematrix you will need a virtual robot. You can start a previously created one, or create a new one with the command:

```
dts duckiebot virtual create --type duckiebot --configuration DB21J VBOT
```

where `VBOT` is the hostname. It can be anything you like, with [some constraints](https://docs.duckietown.com/ente/duckietown-manual/10-setup/03-duckiebot/flashing-sd-card-duckiebot-initialization-complete.html). Make sure to remember your robot (host)name for later.

Then you can start your virtual robot with the command:

```
dts duckiebot virtual start VBOT
```

You should see it with a status `Booting` and finally `Ready` if you look at `dts fleet discover`: 

```
     | Hardware |   Type    | Model |  Status  | Hostname 
---  | -------- | --------- | ----- | -------- | ---------
[VBOT] |  virtual | duckiebot | DB21J |  Ready   | [VBOT].local
```

Now that your virtual robot is ready, you can start the Duckiematrix. From a terminal in this exercise directory that you cloned do:

```
dts code start_matrix [--no-renderer]
```

Where the `--no-renderer` option is to be used if running the command inside a Duckietown Workspace. In this case, to connect the renderer then open a terminal on the host machine, where you have previously installed dts, and run `dts matrix run`.  

You should see the Unity-based Duckiematrix simulator start up. The startup screen will look like:

![duckiematrix_start](assets/images/planning-lx-dtmatrix-splashscreen.jpg)

Your Duckiebot is at the corner of an unusual map with signs and missing tiles.

![duckiematrix_start_top](assets/images/planning-lx-dtmatrix-splashscreen-topview.jpg)

Remember that to activate the Duckiematrix window, click anywhere on it and press [ENTER]. You can then at any time press `ESC` to disengage the mouse. 

From here you can move the duckie towards the Duckiebot with the 'w', 'a', 's', and 'd' keys, or you can move the camera angle to view the Duckiebot with the mouse. If you are close enough to your Duckiebot, you can jump on with the 'E' key. You can then drive the Duckiebot around with the 'w', 'a', 's', and 'd' keys. All available keyboard commands are summarized in the "Settings" tab at the bottom left of the Duckiematrix window. 

### Building your code

After going through the notebooks, at any time you can compile your code in a Docker image with:

```
dts code build -R ROBOT_NAME 
```

### 💻 Testing 

Test your code on the virtual Duckiebot in the Duckiematrix:

```
dts code workbench -m -R [VIRTUAL_ROBOT_NAME]
```
<!--
To test it on a physical Duckiebot instead:

```
dts code workbench -R [ROBOT_NAME]
```

In another terminal, you can launch the `noVNC` viewer for this LX and open RViz. 

```
dts code vnc -R [ROBOT_NAME]
```

where `[ROBOT_NAME]` could be the real or the virtual robot (use whichever you ran the `dts code workbench` and `dts code build` command with).
-->

**Note**: Between each test you will probably want to reset the Duckiebot to its start location. If your duckie is riding the Duckiebot you can do so by making the simulator window active and then pressing `R` for Reset. 

Now you can proceed to the [first notebook](./notebooks/01-Collision-Checker/collision_checker.ipynb).
