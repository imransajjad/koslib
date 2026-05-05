# koslib

A library of autopilot scripts for Kerbal Space Program written with [kOS](https://github.com/KSP-KOS/KOS). kOS is a scripting language and overall an amazing mod for Kerbal Space Program.

Some videos:
[Automated Docking](https://www.youtube.com/watch?v=XqpHnunEl00) and [Automated Landing](https://www.youtube.com/watch?v=7-GHF_yZtzs)

# Setup

Kerbal Space Program and kOS is required. The setup builds upon the [boot file](https://ksp-kos.github.io/KOS/general/boot.html#boot) setup in KOS. Inside the `Script` directory, there should be a boot directory. These files are selectable when building spacecraft as boot files for a kOS core.

`koslib` should live alongside the `Script/boot/` directory as shown below

```
Script/
    boot/
    koslib/
    logs/
    param/
    term-scripts/
```

Then from inside the boot directory, scripts can load files from the `koslib` path and run them. The boot script will need a few sections.

## Example boot file: aero-bare-flcs.ks

The following is one of the many complete examples of a runnable boot script in the `koslib/script-level/boot/` directory.

**Important:** Copy the [`aero-bare-flcs.ks`](script-level/boot/aero-bare-flcs.ks) file into the `Script/boot/` directory and boot up a core with this file.

### Step 1: Load Resources from Base

This code is a little verbose but it allows loading parameters from the homebase everytime the kOS processor is restarted. Very useful for testing changes.

```
// generic atmospheric flight control computer

wait until ship:loaded.

global DEV_FLAG is true.
global FETCH_SOURCE is (DEV_FLAG or not exists("param.json")) and HOMECONNECTION:ISCONNECTED.
if FETCH_SOURCE { print "fetching resources from base".}

function fetch_and_run {
    parameter filehomepath.

    local filepath is filehomepath:replace("0:/", "").
    if FETCH_SOURCE {
        copypath(filehomepath, filepath).
    }
    if filepath:contains(".ks") {
        runoncepath(filepath).
    }
}

fetch_and_run("0:/koslib/util/common.ks").
if FETCH_SOURCE {
    get_element_param_file("0:/param").
}

fetch_and_run("0:/koslib/util/wp.ks").
fetch_and_run("0:/koslib/util/shbus.ks").
fetch_and_run("0:/koslib/util/phys.ks").

fetch_and_run("0:/koslib/ap/stick.ks").
fetch_and_run("0:/koslib/ap/aero-w.ks").
```

### Step 2: The Main Loop

Once all the resources are loaded, we can run the functions provided by the files in the main loop.

```
GLOBAL BOOT_AERO_BARE_FLCS_ENABLED IS true.

// main loop
UNTIL false {
    get_plane_globals().
    util_shbus_rx_msg().
    ap_aero_w_do().
    wait 0.
}
```

## Setup with Multiple Cores

The [`koslib/util/shbus.ks`](util/shbus.ks) file implements a structured method to implement communication between two or more cores or ships. kOS has a limitation where if a core is running a loop that is piloting the spacecraft, there are very few good ways to get user input. This library gets around that problem by implementing another core which handles user input only.

1. Load one core with a `koslib/boot/*.flcs.ks` boot file and label the core `flcs`. This will be the autopilot core. It will check for new messages at the start of every iteration of the main loop.
2. Load another core on the same ship with the [`koslib/boot/flcom.ks`](boot/flcom.ks) file and label the core `flcom`. This will be the command core and the kOS terminal will be running an interactive shell with a text to command parser. Type `help` in this shell.



## The Other Directories

The `Script/param` directory can contain kOS serializable .json files that contain configuration parameters for each module. For instance, the `aero_w_do()` function from the [`koslib/ap/aero-w.ks`](ap/aero-w.ks) has three PIDs for pitch, yaw and roll. The code to load the parameters is usually at the top of a file. The examples in `koslib/script-level/param` can be copied into this directory.

If your ship is named "Docker Kerbal Tester", the code will try to find a file called `Script/param/Docker Kerbal Tester.json`. There are some other rules fore finding a parameter file based on core and element name as well.

Here is a fragment of how the .json file would look to configure those parameters.

```
{
    "entries": [
        "AP_STICK",
        {
            #...
        },
        "AP_AERO_W",
        {
            "entries": [
                "GLIM_VERT", 12,
                "MAX_ROLL", 360,

                "PR_KP",1.0,
                "PR_KI",1.3,
                "PR_KD",0.05,
                "YR_KP",6.0,
                "YR_KI",0.0,
                "YR_KD",0.1,
                "RR_KP",0.25,
                "RR_KI",0.20,
                "RR_KD",0.005,
            ],
            "$type": "kOS.Safe.Encapsulation.Lexicon"
        },
        "UTIL_HUD",
        {
           #...
        },
    ]
}
```

`Script/log` will contain logs from the file `koslib/util/fldr.ks`.

`Script/temp-scripts` contains scripts that the file `koslib/util/term.ks` can run with its meta scripting ability. This is very useful when sending sequences of messages to other cores or ships.

`Script/helpers` contains some useful python scripts. For instance `parse_logs.py` allows plotting data recorded using the `fldr.ks` file.

**Pro Tip:** creating symbolic links to the `koslib/script-level/boot` and `koslib/script-level/param` directory from the `Script` directory will give access to all the files in those directories as if they were copied.

# Code Structure

The library code is divided into the `ap` directory and the `util` directory. The salient features of both are as follows.

1. `ap` is short for autopilot. It runs main control loops and navigation.
2. `ap` code is more parameterizable per ship.
3. `ap` files do not have `shbus` functionality. They interface with `util` files.
4. `util` is short for utility. It has everything except autopilot code.
5. `util` code is less parameterizable.
6. `util` files can have `shbus` functionality which means they can send and receive messages between cores and ships.

# Design Goals

Kerbal Space Program and kOS both are great sources of learning especially for a non science or engineering audience. This is also a goal of this project. But there are some goals which may seem contrary to so-called good coding practices so these are mentioned first.

## Recommended Bad Practices

1. directory and file names are kept short and abbreviated. The goal of this library is to **not scale** and work as a compact project.
2. Code should be readable without comments and with a little effort. This little effort is encouraged. But comments and docs should be present where they are absolutely needed.
3. Code should rely on kOS types especially directions and vectors **even if slower or less readable** than using some clever or more common sin or cosine formulas.

## Handling State

Another design goal is to write robust and reliable code. The following goals are stated with the most outlandish situations in mind for example what if we need to reset the spacecraft **during** a burn. Even in that case everything should simply just work.

1. The autopilot core should be rebootable from anywhere.
2. Prefer stateless, functional code. For instance, a persistent variable `in_burn` that is set to true at the start of a burn is **not recomended**. Prefer something that is run everytime like:
```
    set in_burn to (time:seconds > nextnode:eta - ap_me_get_burn_time(nextnode:deltav)/2 and time:seconds < nextnode:eta + ap_me_get_burn_time(nextnode:deltav)/2).

    or

    set in_burn to (time:seconds > nextnode:eta - ap_me_get_burn_time(nextnode:deltav:mag)/2 and nextnode:deltav:mag > MIN_BURN_DV).

```
3. If code cannot be made stateless, then prefer state that is stable or converges to good values. For instance, `koslib/util/phys.ks` has an estimate for drag that converges to a good value.
4. If a state cannot be made to converge to a known value, commit it to a file. And load the variable from that file upon next bootup. For instance, if the user set the hud color to blue, save its RGB value to a file.
5. Caching done well is good. For example, `koslib/ap/me.ks` has a function to calculate engine parameters but it caches the result so that heavy calculation is avoided in every iteration. But the cache is also invalidated under many conditions.
6. Throttling for event like functionality is good. For instance, this code only sends the departure event once. If a reboot happens, the event is sent again but that is acceptable (sending an event twice is not the worst thing in the universe).
```
local departure is false.
function ap_aero_w_status_string {
    ...
    if abs(alpha) > 45 and not departure {
        util_fldr_send_event("aero_w departure").
        print "aero_w departure".
        set departure to true.
    } else if abs(alpha) < 20 and departure {
        set departure to false.
    }
    ...
}
