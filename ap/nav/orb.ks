
GLOBAL AP_NAV_ORB_ENABLED IS TRUE.
local PARAM is get_param(readJson("param.json"), "AP_NAV_ORB", lexicon()).

local mannode_maneuver_time is 0.
local min_burn_dv is 0.01.
local thrust_vector is V(0,0,1).
local thrust_string is "x".
// is a vector in the current ship:facing frame
// note that controlpart alters ship:facing frame

local function update_burn_info {

    if defined AP_ORB_ENABLED and ISACTIVEVESSEL and HASNODE {
        set mannode_maneuver_time to ap_orb_maneuver_time(NEXTNODE:deltav,thrust_vector).
    } else {
        set mannode_maneuver_time to 0.
    }

    if defined AP_NAV_ORB_ENABLED {
    }
    
    local delta is ship:control:pilottranslation.
    if (delta):mag > 0.5 and (delta-thrust_vector):mag > 0.5 {
        set thrust_vector to delta:normalized.
        set thrust_string to "" + 
            (choose char(8592) if delta:x < -0.5 else "") +
            (choose char(8594) if delta:x > 0.5 else "") +
            (choose char(8595) if delta:y < -0.5 else "") +
            (choose char(8593) if delta:y > 0.5 else "") +
            (choose "o" if delta:z < -0.5 else "") +
            (choose "x" if delta:z > 0.5 else "").
    }
}


local last_stick_time is -1.
local last_stick_roll is 0.
local last_desired_attitude is ship:facing.
local relative_roll is 0.
local slow_roll is 0.
function ap_nav_orb_stick {
    parameter u0 is SHIP:CONTROL:PILOTMAINTHROTTLE. // throttle input
    parameter u1 is SHIP:CONTROL:PILOTPITCH. // pitch input
    parameter u2 is SHIP:CONTROL:PILOTYAW. // yaw input
    parameter u3 is SHIP:CONTROL:PILOTROLL. // roll input

    update_burn_info().

    set last_stick_roll to  last_stick_roll + u3/10.
    set last_stick_time to time:seconds.

    local new_roll is relative_roll.
    local delta_slow is sign(deadzone(u3,0.5)).
    set last_stick_roll to last_stick_roll+0.25*delta_slow.
    set new_roll to 15*round_dec(slow_roll,0).

    if new_roll <> relative_roll {
        set relative_roll to new_roll.
        print "tar roll " + relative_roll.
    }

    if ship:facing:forevector*ship:velocity:orbit:normalized > 0.8 {
        set last_desired_attitude to ship:prograde.
    } else if ship:facing:forevector*ship:velocity:orbit:normalized < -0.8 {
        set last_desired_attitude to ship:retrograde.
    } else if ship:facing:forevector*ship:up:forevector > 0.8 {
        set last_desired_attitude to ship:up.
    } else if ship:facing:forevector*ship:up:forevector < -0.8 {
        set last_desired_attitude to ship:up*R(-180,0,0).
    }

    local omega is -1*vcrs(last_desired_attitude:forevector, ship:facing:forevector).
    local omega_mag is vectorAngle(last_desired_attitude:forevector, ship:facing*thrust_vector).
    set AP_NAV_ATT to angleaxis( omega_mag, omega:normalized )*ship:facing*R(0,0,-relative_roll).

    set AP_NAV_VEL to ship:velocity:orbit.

    return true.
}

// function that sets nav parameters to execute present/future nodes
function ap_nav_orb_mannode {
    update_burn_info().

    local steer_time is 10. // get from orb if possible
    local buffer_time is 1.
    local no_steer_dv is 0.1.
    local no_burn_dv is 0.01.

    if ISACTIVEVESSEL and HASNODE {
        local mannode_delta_v is NEXTNODE:deltav:mag.
        if defined AP_ORB_ENABLED {
            set steer_time to ap_orb_steer_time(NEXTNODE:deltav).
            set no_steer_dv to ap_orb_rcs_dv().
            set no_burn_dv to ap_orb_min_dv(thrust_vector).
        }

        set AP_NAV_ACC to V(0,0,0).
        if NEXTNODE:eta < mannode_maneuver_time/2 + buffer_time {
            if mannode_delta_v < no_burn_dv {
                print "remaining node " + char(916) + "v " + round_fig(mannode_delta_v,3) + " m/s".
                set mannode_maneuver_time to 0.
                set AP_NAV_VEL to ship:velocity:orbit.
                set AP_NAV_ATT to ship:facing.
                REMOVE NEXTNODE.
            } else {
                // do burn
                set AP_NAV_VEL to ship:velocity:orbit + NEXTNODE:deltav.
                if NEXTNODE:deltav:mag > no_steer_dv {
                    local omega is -1*vcrs(NEXTNODE:deltav:normalized, ship:facing*thrust_vector).
                    local omega_mag is vectorAngle(NEXTNODE:deltav:normalized, ship:facing*thrust_vector).
                    set AP_NAV_ATT to angleaxis( omega_mag, omega:normalized )*ship:facing.
                }
            }
        } else if NEXTNODE:eta < mannode_maneuver_time/2 + buffer_time + steer_time {
            // steer to burn direction
            if not (kuniverse:timewarp:rate = 0) {
                set kuniverse:timewarp:rate to 0.
            }
            set AP_NAV_VEL to ship:velocity:orbit.
            local omega is -1*vcrs(NEXTNODE:deltav:normalized, ship:facing*thrust_vector).
            local omega_mag is vectorAngle(NEXTNODE:deltav:normalized, ship:facing*thrust_vector).
            set AP_NAV_ATT to angleaxis( omega_mag, omega:normalized )*ship:facing.
        } else {
            // do nothing
            set AP_NAV_VEL to ship:velocity:orbit.
            set AP_NAV_ATT to ship:facing.
            return false.
        }
        
        return true.

    } else if ISACTIVEVESSEL {
        return false.
    } else {
        set mannode_maneuver_time to 0.
        return false.
    }
}

function ap_nav_orb_status_string {
    local dstr is "".
    local mode_str is "".
    local vel_mag is ap_nav_get_hud_vel():mag.

    if mannode_maneuver_time <> 0 {
        set dstr to char(10) + char(916) + "v " +round_fig((AP_NAV_VEL-ship:velocity:orbit):mag,2)
            + "|" + thrust_string + round_fig(mannode_maneuver_time,2) + "s " +
            + (choose char(10) + "T " + round_fig(-NEXTNODE:eta,2) + "s" if HASNODE else "").
    }
    set DISPLAY_ORB to false.
    set mode_str to mode_str + "o".
    set dstr to dstr + (choose "" if mode_str = "" else char(10)+mode_str).

    return dstr.
}
