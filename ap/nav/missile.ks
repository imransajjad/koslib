
GLOBAL AP_NAV_MISSILE_ENABLED IS TRUE.

local mother_ship is ship.

local last_acc_a is V(0,0,0).
local last_vel_a is V(0,0,0).
local last_time_a is time:seconds-0.02.
local last_ship_status is "LANDED".

local function get_target_acc {
    parameter this_vessel.
    if time:seconds-last_time_a > 0.0 {
        local new_vel is V(0,0,0).

        // if not this_vessel:loaded and
        if (this_vessel:status = "LANDED" or this_vessel:status = "SPLASHED") {
            set new_vel to this_vessel:geoposition:altitudevelocity(this_vessel:altitude):orbit.
        } else {
            set new_vel to this_vessel:velocity:orbit.
        }

        set last_acc_a to 0.5*last_acc_a + 0.5*(new_vel - last_vel_a)/(time:seconds - last_time_a).
        set last_vel_a to new_vel.
        set last_time_a to time:seconds.
    }
    return last_acc_a.
}

local function newton_one_step_intercept {
    parameter t.
    parameter init_pos.
    parameter init_vel.
    parameter init_acc.
    parameter dv_C. // available delta v

    if dv_C > 0.0 {
        // minimize t
        local pos is init_pos + init_vel*t + 0.5*init_acc*t*t.
        local vel is init_vel + init_acc*t.
        local acc is init_acc.

        local f is pos*pos - dv_C*pos:mag*t.
        local df is 2*pos*vel - dv_C*pos:mag - dv_C*t*pos*vel/pos:mag.

        return t -f/df.
    } else {
        // minimize delta v
        local rt is init_pos/t + init_vel + 0.5*init_acc*t.
        local dr is -init_pos/(t^2) + 0.5*init_acc.
        local ddr is 2*init_pos/(t^3).

        local df is dr*rt.
        local ddf is ddr*rt + dr*dr.

        return t - df/ddf.
    }
}


local INTERCEPT_TIME is 10.
local INTERCEPT_POS is V(0,0,0).
local t_last is time:seconds.
local t_print is time:seconds.

local function missile_intercept {
    parameter target_ship.
    parameter available_dv.

    local target_acc is get_target_acc(target_ship).
    local target_vel is ap_nav_get_vessel_vel(target_ship).
    local pos is ship:position - target_ship:position.
    local vel is ap_nav_get_vessel_vel() - target_vel.
    local acc is GRAV_ACC - 0*get_pre_aero_acc() - target_acc.

    set INTERCEPT_TIME to INTERCEPT_TIME - (time:seconds - t_last).
    set INTERCEPT_TIME to newton_one_step_intercept(INTERCEPT_TIME, pos, vel, acc, available_dv).
    set t_last to time:seconds.

    local dv_r is -(vel + pos/INTERCEPT_TIME + 0.5*acc*INTERCEPT_TIME).
    set INTERCEPT_POS to pos + vel*INTERCEPT_TIME + 0.5*acc*INTERCEPT_TIME*INTERCEPT_TIME.

    if INTERCEPT_TIME > -0.1 and INTERCEPT_TIME < 12.5 {
        // this line increases weight of error as intercept becomes closer
        // may want to figure out how to do this with gains
        set dv_r to convex(20.0,1.0,INTERCEPT_TIME/12.5)*dv_r.
    }
    if time:seconds - t_print > 0.25 {
        set t_print to time:seconds.
        local lateral_vec is vectorexclude(V(0,0,1), (-ship:facing)*INTERCEPT_POS):mag.
        if (false) {
            // debug print
            local print_str is  "" +
                    "C " + round_dec(available_dv,1) + " m/s" + char(10) +
                    "dp " + round_dec(lateral_vec,1) + "/" + round_dec(INTERCEPT_POS:mag,1) + " m" + char(10) +
                    "dv " + round_vec((-ship:facing)*dv_r,1) + " " + round_dec(dv_r:mag,1) + " m/s" + char(10) +
                    "acc " + round_vec((-ship:facing)*acc,1) + " " + round_dec(acc:mag,1) + " m/s2" + char(10) +
                    "t " + round_dec(INTERCEPT_TIME,1) + " s" + char(10) +
                    (choose "o" if AP_NAV_IN_ORBIT else "") +
                    (choose "s" if AP_NAV_IN_SURFACE else "") + char(10).
            ap_nav_missile_guide_hud_print(print_str).
        } else {
            // normal print
            local print_str is "d+ " + round_dec(lateral_vec,1) + char(10) +
                            "do " + round_dec(INTERCEPT_POS:mag,1) + char(10) +
                            "t " + round_dec(INTERCEPT_TIME,1) + " s".
            ap_nav_missile_guide_hud_print(print_str).
        }
    }
    return dv_r.
}


function ap_nav_missile_guide {

    local launched is true.
    set target_ship to -1.
    if defined UTIL_SHSYS_ENABLED {
        set target_ship to util_shsys_get_target().
        set launched to util_shsys_check().
    } else if HASTARGET {
        set target_ship to TARGET.
    }

    local available_dv is 0.
    if defined UTIL_SHSYS_ENABLED and not launched {
        set available_dv to 600.
    } else if defined AP_ORB_ENABLED {
        set available_dv to available_dv + ap_me_get_dv():z.
    }

    if launched and (mother_ship:position:mag < 8.0) {
        print "SEPARATING: " + round_dec(mother_ship:position:mag,1).
        set AP_NAV_VEL to ap_nav_get_vessel_vel() -(0.25*ship:airspeed)*(mother_ship:facing:topvector).
        set AP_NAV_ACC to GRAV_ACC.
        set AP_NAV_ATT to ship:facing.
        ap_nav_missile_guide_cleanup().
        if mother_ship:position:mag < 4.0 {
            ap_me_limit_set(20).
        } else {
            ap_me_limit_set(100).
        }.
        return true.
    } else if not (target_ship = -1) and launched {
        set AP_NAV_VEL to ap_nav_get_vessel_vel() + missile_intercept(target_ship, available_dv).
        set AP_NAV_ACC to GRAV_ACC.
        set AP_NAV_ATT to ship:facing.
        return true.
    } else if not (target_ship = -1) and not launched {
        missile_intercept(target_ship, available_dv). // just display
        set AP_NAV_VEL to ap_nav_get_vessel_vel().
        set AP_NAV_ACC to V(0,0,0).
        set AP_NAV_ATT to ship:facing.
        return true.
    } else  {
        set AP_NAV_VEL to ap_nav_get_vessel_vel().
        set AP_NAV_ACC to V(0,0,0).
        set AP_NAV_ATT to ship:facing.
        ap_nav_missile_guide_cleanup().
        set INTERCEPT_TIME to 10.
        return false.
    }
}

local hud_key is "missile_int".
local hud_printed is true.
local function ap_nav_missile_guide_hud_print {
    parameter print_str.

    local local_print is true.
    if defined UTIL_SHSYS_ENABLED and not util_shsys_check() {
        set local_print to false.
    }
    if local_print and defined UTIL_HUD_ENABLED {
        util_hud_push_left(hud_key, print_str).
    } else if not local_print and defined UTIL_SHBUS_ENABLED {
        util_shbus_tx_msg("HUD_PUSHL", list(hud_key, print_str)).
    } else {
        print print_str.
    }
    set hud_printed to true.
}

function ap_nav_missile_guide_cleanup {
     if hud_printed {
        if defined UTIL_HUD_ENABLED {
            util_hud_pop_left(hud_key).
        }
        if defined UTIL_SHBUS_ENABLED {
            util_shbus_tx_msg("HUD_POPL", list(hud_key)).
        }
        set hud_printed to false.
    }
}

function ap_nav_missile_status_string {
    local dstr is "".
    local vel_mag is ap_nav_get_hud_vel():mag.
    local lateral_vec is vectorexclude(V(0,0,1), (-ship:facing)*INTERCEPT_POS):mag.
    set dstr to dstr + "/"+round_fig(vel_mag,2) + char(10) +
        "d+ " + round_dec(lateral_vec,1) + char(10) +
        "do " + round_dec(INTERCEPT_POS:mag,1) + char(10) +
        "t " + round_dec(INTERCEPT_TIME,1) + " s".
    return dstr.
}
