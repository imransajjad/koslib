
GLOBAL AP_NAV_MISSILE_ENABLED IS TRUE.

local PARAM is get_param(readJson("param.json"), "AP_NAV_MISSILE", lexicon()).

local START_DV is get_param(PARAM, "START_DV", 990.0).
local SAFE_SEPARATION is get_param(PARAM, "SAFE_SEPARATION", 6.0).

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
        } else if (this_vessel:status = "ORBITING") {
            set last_vel_a to this_vessel:velocity:orbit.
            set last_acc_a to this_vessel:body:mu/(this_vessel:body:position:mag^2)*(this_vessel:body:position:normalized).
            set last_time_a to time:seconds.
            return last_acc_a.
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
    parameter burn_penalty_time is 0. // a term dependent on remaining burn time and mass fraction

    if dv_C > 0.0 {
        // minimize t
        local pos is init_pos + init_vel*t + 0.5*init_acc*t*t.
        local vel is init_vel + init_acc*t.
        local acc is init_acc.

        local f is pos*pos - dv_C*pos:mag*(t-burn_penalty_time).
        local df is 2*pos*vel - dv_C*pos:mag - dv_C*(t-burn_penalty_time)*pos*vel/pos:mag.

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
local INTERCEPT_POS is V(0,0,0). // in ship frame
local t_last is time:seconds.


local function missile_intercept {
    parameter target_ship.
    parameter available_dv.

    local target_acc is get_target_acc(target_ship).
    local target_vel is ap_nav_get_vessel_vel(target_ship).
    local pos is ship:position - target_ship:position.
    local vel is ap_nav_get_vessel_vel() - target_vel.
    local acc is GRAV_ACC + 0*get_pre_aero_acc() - target_acc.

    set INTERCEPT_TIME to INTERCEPT_TIME - (time:seconds - t_last).
    set INTERCEPT_TIME to newton_one_step_intercept(INTERCEPT_TIME, pos, vel, acc, available_dv).
    set t_last to time:seconds.

    local dv_r is -(vel + pos/INTERCEPT_TIME + 0.5*acc*INTERCEPT_TIME).
    local intercept_pos_raw is pos + vel*INTERCEPT_TIME + 0.5*acc*INTERCEPT_TIME*INTERCEPT_TIME.

    set INTERCEPT_POS to (-ship:facing)*intercept_pos_raw.

    local print_str is  "" +
            "dV avail " + round_dec(available_dv,1) + " m/s" + char(10) +
            "dV req   " + round_dec(dv_r:mag,1) + " m/s" + char(10) +
            "int mag  " + round_dec(INTERCEPT_POS:mag,1) + " m" + char(10) +
            " int x    " + round_dec(INTERCEPT_POS:x,1) + "m" + char(10) +
            " int y    " + round_dec(INTERCEPT_POS:y,1) + "m" + char(10) +
            "int Time " + round_dec(INTERCEPT_TIME,1) + " s" + char(10) +
            "dv " + round_vec((-ship:facing)*dv_r,1) + " " + round_dec(dv_r:mag,1) + " m/s" + char(10) +
            "acc " + round_vec((-ship:facing)*acc,1) + " " + round_dec(acc:mag,1) + " m/s2" + char(10) +
            "NAVMODE     " + (choose "ORB" if AP_NAV_IN_ORBIT else "") +
            (choose "SRF" if AP_NAV_IN_SURFACE else "") + char(10).
    terminal_info_print(print_str, dv_r).
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
        set available_dv to START_DV.
    } else if defined AP_ORB_ENABLED {
        set available_dv to available_dv + ap_me_get_dv():z.
    }

    if launched and (mother_ship:position:mag < SAFE_SEPARATION) {
        set AP_NAV_VEL to ap_nav_get_vessel_vel() -(0.25*ship:airspeed)*(mother_ship:facing:topvector).
        set AP_NAV_ACC to GRAV_ACC.
        set AP_NAV_ATT to lookdirup(ship:facing:forevector, ship:up:forevector).
        if mother_ship:position:mag < SAFE_SEPARATION/2 {
            ap_me_limit_set(20).
        } else {
            ap_me_limit_set(100).
        }.
        return true.
    } else if not (target_ship = -1) and launched {
        local dv_scale is 1.0.
        if INTERCEPT_TIME >= 0 and INTERCEPT_TIME < 12.5 {
            // this line increases weight of error as intercept becomes closer
            // something like this should not be in the code, remove later
            set dv_scale to convex(5.0,1.0,INTERCEPT_TIME/12.5).
        }
        set AP_NAV_VEL to ap_nav_get_vessel_vel() + dv_scale*missile_intercept(target_ship, available_dv).
        set AP_NAV_ACC to GRAV_ACC.
        set AP_NAV_ATT to lookdirup(ship:facing:forevector, ship:up:forevector).
        return true.
    } else if not (target_ship = -1) and not launched {
        missile_intercept(target_ship, available_dv). // just display
        set AP_NAV_VEL to ap_nav_get_vessel_vel().
        set AP_NAV_ACC to V(0,0,0).
        set AP_NAV_ATT to ship:facing.
        return true.
    } else  {
        set INTERCEPT_TIME to 10.
        return false.
    }
}

local t_print is time:seconds.
local function terminal_info_print {
    parameter print_str.
    parameter delta_v.

    if time:seconds - t_print > 0.2 {
        set t_print to time:seconds.
        local line_number is 0.

        local all_lines is "".
        for line in print_str:split(char(10)) {
            set all_lines to all_lines + line:padright(terminal:width).
        }

        local dv_top is ship:facing:topvector*delta_v/delta_v:mag.
        local dv_star is ship:facing:starvector*delta_v/delta_v:mag.
        
        local middle_row is "".
        if dv_star < -0.25 {
            set middle_row to " < < <         ".
        } else if dv_star < -0.125 {
            set middle_row to "   < <         ".
        } else if dv_star < -0.0625 {
            set middle_row to "     <         ".
        } else if dv_star > 0.0625 {
            set middle_row to "         >     ".
        } else if dv_star > 0.125 {
            set middle_row to "         > >   ".
        } else if dv_star > 0.25 {
            set middle_row to "         > > > ".
        } else if abs(dv_top) < 0.0625 {
            set middle_row to "       X       ".
        } else {
            set middle_row to "               ".
        }

        local graphic_rows is list(
        "---------------",
        "       " + (choose "^" if dv_top > 0.25  else " ") + "       ",
        "       " + (choose "^" if dv_top > 0.125  else " ") + "       ",
        "       " + (choose "^" if dv_top > 0.0625  else " ") + "       ",
        middle_row,
        "       " + (choose "V" if dv_top < -0.0625  else " ") + "       ",
        "       " + (choose "V" if dv_top < -0.125  else " ") + "       ",
        "       " + (choose "V" if dv_top < -0.25  else " ") + "       ",
        "---------------").

        for line in graphic_rows {
            set all_lines to all_lines + line:padright(terminal:width).
        }
        print all_lines at (0,0).
    }
}

function ap_nav_missile_status_string {
    local dstr is "".
    local vel_mag is ap_nav_get_hud_vel():mag.
    set dstr to dstr + "/"+round_fig(vel_mag,2) + char(10) +
        "do " + round_dec(INTERCEPT_POS:mag,1) + " m" + char(10) +
        "d+ " + round_dec(INTERCEPT_POS:x,1) + "," + round_dec(INTERCEPT_POS:y,1) + "m" + char(10) +
        "t  " + round_dec(INTERCEPT_TIME,1) + " s".
    return dstr.
}
