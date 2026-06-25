
global AP_ME_ENABLED is true.

local METvec is V(0,0,1). // is a vector
local MEIsp is V(0,0,0). // is also a vector is exhaust velocity in multiple directions
local MEMom is V(0,0,0). // is also a vector of moment in ship facing frame

local last_cached is -1.
local lock cached_params_stale to (abs(ship:availablethrust + stage:number - last_cached) > 0.25).
local function ap_me_params {
    // Main Engine Stuff
    // return the ME thrust vector and the Isp vector
    set last_cached to ship:availablethrust + stage:number.
    list engines in engine_list.
    set METvec to V(0,0,0).
    set MEIsp to V(0,0,0).
    set MEMom to V(0,0,0).

    for e in engine_list {
        until e:availablethrust = 0 or (e:availablethrust > 0 and e:isp > 0) {
            // sometimes engine is activated but isp is still zero. wait till valid
            wait 0.
        }

        if e:ignition and e:availablethrust > 0 and e:isp > 0 {
            local evec is e:availablethrust*((-ship:facing)*e:facing:forevector).
            set METvec to METvec + evec.
            set MEIsp to MEIsp + evec/e:isp.
            set MEMom to MEMom + vcrs((-ship:facing)*e:position,evec).
        }
    }
    if MEIsp:x > 0.0001 { set MEIsp:x to METvec:x/MEIsp:x. }
    if MEIsp:y > 0.0001 { set MEIsp:y to METvec:y/MEIsp:y. }
    if MEIsp:z > 0.0001 { set MEIsp:z to METvec:z/MEIsp:z. }

    // print "ME data" +
    //     char(10) +"last " + round_fig(last_cached,3) + 
    //     char(10) +"F " + round_vec(METvec,3) +
    //     char(10) +"Isp " + round_vec(MEIsp,3) +
    //     char(10) +"Mom " + round_vec(MEMom,3).
}

function ap_me_get_thrust
{
    if cached_params_stale {
        ap_me_params().
    }
    return METvec.
}

function ap_me_get_isp
{
    if cached_params_stale {
        ap_me_params().
    }
    return MEIsp.
}

function ap_me_get_mom
{
    if cached_params_stale {
        ap_me_params().
    }
    return MEMom.
}

function ap_me_get_dv
{
    if cached_params_stale {
        ap_me_params().
    }
    local m_engine_prop is 0.
    for r in ship:resources {
        if r:name = "LiquidFuel" or r:name = "Oxidizer" or r:name = "SolidFuel"  {
            set m_engine_prop to m_engine_prop + r:amount*r:density.
        }
    }
    return constant:g0*MEIsp*ln(max(1,ship:mass/(ship:mass - m_engine_prop))).
}

local function get_burn_time_mass {
    parameter delta_v.
    parameter thrust_vector is V(0,0,1).

    local me_thrust is METvec*(thrust_vector:normalized).

    if (me_thrust = 0) {
        return list(0, 0).
    }
    local m_engine_prop is 0.
    local v_e is g0*MEIsp*(thrust_vector:normalized).
    
    for r in ship:resources {
        if r:name = "LiquidFuel" or r:name = "Oxidizer" {
            set m_engine_prop to m_engine_prop + r:amount*r:density.
        }
    }

    local m_me_burn is ship:mass+1.
    if v_e > 0 {
        set m_me_burn to ship:mass*(1 - constant():e^(-delta_v/(v_e))).
    }

    set m_me_burn to min(m_engine_prop, m_me_burn).

    local burn_time is (v_e/me_thrust)*m_me_burn.

    return list(burn_time, m_me_burn).
}

function ap_me_get_burn_time {
    parameter delta_v.
    parameter thrust_vector is V(0,0,1).

    if cached_params_stale {
        ap_me_params().
    }
    return get_burn_time_mass()[0].
}

function ap_me_get_burn_mass {
    parameter delta_v.
    parameter thrust_vector is V(0,0,1).

    if cached_params_stale {
        ap_me_params().
    }
    return get_burn_time_mass()[1].
}

function ap_me_get_burn_delay {
    parameter delta_v.
    parameter thrust_vector is V(0,0,1).

    if cached_params_stale {
        ap_me_params().
    }

    local burn_data is get_burn_time_mass(delta_v, thrust_vector).
    local m_me_time is burn_data[0].
    local m_me_burn is burn_data[1].

    local lambda is ship:mass/(ship:mass - m_me_burn).
    local delay is 0.5*m_me_time.
    if lambda > 1.0 and lambda < 10000 {
        set delay to (lambda/(lambda -1) - 1/ln(lambda))*m_me_time.
    } else if lambda >= 10000 {
        set delay to 1.0*m_me_time.
    }
    return delay.
}

local throttle_zeroed is true.
function ap_me_throttle {
    parameter input_throttle is SHIP:CONTROL:PILOTMAINTHROTTLE.
    if SAS {
        return.
    }
    if not throttle_zeroed and input_throttle = 0 {
        set throttle_zeroed to true.
    } else if throttle_zeroed and input_throttle > 0 {
        set throttle_zeroed to false.
    }

    if throttle_zeroed {
        set SHIP:CONTROL:MAINTHROTTLE to input_throttle.
    } else {
        set SHIP:CONTROL:MAINTHROTTLE to input_throttle.
    }
}

function ap_me_limit_set {
    parameter lim.
    list engines in engine_list.

    for i in engine_list {
        set i:thrustlimit to lim.
    }
}
