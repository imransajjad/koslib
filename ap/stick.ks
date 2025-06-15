
global AP_STICK_ENABLED is true.

local PARAM is get_param(readJson("param.json"), "AP_STICK", lexicon()).

local STICK_GAIN is get_param(PARAM, "STICK_GAIN", 3.0).
local SWAP_ROLL_YAW is get_param(PARAM, "SWAP_ROLL_YAW", false).
local KEYS_LPF_COARSE is get_param(PARAM, "KEYS_LPF_COARSE", 0.100).
local KEYS_LPF_FINE is get_param(PARAM, "KEYS_LPF_FINE", KEYS_LPF_COARSE/5).

local omega_w is V(0,0,0).
local input_state is 0. // 0 na, 1 keys, 0.5 stick

local hpf_y is 0.
local hpf_e is 0.5.
local last_input_x is 0.
local function lpf_from_hpe {
    parameter input_x.

    local LPF_E is 0.02.
    local HPF is 0.5.
    set hpf_y to (-HPF)*hpf_y + input_x - last_input_x.
    set last_input_x to input_x.

    if not (input_x = 0) {
        set hpf_e to (1-LPF_E)*hpf_e + (LPF_E)*(hpf_y^2).
    }
    
    // scale from coarse to fine
    local lpf_ret is convex(KEYS_LPF_COARSE, KEYS_LPF_FINE, sat(2*hpf_e,1.0)).
    // util_hud_push_left("stick_lpf", "hpf_y " + round_dec(hpf_y,3) + char(10) + "hpf_e " + round_dec(hpf_e,3) + char(10) + "lpf_r " + round_dec(lpf_ret,3)).

    if input_x = 0 {
        return 3*lpf_ret.
    } else {
        return lpf_ret.
    }
}

function ap_stick_w {
    parameter u1.
    parameter u2.
    parameter u3.
    
    if SWAP_ROLL_YAW {
        local temp is u3.
        set u3 to u2.
        set u2 to temp.
    }
    local sum_inputs is (u1+u2+u3).
    local max_inputs is max(max(abs(u1),abs(u2)),abs(u3)).

    if not (sum_inputs = 0) and (max_inputs = 1.0) and (mod(sum_inputs, 1.0) = 0) {
        set input_state to 1.0.
    } else if not (max_inputs = 1.0) and not (mod(sum_inputs, 1.0) = 0) {
        set input_state to 0.5.
    } else {
        // do nothing, retain previous.
    }

    if input_state = 1.0 {
        local LPF is lpf_from_hpe(max_inputs).
        set omega_w to (1-LPF)*omega_w + (LPF)*V(u1,u2,u3).
    } else {
        set omega_w to V(sat(STICK_GAIN*u1,1.0),sat(STICK_GAIN*u2,1.0),sat(STICK_GAIN*u3,1.0)).
    }
    return V(omega_w:x,omega_w:y,omega_w:z).
}
