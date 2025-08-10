from enum import Enum

class RollCase(Enum):
    ROLL_LVL = "Starts at close to zero roll, and has to stay at zero roll"
    ROLL_RCVR_30 = "Starts at roll of 30 degrees or less and has to recover to zero roll"
    ROLL_RCVR_60 = "Starts at roll of 30 to 60 degrees and has to recover to zero roll"
    ROLL_30 = "Starts at 0 roll and has to get to a roll from 10 to 30 degrees"
    ROLL_45 = "Starts at 0 roll and has to get to a roll from 30 to 45 degrees"

class PitchCase(Enum):
    PITCH_LVL = "Starts at close to zero pitch, and has to stay at zero pitch"
    PITCH_RCVR_10 = "Starts at pitch of 10 degrees or less and has to recover to zero pitch"
    PITCH_RCVR_M_10 = "Starts at pitch of -10 degrees or less and has to recover to zero pitch"
    PITCH_RCVR_STALL = "Starts at pitch of 15 to 50 degrees and has to recover to zero pitch"
    PITCH_RCVR_DSC = "Starts at pitch of -15 to -50 degrees and has to recover to zero pitch"
    PITCH_CRSE_CLMB = "Starts at 0 pitch and has to get to a pitch less than 10 degrees"
    PITCH_HIGH_CLMB = "Starts at 0 pitch and has to get to a roll from 10 to 45 degrees"

class AirSpeedCase(Enum):
    AS_CRUISE = "Cruise airspeed"
    AS_L_VNO = "Airspeeds from Vno - 10 to Vno"
    AS_VNO_VNE = "Airspeeds from Vno to Vne"
    AS_P_VNE = "Airspeeds greater than Vne"
    AS_S_CRUISE = "Slow cruse airspeeds, like 70-90 knots for a c172"
    AS_APPROACH = "Airspeeds withing 10knts of final approach speed. 50-70knts for c172"
    AS_STALL = "Airspeeds from Vso -5 to Vs1 + 10"

class AltCase(Enum):
    ALT_HOLD = "hold altitude"
    CLB_L_200 = "climb less than 200 ft"
    CLB_200_500 = "climb between 200 and 500 ft"
    CLB_G_500 = "climb above 500 ft"
    DSC_L_200 = "descend less than 200 ft"
    DSC_200_500 = "descend between 200 and 500 ft"
    DSC_G_500 = "descend above 500 ft"

class HdgCase(Enum):
    HDG_HOLD = "hold heading"
    HDG_L_45 = "change heading by less than 45 degrees"
    HDG_45_90 = "change heading between 45 and 90 degrees"
    HDG_90_180 = "change heading between 90 and 180 degrees"

class WindCase(Enum):
    CLM = "No Winds"
    WND_L_5 = "Max crosswind component of less than 5 knots. Steady wind."
    WND_5_10 = "Max crosswind component between 5 and 10 knots. Steady wind."
    WND_10_15 = "Max crosswind component between 10 and 15 knots. Steady wind."
    WND_G_15 = "Max crosswind component greater than 15 knots. Steady."