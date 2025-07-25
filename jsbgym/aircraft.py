from ctypes import Union
from dataclasses import dataclass
from typing import List, override

KTS_TO_M_PER_S: float = 0.51444
KTS_TO_FT_PER_S: float = 1.6878
@dataclass()
class Aircraft:
    jsbsim_id: str
    flightgear_id: str
    name: str
    cruise_speed_kts: float
    Vs0 : float = None
    Vs1 : float = None
    Vr : float = None
    Vx : float = None
    Vg : float = None
    Vy : float = None
    Va : float = None
    Vno: float = None
    Vne : float = None

    def get_max_distance_m(self, episode_time_s: float) -> float:
        """Estimates the maximum distance this aircraft can travel in an episode"""
        margin = 0.1
        return self.cruise_speed_kts * self.KTS_TO_M_PER_S * episode_time_s * (1 + margin)

    def get_cruise_speed_fps(self) -> float:
        return self.cruise_speed_kts * KTS_TO_FT_PER_S
    
    @override
    def __setattr__(self, key: str, value: float) -> None:
        if key in ["jsbsim_id", "flightgear_id", "name"] and key in self.__dict__:
            raise AttributeError(f"'{key}' is read-only.")

        super().__setattr__(key, value)



c172p = Aircraft("c172p", "c172p", "C172P", 120)
c172x = Aircraft("c172x", "c172p", "C172X", 100, 41, 47, 55, 59, 65, 73, 97, 128, 160)
pa28 = Aircraft("pa28", "PA28-161-180", "PA28", 130)
j3 = Aircraft("J3Cub", "J3Cub", "J3", 70)
f15 = Aircraft("f15", "f15c", "F15", 500)
f16 = Aircraft("f16", "f16-block-52", "F16", 550)
ov10 = Aircraft("OV10", "OV10_USAFE", "OV10", 200)
pc7 = Aircraft("pc7", "pc7", "PC7", 170)
a320 = Aircraft("A320", "A320-200-CFM", "A320", 250)
b747 = Aircraft("B747", "747-400", "B747", 250)
md11 = Aircraft("MD11", "MD-11", "MD11", 250)
dhc6 = Aircraft("DHC6", "dhc6jsb", "DHC6", 170)
c130 = Aircraft("C130", "c130", "C130", 290)
wf = Aircraft("wrightFlyer1903", "wrightFlyer1903-jsbsim", "WF", 25)
ss = Aircraft("Submarine_Scout", "Submarine_Scout", "SS", 40)

available_aircraft: List[Aircraft] = [
    c172p,
    c172x,
    pa28,
    j3,
    f15,
    f16,
    ov10,
    pc7,
    a320,
    b747,
    md11,
    dhc6,
    c130,
    wf,
    ss
]

