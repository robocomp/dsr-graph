from pyrep.robots.mobiles.holonomic_base import HolonomicBase

class Viriato(HolonomicBase):
    def __init__(sefl, count_ int = 0, distance_from_target: float =0)):
        super().__init__(count, 4, distance_from_target, 'Viriato', 4, 6, 0.035)


