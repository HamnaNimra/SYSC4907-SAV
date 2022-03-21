# A linear range in any dimension
class Range:
    def __init__(self, min: float, max: float):
        self.min = min
        self.max = max

    # Checks if there is an overlap between this range and the given range. It is
    # the responsibility of the caller to ensure that the two ranges are logically
    # of the same dimension. For example, both ranges represent a range on the x-axis.
    def overlaps(self, otherRange: 'Range'):
        return self.min < otherRange.max and self.max > otherRange.min

    def to_string(self):
        return "min={}, max={}".format(self.min, self.max)