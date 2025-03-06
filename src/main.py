import math

from current import Current as Car
from purePursuit import PurePursuit
from history import History
from history import Track

# Initialize classes
history = History()

# Create track as lambda functions
track1 = lambda x: -((x - 5)**2) + 25
track2 = lambda x: math.sin(x)
track3 = lambda x: math.sin(x/5.0) * x/2.0

history.generate_track(xmin=0, xmax=50, points=50, function=track3)

car = Car(lookAheadDistance=1.0, velocity=1.0)
pure_pursuit = PurePursuit()

history.animate(car, pure_pursuit)
